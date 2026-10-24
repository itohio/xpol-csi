#pragma once
// Pure C++ DSP: turns each link's stream of raw CSI I/Q frames into a debounced
// motion decision and breathing/heartbeat rates. No ESP-IDF dependencies, so it
// compiles and runs on the host for tests (see host/test_dsp.cpp).
//
// Split for a single-core MCU with no FPU (the Wi-Fi stack shares the core):
//  - LinkProcessor::push()      per frame, CSI consumer task. Integer math on the
//                               16 selected subcarriers only.
//  - LinkProcessor::snapshot()  per tick, under the caller's lock. Cheap: motion
//                               decision + the 1-D breathing component.
//  - analyzeLink() & spectra    per tick, OUTSIDE the lock, on that 1-D series.
//
// Motion follows ESPectre (github.com/PeterkoCZ91/esphome-wifi-csi): per-packet
// spatial spread across subcarriers -> Hampel -> moving variance, threshold =
// P95 x 1.1 of the empty-room calibration, hysteresis + 4/6 voting.
// Vital-sign peaks use a clipped-mean noise floor and an SNR in dB
// (arXiv:2603.10845 eq. 7-9) with thresholds set for <= 1% false alarms on noise.

#include <stdint.h>
#include <array>
#include <algorithm>
#include "fixedvec.h"

#ifndef MAX_SUBCARRIERS
#define MAX_SUBCARRIERS 128
#endif
#ifndef DSP_NSEL
#define DSP_NSEL 16          // subcarriers kept per link (selected across the band)
#endif
#ifndef DSP_DECIM_HZ
#define DSP_DECIM_HZ 20
#endif
#ifndef BREATH_WIN_SEC
#define BREATH_WIN_SEC 24
#endif
#ifndef BREATH_FMIN_HZ
#define BREATH_FMIN_HZ 0.12f
#endif
#ifndef BREATH_FMAX_HZ
#define BREATH_FMAX_HZ 0.70f
#endif
#ifndef BREATH_SNR_DB
#define BREATH_SNR_DB 14.0f
#endif
#ifndef BREATH_IDLE_SEC
#define BREATH_IDLE_SEC 12
#endif
#ifndef HEART_FMIN_HZ
#define HEART_FMIN_HZ 0.75f
#endif
#ifndef HEART_FMAX_HZ
#define HEART_FMAX_HZ 2.50f
#endif
#ifndef HEART_SNR_DB
#define HEART_SNR_DB 12.0f
#endif
#ifndef MOTION_WIN_PKTS
#define MOTION_WIN_PKTS 100
#endif
#ifndef MOTION_MV_DEFAULT
#define MOTION_MV_DEFAULT 1e-4f
#endif
#ifndef MIN_LINK_PKT_RATE
#define MIN_LINK_PKT_RATE 20.0f
#endif
#ifndef MAX_PEAKS
#define MAX_PEAKS 8
#endif
#ifndef SPEC_MAX
#define SPEC_MAX 48
#endif
#ifndef HSPEC_MAX
#define HSPEC_MAX 128  // heart-band spectrum bins (45..150 bpm = 106)
#endif
#ifndef MAX_LINKS
#define MAX_LINKS 3
#endif

// Samples in the breathing window (DSP_DECIM_HZ grid).
#define DSP_BREATH_LEN (BREATH_WIN_SEC * DSP_DECIM_HZ)

// One CSI reading, read in place from the radio's ring: raw int8 I/Q, no
// per-frame float conversion. Only the selected subcarriers' amplitudes are
// ever computed (csi_amp_q4), in integer math.
struct CsiFrame {
    uint32_t      t_us = 0;      // receive timestamp (microseconds)
    int8_t        rssi = 0;
    uint8_t       n_sub = 0;     // valid subcarriers
    uint16_t      gain_q8 = 256; // amplitude gain compensation, Q8 (256 = 1.0)
    const int8_t* iq = nullptr;  // interleaved I/Q, 2*n_sub bytes
    uint8_t       fmt = 0;       // PHY format id (radio.h); a link is one transmitter in one format
};

// floor(sqrt(x)), integer only: the C5 has no FPU, so sqrtf is a libgcc call.
inline uint32_t isqrt32(uint32_t x) {
    uint32_t r = 0, b = 1u << 30;
    while (b > x) b >>= 2;
    while (b) {
        if (x >= r + b) { x -= r + b; r = (r >> 1) + b; } else { r >>= 1; }
        b >>= 2;
    }
    return r;
}

// Stored amplitudes are Q4 (x16). Breathing changes a subcarrier's amplitude by
// ~1-3% (0.2-1 unit at typical 20-40), so whole-unit storage rounds it away.
#define AMP_FRAC_BITS 4

// round(16 * gain * |I + jQ|) of interleaved I/Q pair k, in Q4. |.| is taken as
// isqrt(p << 16) = 256|v| (p <= 2^15 -> <= 2^31), then gain Q8 -> Q4 at the end
// (256|v| * gain_q8 <= 46341 * 65535 < 2^32). Truncation error <= gain/16 Q4 LSB.
inline int16_t csi_amp_q4(const int8_t* iq, int k, uint16_t gain_q8) {
    const int32_t re = iq[2 * k], im = iq[2 * k + 1];
    const uint32_t a = (isqrt32((uint32_t)(re * re + im * im) << 16) * gain_q8 + (1u << 11)) >> 12;
    return (int16_t)(a > 32767 ? 32767 : a);
}

// Runtime-tunable DSP parameters (web "tune" tab, persisted by the firmware).
// Defaults are the config macros. The user-facing fields are floats; integer
// forms used by per-frame/per-tick code are derived in dsp_param_set(), so the
// hot paths never do float math because of tuning. Fields are 32-bit: written
// by the HTTP task, read by the DSP tasks without locking (aligned stores).
struct DspParams {
    float breath_snr_db   = BREATH_SNR_DB;   // detection: counts as a breather / confirms a track
    float heart_snr_db    = HEART_SNR_DB;
    float breath_hyp_db   = 8.0f;            // hypothesis: shown and tracked, not yet a detection
    float heart_hyp_db    = 8.0f;
    float track_confirm_s = 10.0f;           // a track confirms after this long at detection SNR
    float breath_idle_s   = BREATH_IDLE_SEC;
    float min_link_rate   = MIN_LINK_PKT_RATE;
    float thr_scale       = 1.0f;    // effective motion threshold = calibrated x this
    float hysteresis      = 0.7f;    // stay in motion while variance >= this x threshold
    float votes_enter     = 4;       // of the last 6 ticks
    float votes_exit      = 5;
    float hampel_k        = 3.0f;    // outlier = |x - median| > k * 1.4826 * MAD
    float breath_fmin_bpm = BREATH_FMIN_HZ * 60.0f;
    float breath_fmax_bpm = BREATH_FMAX_HZ * 60.0f;
    float heart_fmin_bpm  = HEART_FMIN_HZ * 60.0f;
    float heart_fmax_bpm  = HEART_FMAX_HZ * 60.0f;
    float links_per_board = MAX_LINKS;   // links of each board that may feed the scene (1..MAX_LINKS)
    float hp_window_s     = 8.0f;    // low cut: each subcarrier minus its centered moving average over this (0 = off)
    // derived
    int32_t  hampel_x1e4  = 44478;   // hampel_k * 1.4826 * 1e4
    uint32_t thr_scale_q8 = 256;
    uint32_t hyst_q8      = 179;
};
extern DspParams g_dsp;

struct DspParamInfo {
    const char* name; const char* unit; const char* group;
    float min, max, step;
    float DspParams::* field;
};
// Parameter table; a parameter's id is its index.
const DspParamInfo* dsp_param_table(int* count);
// Clamp to [min,max] (and band-width limits), store, refresh derived fields.
bool dsp_param_set(int id, float value);
void dsp_params_defaults();

// A periodic-component hypothesis (breathing or heartbeat): rate + SNR in dB.
// It is a detection when snr >= g_dsp.breath_snr_db / heart_snr_db.
struct SpectralPeak {
    float rate_bpm = 0;
    float snr = 0;      // dB above the band's noise floor
};

// Per-decision output for a single link (one transmitter MAC).
struct LinkResult {
    bool    valid = false;
    float   motion_index = 0;  // moving variance / threshold: >= 1 means moving
    bool    motion = false;    // debounced decision
    float   pkt_rate = 0;      // CSI frames per second on this link
    int8_t  rssi = 0;
    float   mean_amp = 0;      // mean CSI amplitude (direct-path coupling proxy, for RF alignment)
    FixedVec<SpectralPeak, MAX_PEAKS> breaths;   // hypotheses, snr >= breath_hyp_db
    FixedVec<SpectralPeak, MAX_PEAKS> hearts;    // hypotheses, snr >= heart_hyp_db
};

// Everything analyzeLink() needs besides the breathing series. Taken under the
// caller's lock by LinkProcessor::snapshot().
struct LinkSnapshot {
    bool   valid = false;
    int8_t rssi = 0;
    float  mean_amp = 0;
    float  pkt_rate = 0;
    float  motion_index = 0;
    bool   motion = false;
    float  idle_s = 0;      // seconds since the last motion decision
    int    n = 0;           // breathing samples in y (0 = not still long enough)
};

// Processes the CSI stream of a single bistatic link.
class LinkProcessor {
public:
    LinkProcessor();
    void reset();
    // Clear all signal state (rings, motion window, votes) but keep the
    // calibrated threshold: the web "reset buffers" button.
    void resetBuffers();

    // Feed one CSI frame (consumer task). Integer-only on the selected subcarriers.
    void push(const CsiFrame& f);

    // Empty-room calibration: motion threshold = P95 x 1.1 of the moving
    // variance seen between begin and finalize (ESPectre threshold.h).
    void beginCalibration();
    void finalizeCalibration();
    bool calibrated() const { return calibrated_; }
    // Motion threshold (moving variance, Q32) for persisting a calibration and
    // restoring it after a reboot. Floored so a zero-variance calibration cannot
    // make any change count as motion.
    uint64_t motionThreshold() const { return thr_q32_; }
    void setMotionThreshold(uint64_t q32) { thr_q32_ = std::max(q32, kMotionFloorQ32); calibrated_ = true; }

    // Per tick, under the caller's lock (cheap). Advances the debounced motion
    // decision at time now_us and, once the link has been still for
    // BREATH_IDLE_SEC, writes the breathing component to y[DSP_BREATH_LEN].
    LinkSnapshot snapshot(int64_t now_us, float* y);

    float meanAmp() const;    // mean amplitude, in amplitude units (RF coupling proxy)
    int8_t rssi() const { return (int8_t)((rssi_ema_q8_ + 128) >> 8); }   // per-frame EMA (~32 frames), dBm
    float pktRate() const { return pkt_rate_; }   // frames/s, refreshed by snapshot() each second
    int nSel() const { return n_sel_; }
    // Breathing-grid rows written since the link opened (monotonic): a reader's
    // position in this link's row stream.
    uint32_t rowsTotal() const { return rows_total_; }
    // Rows after *cursor (oldest first, row-major [row][nSel()] Q4), at most
    // max_rows; advances *cursor. A cursor older than the window skips ahead.
    int rowsSince(uint32_t* cursor, int16_t* out, int max_rows) const;

private:
    static constexpr int kBreathLen = DSP_BREATH_LEN;
    static constexpr int kHampel = 7;

    void selectSubcarriers(const CsiFrame& f);
    void flushBreathSlot();
    void motionSample(int32_t spread_q16);
    // First principal component of the linearly detrended selected subcarriers
    // over the newest max_rows of the breathing window, into y[kBreathLen].
    // Returns sample count.
    int breathComponent(float* y, int max_rows) const;

    uint8_t n_sub_ = 0;
    // sel_[j] = original subcarrier index for the j-th stored column.
    std::array<uint8_t, DSP_NSEL> sel_{};
    uint8_t n_sel_ = 0;
    bool seeded_ = false;
    int8_t last_rssi_ = 0;
    int32_t rssi_ema_q8_ = 0;      // RSSI EMA in Q8 (alpha 1/32), seeded by the first frame
    bool rate_ok_ = false;         // frame rate high enough to sense (with hysteresis)

    // Breathing ring: per-slot mean of every frame, selected subcarriers, Q4.
    // Fixed arrays => no heap, ~15 KB/link.
    int16_t bring_[kBreathLen][DSP_NSEL] = {};
    int bhead_ = 0;
    int bcount_ = 0;
    uint32_t rows_total_ = 0;
    int64_t last_breath_slot_ = -1;
    uint32_t bacc_[DSP_NSEL] = {};   // Q4 amplitude sums of the current breathing slot
    uint16_t bacc_n_ = 0;

    // Motion, all integer: Hampel window of the per-frame spread (CV, Q16), then
    // a sliding variance (Q32) over MOTION_WIN_PKTS frames with exact sums.
    static constexpr uint64_t kSpreadMax = 1u << 20;     // CV <= 16
    static constexpr uint64_t kMotionDefaultQ32 = (uint64_t)(MOTION_MV_DEFAULT * 4294967296.0);
    static constexpr uint64_t kMotionFloorQ32 = kMotionDefaultQ32 / 20;
    int32_t hamp_[kHampel] = {};
    int hamp_n_ = 0, hamp_i_ = 0;
    int32_t mvw_[MOTION_WIN_PKTS] = {};
    int mv_i_ = 0, mv_n_ = 0;
    int64_t mv_s_ = 0;
    uint64_t mv_ss_ = 0;
    uint64_t mv_q32_ = 0;
    uint32_t amp_ema_ = 0;           // mean amplitude, Q4 << 6 (EMA, 1/64 per frame)
    uint32_t frames_ = 0;

    // Decision state (snapshot()).
    uint64_t thr_q32_ = kMotionDefaultQ32;
    bool calibrating_ = false, calibrated_ = false;
    FixedVec<uint32_t, 96> calib_;   // moving variance samples at 4 Hz, Q32
    bool motion_ = false;
    uint8_t votes_ = 0;              // last 6 raw decisions, bit 0 = newest
    int64_t last_motion_us_ = -1;    // also the start of the idle timer
    int64_t rate_t_us_ = -1;
    uint32_t rate_frames_ = 0;
    float pkt_rate_ = 0;
};

// Breathing (and, while breathing and still, heartbeat) hypotheses of one link
// from its snapshot and breathing series. Pure, lock-free; uses one static
// scratch buffer, so call from one task only.
LinkResult analyzeLink(const LinkSnapshot& s, const float* y);

// Web views of a breathing series y[0..n): band spectrum (1 bin per bpm from
// BREATH_FMIN_HZ*60, Hann-windowed), waveform over the whole window
// (block-averaged to <= maxn points, ~unit scale), and the phase 0..1 of the
// oscillation at rate_bpm at the newest sample.
FixedVec<float, SPEC_MAX> breathSpectrum(const float* y, int n);
FixedVec<float, HSPEC_MAX> heartSpectrum(const float* y, int n);   // 45..150 bpm at the grid rate
// Mean of the newest `last` samples of y: one point of the breathing waveform
// per tick; the web builds the curve.
float breathWavePoint(const float* y, int n, int last);
float breathPhase(const float* y, int n, float rate_bpm);

// How useful a link's CSI is, in dB: per-frame SNR proxy (RSSI above -100 dBm,
// capped at -15 dBm where the receiver compresses and more power gains
// nothing) plus the averaging gain of its frame rate (10 log10 frames/s). Used
// for link admission and for choosing which links feed the scene.
float link_quality_db(float rate_hz, float rssi_dbm);

// Fuses several links into a scene-level hypothesis.
struct SceneResult {
    bool  presence = false;
    float motion_index = 0;        // max over links
    int   breather_count = 0;          // clusters at detection SNR
    FixedVec<SpectralPeak, MAX_PEAKS> breaths; // all hypotheses, clustered across links
    int   heart_count = 0;             // clusters at detection SNR
    FixedVec<SpectralPeak, MAX_PEAKS> hearts;
    int   active_links = 0;
};

// Cluster per-link breathing peaks into distinct breathers.
// Two peaks within tol_bpm are the same person seen on different links.
SceneResult fuseLinks(const FixedVec<LinkResult, MAX_LINKS * 2>& links, float tol_bpm = 2.5f);
