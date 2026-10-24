#include "dsp.h"
#include <math.h>
#include <string.h>
#include <algorithm>

// floor(sqrt(x)) for 64-bit x (per-frame spatial spread; integer only).
static uint32_t isqrt64(uint64_t x) {
    uint64_t r = 0, b = 1ull << 62;
    while (b > x) b >>= 2;
    while (b) {
        if (x >= r + b) { x -= r + b; r = (r >> 1) + b; } else { r >>= 1; }
        b >>= 2;
    }
    return (uint32_t)r;
}

// ---------------------------------------------------------------------------
// Runtime parameters (web tune tab)
// ---------------------------------------------------------------------------
DspParams g_dsp;

static const DspParamInfo kParams[] = {
    {"breath SNR (detect)", "dB",     "breathing", 6.0f, 30.0f, 0.5f,  &DspParams::breath_snr_db},
    {"breath SNR (hypothesis)", "dB", "breathing", 3.0f, 20.0f, 0.5f,  &DspParams::breath_hyp_db},
    {"confirm after",       "s",      "breathing", 2.0f, 60.0f, 1.0f,  &DspParams::track_confirm_s},
    {"breath band min",     "bpm",    "breathing", 4.0f, 30.0f, 1.0f,  &DspParams::breath_fmin_bpm},
    {"breath band max",     "bpm",    "breathing", 10.0f, 60.0f, 1.0f, &DspParams::breath_fmax_bpm},
    {"still before vitals", "s",      "breathing", 0.0f, 24.0f, 1.0f,  &DspParams::breath_idle_s},
    {"low-cut window",      "s",      "breathing", 0.0f, 20.0f, 1.0f,  &DspParams::hp_window_s},
    {"heart SNR (detect)",  "dB",     "heart",     6.0f, 30.0f, 0.5f,  &DspParams::heart_snr_db},
    {"heart SNR (hypothesis)", "dB",  "heart",     3.0f, 20.0f, 0.5f,  &DspParams::heart_hyp_db},
    {"heart band min",      "bpm",    "heart",     30.0f, 100.0f, 1.0f, &DspParams::heart_fmin_bpm},
    {"heart band max",      "bpm",    "heart",     60.0f, 200.0f, 1.0f, &DspParams::heart_fmax_bpm},
    {"threshold scale",     "x cal",  "motion",    0.2f, 5.0f, 0.05f,  &DspParams::thr_scale},
    {"hysteresis",          "x thr",  "motion",    0.3f, 1.0f, 0.05f,  &DspParams::hysteresis},
    {"votes to enter",      "of 6",   "motion",    1.0f, 6.0f, 1.0f,   &DspParams::votes_enter},
    {"votes to exit",       "of 6",   "motion",    1.0f, 6.0f, 1.0f,   &DspParams::votes_exit},
    {"hampel k",            "MAD",    "motion",    1.0f, 10.0f, 0.25f, &DspParams::hampel_k},
    {"min link rate",       "fr/s",   "link",      0.0f, 100.0f, 1.0f, &DspParams::min_link_rate},
    {"links per board",     "max",    "link",      1.0f, (float)MAX_LINKS, 1.0f, &DspParams::links_per_board},
};

const DspParamInfo* dsp_param_table(int* count) {
    *count = (int)(sizeof(kParams) / sizeof(kParams[0]));
    return kParams;
}

static void derive_params() {
    g_dsp.hampel_x1e4 = (int32_t)lrintf(g_dsp.hampel_k * 14826.0f);
    g_dsp.thr_scale_q8 = (uint32_t)lrintf(g_dsp.thr_scale * 256.0f);
    g_dsp.hyst_q8 = (uint32_t)lrintf(g_dsp.hysteresis * 256.0f);
}

// Keep a band at least 4 bpm wide and within its bin buffer, moving the edge
// the user did not touch.
static void fix_band(float& lo, float& hi, bool moved_lo, float max_width) {
    if (moved_lo) hi = std::min(std::max(hi, lo + 4.0f), lo + max_width);
    else          lo = std::max(std::min(lo, hi - 4.0f), hi - max_width);
}

bool dsp_param_set(int id, float v) {
    int n; const DspParamInfo* t = dsp_param_table(&n);
    if (id < 0 || id >= n || !(v == v)) return false;
    const DspParamInfo& p = t[id];
    v = std::min(std::max(v, p.min), p.max);
    if (p.step >= 1.0f) v = roundf(v);
    g_dsp.*(p.field) = v;
    if (p.field == &DspParams::breath_fmin_bpm || p.field == &DspParams::breath_fmax_bpm)
        fix_band(g_dsp.breath_fmin_bpm, g_dsp.breath_fmax_bpm, p.field == &DspParams::breath_fmin_bpm, SPEC_MAX - 1);
    if (p.field == &DspParams::heart_fmin_bpm || p.field == &DspParams::heart_fmax_bpm)
        fix_band(g_dsp.heart_fmin_bpm, g_dsp.heart_fmax_bpm, p.field == &DspParams::heart_fmin_bpm, 127.0f);
    derive_params();
    return true;
}

void dsp_params_defaults() {
    g_dsp = DspParams();
    derive_params();
}

LinkProcessor::LinkProcessor() {}

void LinkProcessor::reset() {
    n_sub_ = 0; n_sel_ = 0; seeded_ = false; last_rssi_ = 0; rssi_ema_q8_ = 0;
    bhead_ = 0; bcount_ = 0; rows_total_ = 0; last_breath_slot_ = -1;
    memset(bacc_, 0, sizeof(bacc_)); bacc_n_ = 0;
    hamp_n_ = 0; hamp_i_ = 0;
    mv_i_ = 0; mv_n_ = 0; mv_s_ = 0; mv_ss_ = 0; mv_q32_ = 0;
    amp_ema_ = 0; frames_ = 0;
    thr_q32_ = kMotionDefaultQ32; calibrating_ = false; calibrated_ = false; calib_.clear();
    motion_ = false; votes_ = 0; last_motion_us_ = -1;
    rate_t_us_ = -1; rate_frames_ = 0; pkt_rate_ = 0; rate_ok_ = false;
}

void LinkProcessor::beginCalibration() {
    calibrating_ = true;
    calib_.clear();
}

// Threshold = P95 x 1.1 of the moving variance over the calibration period
// (ESPectre threshold.h). Too few samples: keep the previous threshold.
void LinkProcessor::finalizeCalibration() {
    calibrating_ = false;
    if (calib_.size() < 8) return;
    std::sort(calib_.begin(), calib_.end());
    const int n = (int)calib_.size();
    const int k = std::min(n - 1, (n * 95 + 99) / 100 - 1);
    setMotionThreshold((uint64_t)calib_[k] + calib_[k] / 10);
    calibrated_ = true;
    motion_ = false; votes_ = 0;
}

// Pick up to DSP_NSEL subcarriers carrying signal, spread across the band.
// Compared in the power domain: amp > 0.2*max  <=>  25*p > p_max (no sqrt).
void LinkProcessor::selectSubcarriers(const CsiFrame& f) {
    n_sel_ = 0;
    if (n_sub_ == 0) return;
    auto power = [&](int k) {
        const int32_t re = f.iq[2 * k], im = f.iq[2 * k + 1];
        return (uint32_t)(re * re + im * im);
    };
    uint32_t pmax = 0;
    for (int k = 0; k < n_sub_; k++) pmax = std::max(pmax, power(k));
    if (pmax == 0) return;
    uint8_t cand[MAX_SUBCARRIERS]; int nc = 0;
    for (int k = 0; k < n_sub_; k++)
        if (25 * power(k) > pmax) cand[nc++] = (uint8_t)k;
    if (nc == 0) return;
    if (nc <= DSP_NSEL) {
        for (int i = 0; i < nc; i++) sel_[n_sel_++] = cand[i];
    } else {
        for (int i = 0; i < DSP_NSEL; i++) sel_[n_sel_++] = cand[(i * nc) / DSP_NSEL];
    }
}

void LinkProcessor::push(const CsiFrame& f) {
    if (f.n_sub == 0) return;
    if (!seeded_) n_sub_ = f.n_sub;
    if (f.n_sub != n_sub_) return;      // ignore format changes mid-window
    last_rssi_ = f.rssi;
    rssi_ema_q8_ = !seeded_ ? (int32_t)f.rssi * 256 : rssi_ema_q8_ + (((int32_t)f.rssi * 256 - rssi_ema_q8_) >> 5);
    if (!seeded_) {
        // Select from the first frame that carries signal; an all-zero frame
        // must not leave the link with no subcarriers forever.
        selectSubcarriers(f);
        seeded_ = n_sel_ > 0;
        if (!seeded_) return;
    }
    frames_++;

    // Breathing grid (DSP_DECIM_HZ): average every frame of the slot. Empty
    // slots repeat the previous row, so the ring keeps a uniform time base
    // (skipping them would compress time and scramble the spectrum).
    const int64_t bslot = (int64_t)(f.t_us / (1000000 / DSP_DECIM_HZ));
    if (bslot != last_breath_slot_) {
        const int64_t gap = last_breath_slot_ < 0 ? 0 : bslot - last_breath_slot_ - 1;
        flushBreathSlot();
        for (int64_t g = 0; g < std::min<int64_t>(gap, kBreathLen) && bcount_ > 0; g++) {
            memcpy(bring_[bhead_], bring_[(bhead_ + kBreathLen - 1) % kBreathLen], sizeof(bring_[0]));
            bhead_ = (bhead_ + 1) % kBreathLen;
            if (bcount_ < kBreathLen) bcount_++;
            rows_total_++;
        }
        last_breath_slot_ = bslot;
    }

    uint32_t s = 0; uint64_t ss = 0;
    for (int j = 0; j < n_sel_; j++) {
        const uint32_t a = (uint32_t)csi_amp_q4(f.iq, sel_[j], f.gain_q8);
        bacc_[j] += a; s += a; ss += (uint64_t)a * a;
    }
    bacc_n_++;

    const uint32_t m = n_sel_, mean = s / m;
    amp_ema_ = (uint32_t)((int32_t)amp_ema_ + (((int32_t)(mean << 6) - (int32_t)amp_ema_) >> 6));
    if (mean == 0) return;
    // Spatial spread across subcarriers as a coefficient of variation: gain-
    // invariant, which ESPectre uses when AGC is not locked. var is Q8 (Q4^2),
    // isqrt(var << 8) the std in Q8, and std / (16 mean) in Q16 is the spread.
    const uint64_t var = (ss * m - (uint64_t)s * s) / ((uint64_t)m * m);
    const uint64_t cv_q16 = ((uint64_t)isqrt64(var << 8) << 12) / mean;
    motionSample((int32_t)std::min<uint64_t>(cv_q16, kSpreadMax));
}

// Hampel (causal, window 7, 3 scaled MADs: replace outliers by the median), then
// a sliding variance over MOTION_WIN_PKTS frames (ESPectre csi_filters.cpp,
// mvs_detector.cpp). All integer: x is Q16 and <= 2^20, so x^2 < 2^40 and the
// window sums are exact in 64 bits (no drift).
void LinkProcessor::motionSample(int32_t x) {
    hamp_[hamp_i_] = x;
    hamp_i_ = (hamp_i_ + 1) % kHampel;
    if (hamp_n_ < kHampel) hamp_n_++;
    if (hamp_n_ == kHampel) {
        int32_t w[kHampel];
        memcpy(w, hamp_, sizeof(w));
        std::nth_element(w, w + kHampel / 2, w + kHampel);
        const int32_t med = w[kHampel / 2];
        for (int i = 0; i < kHampel; i++) w[i] = abs(hamp_[i] - med);
        std::nth_element(w, w + kHampel / 2, w + kHampel);
        // |x - med| > 3 * 1.4826 * MAD, scaled by 10^4.
        if ((int64_t)abs(x - med) * 10000 > (int64_t)g_dsp.hampel_x1e4 * w[kHampel / 2]) x = med;
    }

    if (mv_n_ == MOTION_WIN_PKTS) {
        const int64_t o = mvw_[mv_i_];
        mv_s_ -= o; mv_ss_ -= (uint64_t)(o * o);
    } else {
        mv_n_++;
    }
    mvw_[mv_i_] = x;
    mv_s_ += x; mv_ss_ += (uint64_t)((int64_t)x * x);
    mv_i_ = (mv_i_ + 1) % MOTION_WIN_PKTS;
    // Variance in Q32: (n*Sxx - Sx^2) / n^2; n*Sxx < 100 * 100 * 2^40 < 2^54.
    const uint64_t n = (uint64_t)mv_n_;
    const uint64_t num = mv_ss_ * n - (uint64_t)(mv_s_ * mv_s_);
    mv_q32_ = num / (n * n);
}

// Close the breathing slot: store the mean of its frames (rounded, Q4).
void LinkProcessor::flushBreathSlot() {
    if (bacc_n_ == 0) return;
    for (int j = 0; j < n_sel_; j++) {
        bring_[bhead_][j] = (int16_t)((bacc_[j] + bacc_n_ / 2) / bacc_n_);
        bacc_[j] = 0;
    }
    bacc_n_ = 0;
    bhead_ = (bhead_ + 1) % kBreathLen;
    if (bcount_ < kBreathLen) bcount_++;
    rows_total_++;
}

LinkSnapshot LinkProcessor::snapshot(int64_t now_us, float* y) {
    if (rate_t_us_ < 0) {
        rate_t_us_ = now_us; rate_frames_ = frames_;
        last_motion_us_ = now_us;   // the idle timer starts when the link appears
    } else if (now_us - rate_t_us_ >= 1000000) {
        // Frame rate: EMA over ~5 s (alpha 0.2 per 1 s update), so bursty
        // transmitters (a router's data) do not flip in and out of use.
        const float r = (float)(frames_ - rate_frames_) * 1e6f / (float)(now_us - rate_t_us_);
        pkt_rate_ = pkt_rate_ > 0 ? pkt_rate_ + 0.2f * (r - pkt_rate_) : r;
        rate_t_us_ = now_us; rate_frames_ = frames_;
    }

    // A link too sparse for a meaningful 100-frame variance or a filled
    // breathing grid is not used for sensing (threshold tunable: min link rate),
    // with hysteresis: in at min link rate, out below 75 % of it.
    rate_ok_ = pkt_rate_ >= (rate_ok_ ? 0.75f : 1.0f) * g_dsp.min_link_rate;
    const bool full = mv_n_ >= MOTION_WIN_PKTS / 2 && rate_ok_;
    const uint64_t thr = std::max<uint64_t>((thr_q32_ * g_dsp.thr_scale_q8) >> 8, 1);
    if (full) {
        if (calibrating_ && calib_.size() < calib_.capacity()) calib_.push_back((uint32_t)std::min<uint64_t>(mv_q32_, UINT32_MAX));
        // Hysteresis: enter above T, stay while >= hysteresis x T; then votes over
        // the last 6 ticks to enter / leave (ESPectre: 0.7, 4-of-6, 5-of-6).
        const bool raw = motion_ ? (mv_q32_ << 8) >= thr * g_dsp.hyst_q8 : mv_q32_ > thr;
        votes_ = (uint8_t)(((votes_ << 1) | (raw ? 1 : 0)) & 0x3F);
        const int on = __builtin_popcount(votes_);
        if (!motion_ && on >= (int)g_dsp.votes_enter) motion_ = true;
        else if (motion_ && 6 - on >= (int)g_dsp.votes_exit) motion_ = false;
    }
    if (motion_) last_motion_us_ = now_us;

    LinkSnapshot s;
    s.valid = full;
    s.rssi = rssi();
    s.mean_amp = meanAmp();
    s.pkt_rate = pkt_rate_;
    s.motion_index = (float)mv_q32_ / (float)thr;
    s.motion = motion_;
    s.idle_s = (float)(now_us - last_motion_us_) * 1e-6f;
    // Vital signs only on a still window: gross motion swamps the breathing band.
    // The window is only what was recorded since motion ended (the 24 s ring can
    // still hold the motion transient, which would dominate PC1 and the spectrum);
    // analyzeLink needs >= 12 s of it, so vitals appear 12-24 s after settling.
    const int since_still = (int)std::min<int64_t>((now_us - last_motion_us_) / (1000000 / DSP_DECIM_HZ), kBreathLen);
    s.n = (full && !motion_ && s.idle_s >= g_dsp.breath_idle_s) ? breathComponent(y, since_still) : 0;
    return s;
}

void LinkProcessor::resetBuffers() {
    const uint64_t thr = thr_q32_;
    const bool cal = calibrated_;
    reset();
    thr_q32_ = thr; calibrated_ = cal;
}

float LinkProcessor::meanAmp() const {
    return (float)amp_ema_ / (float)(1 << (6 + AMP_FRAC_BITS));
}

// Breathing moves each subcarrier's amplitude with its own sign and depth
// (multipath), so a plain mean across subcarriers partly cancels it. The first
// principal component weights and sign-aligns them.
//
// Low frequencies are removed first: each subcarrier minus its centered moving
// average over hp_window_s (x - MA_L, gain 1 - sinc(f L)). Slow intensity
// changes (drift, AGC and gain steps, someone shifting nearby) otherwise carry
// most of the variance, so PC1 follows them and their 1/f skirt fills the
// breathing band. With L = 8 s: DC removed, 1.2 bpm -28 dB, 3 bpm -12 dB,
// 7 bpm -0.7 dB. Integer only: running window sums per subcarrier and one
// division per sample, straight off the ring; a 16x16 power iteration; a Q12
// projection.
int LinkProcessor::breathComponent(float* y, int max_rows) const {
    const int n = std::min(bcount_, max_rows), m = n_sel_;
    if (n < DSP_DECIM_HZ * 2 || m == 0) return 0;
    auto row = [&](int i) { return bring_[(bhead_ - n + i + 2 * kBreathLen) % kBreathLen]; };
    const int h = (int)lrintf(g_dsp.hp_window_s * DSP_DECIM_HZ * 0.5f);   // half window, rows

    // Visit rows 0..n-1 with their high-passed values hp[j] = x - round(mean of
    // rows [i-h, i+h] clipped to the window). |hp| < 2^16 (Q4 amplitudes < 2^15).
    auto for_each_hp = [&](auto&& f) {
        int32_t S[DSP_NSEL] = {}, hp[DSP_NSEL];
        int lo = 0, hi = -1;
        for (int i = 0; i < n; i++) {
            if (h > 0) {
                while (hi < std::min(n - 1, i + h)) { ++hi; const int16_t* a = row(hi); for (int j = 0; j < m; j++) S[j] += a[j]; }
                while (lo < i - h) { const int16_t* a = row(lo); for (int j = 0; j < m; j++) S[j] -= a[j]; ++lo; }
            }
            const int cnt = hi - lo + 1;
            const int16_t* x = row(i);
            for (int j = 0; j < m; j++)
                hp[j] = h > 0 ? x[j] - (S[j] >= 0 ? (S[j] + cnt / 2) / cnt : -((-S[j] + cnt / 2) / cnt)) : x[j];
            f(i, hp);
        }
    };

    // Sums for mean and covariance (products < 2^32 in int64, n <= 480 terms).
    // Symmetric matrices are stored as packed upper triangles to keep this off
    // the caller's stack budget.
    constexpr int kTri = DSP_NSEL * (DSP_NSEL + 1) / 2;
    auto tri = [](int j, int k) { return j * DSP_NSEL - j * (j + 1) / 2 + k; };   // j <= k
    int64_t s[DSP_NSEL] = {}, sxx[kTri] = {};
    for_each_hp([&](int, const int32_t* x) {
        for (int j = 0; j < m; j++) {
            s[j] += x[j];
            for (int k = j; k < m; k++) sxx[tri(j, k)] += (int64_t)x[j] * x[k];
        }
    });
    double mean[DSP_NSEL];
    for (int j = 0; j < m; j++) mean[j] = (double)s[j] / n;
    auto cov = [&](int j, int k) { return (double)sxx[tri(j, k)] - (double)s[j] * s[k] / n; };
    double cmax = 0;
    for (int j = 0; j < m; j++) for (int k = j; k < m; k++) cmax = std::max(cmax, fabs(cov(j, k)));
    if (cmax <= 0) return 0;
    // Covariance in Q30 (|c| <= 1), eigenvector in Q15: the iteration is integer.
    int32_t cq[kTri];
    const double sc = (double)(1 << 30) / cmax;
    for (int j = 0; j < m; j++) for (int k = j; k < m; k++) cq[tri(j, k)] = (int32_t)llrint(cov(j, k) * sc);

    // Power iteration for the dominant eigenvector: |C v| <= 16 * 2^30 * 2^15 = 2^49,
    // >> 20 leaves <= 2^29, so the squared norm fits 64 bits.
    int32_t v[DSP_NSEL];
    for (int j = 0; j < m; j++) v[j] = 1 << 13;
    for (int it = 0; it < 30; it++) {
        int64_t w[DSP_NSEL];
        uint64_t nrm2 = 0;
        for (int j = 0; j < m; j++) {
            int64_t a = 0;
            for (int k = 0; k < m; k++) a += (int64_t)cq[j <= k ? tri(j, k) : tri(k, j)] * v[k];
            w[j] = a >> 20;
            nrm2 += (uint64_t)(w[j] * w[j]);
        }
        const uint32_t nrm = isqrt64(nrm2);
        if (nrm == 0) return 0;
        for (int j = 0; j < m; j++) v[j] = (int32_t)((w[j] * 32768) / (int64_t)nrm);
    }
    // Stable sign: largest loading positive, so the trace does not flip between updates.
    int big = 0;
    for (int j = 1; j < m; j++) if (abs(v[j]) > abs(v[big])) big = j;
    if (v[big] < 0) for (int j = 0; j < m; j++) v[j] = -v[j];

    // y_i = sum_j v_j (hp_ij - mean_j), the sum over hp in Q12 ints
    // (|v_j| <= 1: 16 * 2^16 * 2^12 = 2^32 needs int64).
    int32_t wq[DSP_NSEL];
    double c0 = 0;
    for (int j = 0; j < m; j++) { wq[j] = v[j] >> 3; c0 += v[j] / 32768.0 * mean[j]; }
    for_each_hp([&](int i, const int32_t* x) {
        int64_t acc = 0;
        for (int j = 0; j < m; j++) acc += (int64_t)wq[j] * x[j];
        y[i] = (float)acc * (1.0f / 4096.0f) - (float)c0;
    });
    return n;
}

int LinkProcessor::rowsSince(uint32_t* cursor, int16_t* out, int max_rows) const {
    const int m = n_sel_;
    if (m == 0 || max_rows <= 0) return 0;
    // Rows older than the window are gone: a lagging reader skips ahead.
    if (rows_total_ - *cursor > (uint32_t)bcount_) *cursor = rows_total_ - (uint32_t)bcount_;
    const int n = (int)std::min<uint32_t>(rows_total_ - *cursor, (uint32_t)max_rows);
    for (int r = 0; r < n; r++) {
        const int back = (int)(rows_total_ - (*cursor + (uint32_t)r));   // 1 = newest row
        memcpy(out + r * m, bring_[(bhead_ - back + 2 * kBreathLen) % kBreathLen], sizeof(int16_t) * m);
    }
    *cursor += (uint32_t)n;
    return n;
}

// ---------------------------------------------------------------------------
// Spectral analysis of a link's 1-D breathing series (outside any lock).
// ---------------------------------------------------------------------------

static int32_t s_x[DSP_BREATH_LEN];   // main-task scratch (see analyzeLink)

static constexpr int kBreathDecim = DSP_DECIM_HZ / 5;        // breathing band analysed at 5 Hz
static_assert(DSP_DECIM_HZ % 5 == 0 && DSP_DECIM_HZ >= 10, "grid must be a multiple of 5 Hz and cover the heart band");
static constexpr float kBinHz = 1.0f / 60.0f;                // 1 bin per bpm
static constexpr int kHeartMaxBins = 128;
// Bin counts follow the (tunable) bands; dsp_param_set() keeps them in range.
static int breathBins() { return std::min(SPEC_MAX, (int)(g_dsp.breath_fmax_bpm - g_dsp.breath_fmin_bpm) + 1); }
static int heartBins() { return std::min(kHeartMaxBins, (int)(g_dsp.heart_fmax_bpm - g_dsp.heart_fmin_bpm) + 1); }

// y[0..n) -> x: block-mean decimated by d (boxcar anti-alias, newest kept),
// optionally Hann-windowed, scaled to |x| <= 2^15. One float pass per call;
// everything per bin after this is integer. Returns length.
static int prep(const float* y, int n, int d, bool window, int32_t* x) {
    const int m = n / d, start = n - m * d;
    float mx = 0;
    for (int i = 0; i < n; i++) mx = std::max(mx, fabsf(y[i]));
    if (m < 2 || mx <= 0) return 0;
    const float sc = 32767.0f / (mx * d);
    const float th = 2.0f * (float)M_PI / (float)(m - 1), cr = cosf(th), sr = sinf(th);
    float c = 1.0f, s = 0.0f;   // Hann by rotation recurrence: no cosf per sample
    for (int i = 0; i < m; i++) {
        float a = 0;
        for (int k = 0; k < d; k++) a += y[start + i * d + k];
        x[i] = (int32_t)lrintf(a * sc * (window ? 0.5f - 0.5f * c : 1.0f));
        const float c2 = c * cr - s * sr;
        s = s * cr + c * sr; c = c2;
    }
    return m;
}

// Integer Goertzel: Q29 coefficient, int32 state, 64-bit products. The state is
// bounded by n|x|/sin(w) < 480 * 2^15 / sin(2pi 0.12/5) < 2^27 for these bands.
struct GState { int32_t s1, s2; int32_t c; };
static GState goertzel(const int32_t* x, int n, float f, float fs) {
    GState g{0, 0, (int32_t)lrintf(2.0f * cosf(2.0f * (float)M_PI * f / fs) * (float)(1 << 29))};
    for (int i = 0; i < n; i++) {
        const int32_t s0 = x[i] + (int32_t)(((int64_t)g.c * g.s1) >> 29) - g.s2;
        g.s2 = g.s1; g.s1 = s0;
    }
    return g;
}
static float gpower(const GState& g) {   // once per bin
    const int64_t cs = ((int64_t)g.c * g.s1) >> 29;
    return (float)((double)g.s1 * g.s1 + (double)g.s2 * g.s2 - (double)cs * g.s2);
}

// Hann-windowed breathing-band periodogram of y: P[b] at breath_fmin + b bpm.
static int breathPeriodogram(const float* y, int n, float* P) {
    const int m = prep(y, n, kBreathDecim, true, s_x);
    const float fs = (float)DSP_DECIM_HZ / kBreathDecim, f0 = g_dsp.breath_fmin_bpm / 60.0f;
    const int nb = breathBins();
    for (int b = 0; b < nb; b++) P[b] = m ? gpower(goertzel(s_x, m, f0 + b * kBinHz, fs)) : 0.0f;
    return nb;
}

// Hann-windowed heart-band periodogram at the full grid rate: H[b] at heart_fmin + b bpm.
static int heartPeriodogram(const float* y, int n, float* H) {
    const int m = prep(y, n, 1, true, s_x), nh = heartBins();
    const float h0 = g_dsp.heart_fmin_bpm / 60.0f;
    for (int b = 0; b < nh; b++)
        H[b] = m ? gpower(goertzel(s_x, m, h0 + b * kBinHz, (float)DSP_DECIM_HZ)) : 0.0f;
    return nh;
}

static bool nearHarmonic(float rate, float base, int kmin, int kmax, float tol_bpm) {
    for (int k = kmin; k <= kmax; k++) if (fabsf(rate - k * base) < tol_bpm) return true;
    return false;
}

// Peaks of P[0..nb) (bin b = f0_hz + b/60 Hz) more than thr_db above the noise
// floor. The bins are 1 bpm apart but the 24 s Hann window resolves 2.5 bpm,
// so a peak's main lobe spans +-5 bins: those are excluded around the strongest
// bin, and the floor is the median of the rest / ln 2 (median of the exponential
// periodogram of noise = ln 2 x its mean), which a second peak's lobe shifts
// far less than a mean would. Only interior local maxima count: an edge bin is
// leakage from outside the band (drift, motion).
// Weaker peaks within 3 bpm of, or at 2x/3x, a stronger accepted peak are
// dropped; with `avoid`, so are 1x..8x harmonics of those rates (heart band).
static constexpr int kLobeBins = 5;
static void findPeaks(const float* P, int nb, float f0_hz, float thr_db,
                      const FixedVec<SpectralPeak, MAX_PEAKS>* avoid,
                      FixedVec<SpectralPeak, MAX_PEAKS>& out) {
    if (nb < 2 * kLobeBins + 5) return;
    int top = 0;
    for (int b = 1; b < nb; b++) if (P[b] > P[top]) top = b;
    float rest[kHeartMaxBins]; int nn = 0;
    for (int b = 0; b < nb; b++)
        if (b < top - kLobeBins || b > top + kLobeBins) rest[nn++] = P[b];
    std::nth_element(rest, rest + nn / 2, rest + nn);
    const double noise = rest[nn / 2] / 0.6931472;
    if (noise <= 0) return;

    struct Cand { float rate, snr; };
    FixedVec<Cand, 64> cand;
    for (int b = 1; b + 1 < nb; b++) {
        if (!(P[b] > P[b - 1] && P[b] >= P[b + 1])) continue;
        const float snr = 10.0f * log10f((float)(P[b] / noise));
        if (snr < thr_db) continue;
        // Parabolic interpolation of the peak position between bins.
        const float den = P[b - 1] - 2.0f * P[b] + P[b + 1];
        const float d = den != 0 ? 0.5f * (P[b - 1] - P[b + 1]) / den : 0.0f;
        cand.push_back({(f0_hz + (b + d) * kBinHz) * 60.0f, snr});
    }
    std::sort(cand.begin(), cand.end(), [](const Cand& a, const Cand& b) { return a.snr > b.snr; });
    for (const auto& c : cand) {
        if (out.size() >= MAX_PEAKS) break;
        bool ok = true;
        if (avoid) for (const auto& a : *avoid) if (nearHarmonic(c.rate, a.rate_bpm, 1, 8, 2.0f)) ok = false;
        for (const auto& o : out) {
            if (fabsf(c.rate - o.rate_bpm) < 3.0f || nearHarmonic(c.rate, o.rate_bpm, 2, 3, 1.5f)) ok = false;
            // Hann sidelobes: -31 dB at one lobe width, < -45 dB beyond two. A
            // much weaker neighbour inside two lobes is leakage, not a second rate.
            if (fabsf(c.rate - o.rate_bpm) < 2.0f * kLobeBins && c.snr < o.snr - 25.0f) ok = false;
        }
        if (ok) out.push_back({c.rate, c.snr});
    }
}

LinkResult analyzeLink(const LinkSnapshot& s, const float* y) {
    LinkResult r;
    r.valid = s.valid; r.rssi = s.rssi; r.mean_amp = s.mean_amp; r.pkt_rate = s.pkt_rate;
    r.motion_index = s.motion_index; r.motion = s.motion;
    if (!s.valid || s.n < DSP_DECIM_HZ * 12) return r;

    float P[SPEC_MAX];
    const int nb = breathPeriodogram(y, s.n, P);
    // Every peak above the hypothesis SNR goes out with its SNR; consumers
    // decide what counts as a detection (snr >= breath_snr_db / heart_snr_db).
    findPeaks(P, nb, g_dsp.breath_fmin_bpm / 60.0f, g_dsp.breath_hyp_db, nullptr, r.breaths);
    if (r.breaths.empty()) return r;

    // Heartbeat only on a still window that shows at least a breathing hypothesis.
    float H[kHeartMaxBins];
    const int nh = heartPeriodogram(y, s.n, H);
    findPeaks(H, nh, g_dsp.heart_fmin_bpm / 60.0f, g_dsp.heart_hyp_db, &r.breaths, r.hearts);
    return r;
}

FixedVec<float, SPEC_MAX> breathSpectrum(const float* y, int n) {
    FixedVec<float, SPEC_MAX> out;
    if (n < DSP_DECIM_HZ * 4) return out;
    float P[SPEC_MAX];
    const int nb = breathPeriodogram(y, n, P);
    for (int b = 0; b < nb; b++) out.push_back(P[b]);
    return out;
}

FixedVec<float, HSPEC_MAX> heartSpectrum(const float* y, int n) {
    FixedVec<float, HSPEC_MAX> out;
    if (n < DSP_DECIM_HZ * 4) return out;
    float H[kHeartMaxBins];
    const int nh = heartPeriodogram(y, n, H);
    for (int b = 0; b < nh; b++) out.push_back(H[b]);
    return out;
}

float breathWavePoint(const float* y, int n, int last) {
    if (n == 0 || last <= 0) return 0;
    last = std::min(last, n);
    float a = 0;
    for (int i = n - last; i < n; i++) a += y[i];
    return a / last;
}

// Phase of the oscillation at rate_bpm at the newest sample (integer Goertzel,
// unwindowed; state bound n 2^15 / sin(2pi 0.12/20) < 2^29).
float breathPhase(const float* y, int n, float rate_bpm) {
    if (rate_bpm <= 0 || n == 0) return 0;
    const int m = prep(y, n, 1, false, s_x);
    if (m == 0) return 0;
    const float w = 2.0f * (float)M_PI * (rate_bpm / 60.0f) / (float)DSP_DECIM_HZ;
    const GState g = goertzel(s_x, m, rate_bpm / 60.0f, (float)DSP_DECIM_HZ);
    const float re = (float)g.s1 - (float)g.s2 * cosf(w), im = (float)g.s2 * sinf(w);
    float p = atan2f(im, re) / (2.0f * (float)M_PI);
    p -= floorf(p);
    return p;
}

float link_quality_db(float rate_hz, float rssi_dbm) {
    const float r = std::min(rssi_dbm, -15.0f);
    return (r + 100.0f) + 10.0f * log10f(std::max(1.0f, std::min(rate_hz, 200.0f)));
}

// Peaks within tol_bpm of a cluster's seed rate are one vital seen on several
// links. Rate: SNR-weighted mean. SNR: the max, not the sum. The links are not
// independent (both boards hear the same transmitters with antennas a few cm
// apart; leader<->follower is one reciprocal channel), so summing would let
// two 11 dB noise peaks pass a 14 dB threshold. Comparing to the seed keeps a
// cluster from drifting across rates one tolerance at a time.
static void clusterInto(FixedVec<SpectralPeak, MAX_PEAKS * MAX_LINKS * 2>& all, float tol_bpm,
                        FixedVec<SpectralPeak, MAX_PEAKS>& out) {
    std::sort(all.begin(), all.end(), [](const SpectralPeak& a, const SpectralPeak& b) { return a.rate_bpm < b.rate_bpm; });
    float seed = 0, wsum = 0;
    for (const auto& p : all) {
        const float wp = powf(10.0f, p.snr / 10.0f);
        if (!out.empty() && fabsf(p.rate_bpm - seed) <= tol_bpm) {
            SpectralPeak& k = out.back();
            k.rate_bpm = (k.rate_bpm * wsum + p.rate_bpm * wp) / (wsum + wp);
            k.snr = std::max(k.snr, p.snr);
            wsum += wp;
        } else if (out.size() < MAX_PEAKS) {
            out.push_back(p);
            seed = p.rate_bpm; wsum = wp;
        }
    }
}

SceneResult fuseLinks(const FixedVec<LinkResult, MAX_LINKS * 2>& links, float tol_bpm) {
    SceneResult scene;
    FixedVec<SpectralPeak, MAX_PEAKS * MAX_LINKS * 2> breaths, hearts;
    for (const auto& l : links) {
        if (!l.valid) continue;
        scene.active_links++;
        if (l.motion_index > scene.motion_index) scene.motion_index = l.motion_index;
        if (l.motion) scene.presence = true;
        for (const auto& b : l.breaths) breaths.push_back(b);
        for (const auto& h : l.hearts) hearts.push_back(h);
    }
    clusterInto(breaths, tol_bpm, scene.breaths);
    clusterInto(hearts, tol_bpm, scene.hearts);
    // Hypotheses stay in the lists (for the tracker); only detections count.
    // Presence here is gross motion only: breathing presence is decided by the
    // tracker (confirmed tracks), which controls the false-alarm rate.
    for (const auto& b : scene.breaths) scene.breather_count += b.snr >= g_dsp.breath_snr_db;
    for (const auto& h : scene.hearts) scene.heart_count += h.snr >= g_dsp.heart_snr_db;
    return scene;
}
