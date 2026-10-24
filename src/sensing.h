#pragma once
// Routes the CSI stream into per-link processors and exposes current results.

#include "radio.h"
#include "dsp.h"
#include "config.h"
#include <stdint.h>

struct LinkView {
    uint8_t    mac[6];
    uint8_t    fmt;     // CSI format: one transmitter can be two links (HT data, legacy beacons)
    LinkResult result;
};

// Creates the (static) CSI ring and its consumer task; pass the ring to radio_start_csi.
RingbufHandle_t sensing_init();
void sensing_set_mode(Mode m);

// Empty-room calibration of every current link. Finalize stores each link's
// motion threshold in NVS; links seen later pick theirs up on first frame.
void sensing_begin_calibration();
void sensing_finalize_calibration();
// Number of link thresholds restored from NVS at init (0 = never calibrated).
int  sensing_stored_calibrations();
bool sensing_calibrating();   // true between begin and finalize

// Web tune tab. Commands from the WebSocket are queued for the main loop:
// 'R' reset buffers, 'C' calibrate, 'S' save params, 'D' default params.
void sensing_post_command(char c);
char sensing_take_command();  // 0 if none
// True once after a link delivered kFrozenRun byte-identical CSI frames in a
// row (latched CSI engine): the caller recovers (radio_recover_csi).
bool sensing_take_csi_restart();
// Clear every link's signal buffers, keeping calibrations.
void sensing_reset_buffers();
// Persist g_dsp to NVS (loaded automatically by sensing_init()).
bool sensing_save_params();

// Per-link results for this tick. Each link is snapshotted under the sensing
// lock (motion decision + breathing series) and analysed after releasing it;
// one still link per tick also gets a long-window analysis (LinkResult.long_*
// carries each link's latest). Main task only. Returns link count (<= MAX_LINKS).
int  sensing_snapshot(LinkView* out, int max);

// Views from the last sensing_snapshot(): the still link with the highest mean
// amplitude. Empty/0 while no link is still long enough. Main task only.
FixedVec<float, SPEC_MAX> sensing_spectrum();
FixedVec<float, HSPEC_MAX> sensing_heart_spectrum();
float sensing_wave_point(bool* valid);   // newest waveform point (mean of the last tick's samples)
float sensing_phase(float bpm);
// That link's last long-window band power (n = 0: none yet); p stays valid
// until the next sensing_snapshot(). Main task only.
struct LongSpectrum { float fmin_bpm, bin_bpm, window_s; int n; const float* p; };
LongSpectrum sensing_long_spectrum();
// A link delivered >= 5 byte-identical frames in a row within the last 3 s.
bool  sensing_csi_frozen();

// CSI reader-vs-writer health: frames dropped because the ring was full and
// frames written (both cumulative), the largest backlog (written - consumed) and
// ring bytes (size, used now, peak) since the previous call; and the main
// task's longest hold of the link lock (the consumer waits that long), longest
// long-window analysis and their count.
struct SensingHealth {
    uint32_t drops, written, max_backlog;
    uint32_t ring_size, ring_used, ring_peak;
    uint32_t lock_max_us, long_max_us, long_runs;
};
SensingHealth sensing_health();   // resets the peaks: diag_update() is the only caller

// MACs of the transmitters currently admitted as sensing links.
int sensing_link_macs(uint8_t (*out)[6], int max);

// Raw-tab stream of the strongest link: its breathing-grid rows written after
// *cursor (row-major [row][n_sub] Q4, AMP_FRAC_BITS; a row every
// 1000/DSP_DECIM_HZ ms), at most max_rows; advances *cursor. *slot names the
// link: when the strongest link changes it is updated and the stream restarts
// from the newest row (-1: no link). Returns rows.
int sensing_csi_rows(int* slot, uint32_t* cursor, int16_t* out, int max_rows, int* n_sub, int* rssi);
