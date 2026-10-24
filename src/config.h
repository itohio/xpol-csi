#pragma once
// Project-wide tunables. One firmware runs on every board; the role (leader /
// follower / standalone) is decided at runtime by leader election, and each
// board's identity is its factory MAC. No build-time role split.

#include <stdint.h>

// ---------------------------------------------------------------------------
// Board pins (Seeed XIAO ESP32-C5)
// ---------------------------------------------------------------------------
#define PIN_USER_LED        27   // yellow USER LED, active HIGH
#define PIN_LINK_UART_TX     11  // D6
#define PIN_LINK_UART_RX     12  // D7
#define LINK_UART_PORT        1  // UART_NUM_1 (USB Serial/JTAG stays the console)
#define LINK_UART_BAUD   921600

// ---------------------------------------------------------------------------
// SoftAP (hosted by the leader) + web dashboard
// ---------------------------------------------------------------------------
#define AP_SSID   "xpol-csi"
#define AP_PASS   "sensesense"
// Served at 10.0.0.1 (see radio_enable_ap).

// ---------------------------------------------------------------------------
// Radio / sensing band
// ---------------------------------------------------------------------------
#define SENSE_BAND_5G            1
#define SENSE_USE_HT40           0   // HT20: consistent illuminator/CSI, fewer subcarriers

#if SENSE_BAND_5G
  #define SURVEY_CHANNELS   {36, 40, 44, 48, 149, 153, 157, 161}
  #define SENSE_CH_FALLBACK 149
#else
  #define SURVEY_CHANNELS   {1, 6, 11}
  #define SENSE_CH_FALLBACK 11
#endif

#define SURVEY_TOTAL_MS         3000
#define ILLUMINATE_RATE_HZ       100   // follower ESP-NOW frames/s (default; web-adjustable)
#define ILLUMINATE_MAX_HZ        250   // upper bound for the web control (~4% airtime)
#ifndef TX_POWER_DBM
#define TX_POWER_DBM              14   // max TX power of both boards at boot (25 mW; driver default 18).
#endif                                 // Web/console (`t<dBm>`) change it until the next reboot.
// Both boards illuminate (ESP-NOW broadcasts) and each senses the other.
// ESP-NOW is initialised on the first nonzero rate only.
#ifndef LEADER_ILLUMINATE_RATE_HZ
#define LEADER_ILLUMINATE_RATE_HZ 100
#endif

// ---------------------------------------------------------------------------
// Leader election
// ---------------------------------------------------------------------------
#define ELECTION_HELLO_MS         200   // HELLO resend interval
#define ELECTION_WINDOW_MS       2000   // listen window before deciding
#define LEADER_HEARTBEAT_MS      1000   // leader liveness beacon
#define LEADER_TIMEOUT_MS        3500   // follower promotes if leader silent

// ---------------------------------------------------------------------------
// DSP constants (shared with the host tests)
// ---------------------------------------------------------------------------
#define MAX_SUBCARRIERS          128   // max incoming CSI subcarriers per sample
#define DSP_NSEL                  16   // subcarriers stored/analyzed per link
#ifndef MAX_LINKS
#define MAX_LINKS                  3   // sensing links per board; build flag (platformio.ini sensing_flags)
#endif
#ifndef DSP_DECIM_HZ
#define DSP_DECIM_HZ              20   // breathing/heart grid (multiple of 5); build flag. Ring per link =
#endif                                 // BREATH_WIN_SEC x DSP_DECIM_HZ x DSP_NSEL x 2 B (15.4 KB at 20 Hz)
#define BREATH_WIN_SEC             24
#define BREATH_FMIN_HZ          0.12f   // ~7 breaths/min
#define BREATH_FMAX_HZ          0.70f   // ~42 breaths/min
#define BREATH_SNR_DB           14.0f   // peak over the median noise floor. White noise p99.9 = 13.5 dB; empty-room
                                        // replays (host/replay.cpp, data/empty*) exceed it in 0-0.7% of still ticks,
                                        // which the tracker's 10 s confirmation absorbs.
#define BREATH_IDLE_SEC            12   // vital signs only after this long without motion (and only on data since then)
#define HEART_FMIN_HZ           0.75f   // 45 bpm
#define HEART_FMAX_HZ           2.50f   // 150 bpm
#define HEART_SNR_DB            12.0f   // white noise p99.9 = 11.1; experimental: only evaluated while breathing + still
#define MOTION_WIN_PKTS          100   // moving-variance window, frames (ESPectre: 75-100)
#define MOTION_MV_DEFAULT       1e-4f   // motion threshold until a calibration exists (empty room here: ~2.3e-5)
#define MIN_LINK_PKT_RATE       20.0f   // frames/s a link needs to be used for sensing at all (motion + vitals)
#define MAX_PEAKS                  8   // LinkResult/SceneResult breaths & hearts cap
#define SPEC_MAX                  48   // breathSpectrum bin cap (35 bins used)
#define CALIB_SECONDS             20   // 80 samples at 4 Hz for the P95 threshold

// ---------------------------------------------------------------------------
// Verbose USB telemetry
//   Lightweight EVT/ROLE/CH/SCENE lines stream in every build. The heavy CSI
//   stream is on by default only in a VERBOSE_USB build, and can be toggled at
//   runtime by the host tool ('v').
// ---------------------------------------------------------------------------
#ifndef VERBOSE_USB
#define VERBOSE_USB 0
#endif
#define TELEM_SCENE_HZ            4
#define TELEM_FEAT_HZ            2

// ---------------------------------------------------------------------------
// Operating mode
// ---------------------------------------------------------------------------
enum class Mode : uint8_t {
    CALIBRATE = 0,
    RUN       = 1,
};
#ifndef DEFAULT_MODE
#define DEFAULT_MODE Mode::RUN
#endif
