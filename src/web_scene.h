#pragma once
// What the web dashboard is sent each scene tick, filled by the leader. Current
// values only: the browser builds every history (rate charts, waveform,
// waterfall), so nothing here keeps display history on the board.
#include "tracker.h"
#include "fixedvec.h"
#include <string.h>
#include <stdio.h>

// One sensing link as the web shows it: "<mac> <format>", e.g. "a691… ht0",
// "a691… 11g" (beacons), since one transmitter is one link per PHY format.
struct WebRf {
    char  mac[24] = {0};
    float coupling = 0;   // mean CSI amplitude (AGC-normalised)
    int   rssi = 0;
    bool  used = false;   // its CSI feeds the scene ("links per board")
    float long_bpm = 0, long_snr = 0;   // strongest long-window peak (0: none)
    void set_label(const char* hexmac, const char* fmt_label) {
        snprintf(mac, sizeof(mac), "%s %s", hexmac, fmt_label);
    }
};

// What the web shows of a vital track (the tracker's state without its CI rings).
struct WebBreather {
    int   id = 0;
    bool  confirmed = false;
    float breath_bpm = 0, breath_ci = 0, confidence = 0, breath_amp = 0, age_s = 0, phase = 0;
    float heart_bpm = 0, heart_ci = 0, heart_amp = 0;
    bool  heart_confirmed = false;

    static WebBreather from(const VitalTrack& t) {
        WebBreather b;
        b.id = t.id; b.confirmed = t.confirmed;
        b.breath_bpm = t.breath_bpm; b.breath_ci = t.breath_ci; b.confidence = t.confidence;
        b.breath_amp = t.breath_amp; b.age_s = t.age_s; b.phase = t.phase;
        b.heart_bpm = t.heart_bpm; b.heart_ci = t.heart_ci; b.heart_amp = t.heart_amp;
        b.heart_confirmed = t.heart_confirmed;
        return b;
    }
};

struct WebScene {
    bool  presence = false;
    float motion = 0;
    int   channel = 0;
    int   mode = 0;          // Mode
    int   active_links = 0;
    int   breather_count = 0;
    int   heart_count = 0;
    FixedVec<WebBreather, TRACK_MAX> breathers;  // sorted, with phase filled in
    FixedVec<float, SPEC_MAX> spectrum;          // breathing-band power per 1-bpm bin, strongest link
    float spectrum_fmin_bpm = 7.0f;              // first spectrum bin
    FixedVec<float, HSPEC_MAX> hspectrum;        // heart-band power per 1-bpm bin, same series
    float hspectrum_fmin_bpm = 45.0f;
    float wave_point = 0;                        // newest breathing-waveform point (mean of the last tick)
    bool  wave_valid = false;                    // a still window exists
    FixedVec<WebRf, MAX_LINKS> rf;               // this board's links (coupling, RSSI)
    // Long-window evidence: the scene's clustered peaks, and the strongest still
    // link's band power (lspec_bin_bpm per bin from lspec_fmin_bpm).
    float long_window_s = 0, long_detect_db = 0;
    FixedVec<SpectralPeak, MAX_PEAKS> long_breaths;
    FixedVec<float, LONG_SPEC_MAX> lspectrum;
    float lspec_fmin_bpm = 0, lspec_bin_bpm = 0, lspec_window_s = 0;
};
