#pragma once
// Vital tracker: turns the per-tick spectral hypotheses into persistent
// "breather" tracks.
//  - Every breathing hypothesis (snr >= breath_hyp_db) can form or feed a track;
//    a track is shown as "forming" from its first observation.
//  - A track is confirmed only after >= track_confirm_s of tracking with
//    confidence >= 0.6 AND a smoothed SNR at detection level (>= breath_snr_db).
//    Consecutive 24 s windows overlap almost entirely, so persistence alone
//    is weak evidence; the SNR condition keeps the false-alarm control of the
//    detector.
//  - The rate CI is the spread (std) of the raw rate estimates matched in the
//    last ~10 s, not of the smoothed rate.
//  - A few skipped breaths are tolerated (confidence decays slowly during a gap
//    and recovers when breathing resumes); a long gap drops the track.
//  - Each breather carries an associated heartbeat hypothesis with its own SNR,
//    CI and confirmation (>= track_confirm_s at heart_snr_db).
// Pure C++ (no IDF) so it is host-testable.

#include "dsp.h"
#include "fixedvec.h"

#ifndef TRACK_MATCH_BPM
#define TRACK_MATCH_BPM      3.0f
#endif
#ifndef TRACK_GROW_SEC
#define TRACK_GROW_SEC      15.0f   // seconds of consistency to reach full confidence
#endif
#ifndef TRACK_GRACE_SEC
#define TRACK_GRACE_SEC      4.0f   // skipped-breath tolerance before confidence decays
#endif
#ifndef TRACK_DROP_SEC
#define TRACK_DROP_SEC      10.0f   // absent this long -> remove track
#endif
#ifndef TRACK_OBS
#define TRACK_OBS            40     // raw matched estimates kept for the CI (~10 s at 4 Hz)
#endif
#ifndef TRACK_MAX
#define TRACK_MAX             4
#endif

struct VitalTrack {
    int    id = 0;
    float  breath_bpm = 0;
    float  breath_amp = 0;      // smoothed spectral SNR, dB
    float  breath_ci = 0;       // +/- bpm: std of the raw matched rate estimates
    float  heart_bpm = 0;
    float  heart_amp = 0;       // smoothed heart SNR, dB
    float  heart_ci = 0;        // +/- bpm
    float  heart_age_s = 0;     // continuously associated heart duration
    float  heart_since_s = 0;   // since the heart hypothesis was last matched
    bool   heart_confirmed = false;
    float  confidence = 0;      // 0..1
    float  age_s = 0;           // continuously-tracked duration
    float  since_seen_s = 0;
    bool   confirmed = false;
    float  phase = 0;           // 0..1 current breathing phase (for synced pulsing)
    FixedRing<float, TRACK_OBS>  breath_obs;   // raw matched rates (CI)
    FixedRing<float, TRACK_OBS>  heart_obs;
};

class VitalTracker {
public:
    // Advance by dt seconds given this tick's breathing and heart hypotheses.
    void update(float dt, const FixedVec<SpectralPeak, MAX_PEAKS>& breaths,
                const FixedVec<SpectralPeak, MAX_PEAKS>& hearts);

    // Sorts in place (confirmed first, then confidence*amplitude) and returns a
    // reference — no multi-KB copy. update() matches by rate, so reordering is safe.
    const FixedVec<VitalTrack, TRACK_MAX>& tracks();
    int confirmedCount() const;

private:
    FixedVec<VitalTrack, TRACK_MAX> tracks_;
    int next_id_ = 1;
};
