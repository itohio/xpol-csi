#include "tracker.h"
#include <algorithm>
#include <cmath>

template <int N>
static float ring_std(const FixedRing<float, N>& h) {
    if (h.size() < 3) return 0;
    double m = 0; for (size_t i = 0; i < h.size(); i++) m += h[i]; m /= h.size();
    double s = 0; for (size_t i = 0; i < h.size(); i++) s += (h[i] - m) * (h[i] - m);
    return (float)std::sqrt(s / h.size());
}

// Associate this tick's heart hypotheses with track t: nearest to its current
// heart rate (within 8 bpm), or the strongest when it has none yet.
static void update_heart(VitalTrack& t, float dt, const FixedVec<SpectralPeak, MAX_PEAKS>& hearts) {
    const SpectralPeak* hp = nullptr;
    float best = 1e9f;
    for (const auto& h : hearts) {
        const float d = t.heart_bpm == 0 ? -h.snr : std::fabs(h.rate_bpm - t.heart_bpm);
        if ((t.heart_bpm == 0 || d < 8.0f) && d < best) { best = d; hp = &h; }
    }
    if (hp) {
        if (t.heart_bpm == 0) {
            t.heart_bpm = hp->rate_bpm; t.heart_amp = hp->snr; t.heart_age_s = dt;
            t.heart_obs.clear();
        } else {
            t.heart_bpm = 0.7f * t.heart_bpm + 0.3f * hp->rate_bpm;
            t.heart_amp = 0.7f * t.heart_amp + 0.3f * hp->snr;
            t.heart_age_s += dt;
        }
        t.heart_since_s = 0;
        t.heart_obs.push(hp->rate_bpm);
        t.heart_ci = ring_std(t.heart_obs);
    } else if ((t.heart_since_s += dt) > TRACK_DROP_SEC) {
        t.heart_bpm = 0; t.heart_amp = 0; t.heart_ci = 0; t.heart_age_s = 0;
        t.heart_obs.clear();
    }
    t.heart_confirmed = t.heart_bpm > 0 && t.heart_age_s >= g_dsp.track_confirm_s &&
                        t.heart_amp >= g_dsp.heart_snr_db;
}

void VitalTracker::update(float dt, const FixedVec<SpectralPeak, MAX_PEAKS>& breaths,
                          const FixedVec<SpectralPeak, MAX_PEAKS>& hearts) {
    bool used[MAX_PEAKS] = {false};

    // 1) Match each existing track to the nearest unused breathing hypothesis.
    for (auto& t : tracks_) {
        int best = -1; float bestd = TRACK_MATCH_BPM;
        for (size_t i = 0; i < breaths.size(); i++) {
            if (used[i]) continue;
            const float d = std::fabs(breaths[i].rate_bpm - t.breath_bpm);
            if (d < bestd) { bestd = d; best = (int)i; }
        }
        if (best >= 0) {
            used[best] = true;
            const SpectralPeak& p = breaths[best];
            t.breath_bpm = 0.7f * t.breath_bpm + 0.3f * p.rate_bpm;
            t.breath_amp = 0.7f * t.breath_amp + 0.3f * p.snr;
            t.age_s += dt;
            t.since_seen_s = 0;
            t.confidence = std::min(1.0f, t.confidence + dt / TRACK_GROW_SEC);
            t.breath_obs.push(p.rate_bpm);
            t.breath_ci = ring_std(t.breath_obs);
            update_heart(t, dt, hearts);
        } else {
            // Not seen this tick: tolerate a short gap, then decay confidence.
            t.since_seen_s += dt;
            if (t.since_seen_s > TRACK_GRACE_SEC)
                t.confidence = std::max(0.0f, t.confidence - dt / TRACK_GROW_SEC);
            update_heart(t, dt, FixedVec<SpectralPeak, MAX_PEAKS>{});
        }
        t.confirmed = t.age_s >= g_dsp.track_confirm_s && t.confidence >= 0.6f &&
                      t.breath_amp >= g_dsp.breath_snr_db;
    }

    // 2) Spawn new tracks for unmatched hypotheses, strongest first.
    for (int k = 0; k < (int)breaths.size() && (int)tracks_.size() < TRACK_MAX; k++) {
        int i = -1;
        for (int j = 0; j < (int)breaths.size(); j++)
            if (!used[j] && (i < 0 || breaths[j].snr > breaths[i].snr)) i = j;
        if (i < 0) break;
        used[i] = true;
        VitalTrack t;
        t.id = next_id_++;
        t.breath_bpm = breaths[i].rate_bpm;
        t.breath_amp = breaths[i].snr;
        t.age_s = dt;
        t.confidence = std::min(1.0f, dt / TRACK_GROW_SEC);
        t.breath_obs.push(t.breath_bpm);
        tracks_.push_back(t);
    }

    // 3) Drop tracks absent too long.
    int w = 0;
    for (int i = 0; i < (int)tracks_.size(); i++)
        if (!(tracks_[i].since_seen_s > TRACK_DROP_SEC)) { if (w != i) tracks_[w] = tracks_[i]; w++; }
    tracks_.n_ = (uint16_t)w;
}

const FixedVec<VitalTrack, TRACK_MAX>& VitalTracker::tracks() {
    std::sort(tracks_.begin(), tracks_.end(), [](const VitalTrack& a, const VitalTrack& b){
        if (a.confirmed != b.confirmed) return a.confirmed > b.confirmed;
        return a.confidence * a.breath_amp > b.confidence * b.breath_amp;
    });
    return tracks_;
}

int VitalTracker::confirmedCount() const {
    int n = 0; for (const auto& t : tracks_) if (t.confirmed) n++; return n;
}
