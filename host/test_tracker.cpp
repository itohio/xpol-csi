// Host test for the vital tracker.
//   c++ -std=c++17 -O2 -I../src test_tracker.cpp ../src/tracker.cpp ../src/dsp.cpp -o /tmp/tt && /tmp/tt
#include "tracker.h"
#include <cstdio>
#include <cmath>
#include <initializer_list>

static int fails = 0;
static void check(bool ok, const char* m) { printf("  [%s] %s\n", ok ? "PASS" : "FAIL", m); if (!ok) fails++; }

static void feed(VitalTracker& T, int steps, float dt,
                 std::initializer_list<SpectralPeak> br,
                 std::initializer_list<SpectralPeak> hr) {
    FixedVec<SpectralPeak, MAX_PEAKS> b, h;
    for (auto& p : br) b.push_back(p);
    for (auto& p : hr) h.push_back(p);
    for (int i = 0; i < steps; i++) T.update(dt, b, h);
}

int main() {
    const float dt = 0.25f;  // 4 Hz decisions

    // Test 1: one consistent breather at 15 bpm + heart 66 for 12 s -> confirmed.
    {
        VitalTracker T;
        feed(T, 48, dt, {{15.0f, 40.0f}}, {{66.0f, 10.0f}});
        auto ts = T.tracks();
        printf("T1: tracks=%zu confirmed=%d\n", ts.size(), T.confirmedCount());
        check(!ts.empty(), "track exists");
        check(ts[0].confirmed, "confirmed after >=10s");
        check(std::fabs(ts[0].breath_bpm - 15.0f) < 1.0f, "breath ~15");
        check(std::fabs(ts[0].heart_bpm - 66.0f) < 2.0f, "heart ~66");
        check(ts[0].confidence > 0.6f, "confidence high");
        printf("   bpm=%.1f ci=%.2f heart=%.1f conf=%.2f age=%.1f\n",
               ts[0].breath_bpm, ts[0].breath_ci, ts[0].heart_bpm, ts[0].confidence, ts[0].age_s);
    }

    // Test 2: two breathers -> two confirmed tracks, sorted by conf*amp.
    {
        VitalTracker T;
        feed(T, 48, dt, {{15.0f, 50.0f}, {22.0f, 20.0f}}, {{66.0f, 10.0f}, {78.0f, 6.0f}});
        auto ts = T.tracks();
        printf("T2: confirmed=%d\n", T.confirmedCount());
        check(T.confirmedCount() == 2, "two confirmed breathers");
        check(ts[0].breath_amp >= ts[1].breath_amp, "sorted by amplitude*confidence");
    }

    // Test 3: a short skip is tolerated; a long gap drops the track.
    {
        VitalTracker T;
        feed(T, 48, dt, {{15.0f, 40.0f}}, {{66.0f, 10.0f}});       // confirm
        float conf0 = T.tracks()[0].confidence;
        feed(T, 12, dt, {}, {});                                   // 3 s gap (< grace+)
        check(!T.tracks().empty(), "survives short skip");
        feed(T, 40, dt, {}, {});                                   // long gap ~13 s total
        printf("T3: after long gap tracks=%zu\n", T.tracks().size());
        check(T.tracks().empty(), "dropped after long gap");
        (void)conf0;
    }

    // Test 4: a persistent hypothesis below the detection SNR (9 dB vs 14) is
    // shown as a track but never confirms, however long it lasts.
    {
        VitalTracker T;
        feed(T, 200, dt, {{15.0f, 9.0f}}, {});
        auto ts = T.tracks();
        printf("T4: weak hypothesis tracks=%zu confirmed=%d conf=%.2f\n", ts.size(), T.confirmedCount(), ts.empty() ? 0 : ts[0].confidence);
        check(ts.size() == 1, "hypothesis is tracked (forming)");
        check(T.confirmedCount() == 0, "below detection SNR never confirms");
    }

    // Test 5: CI comes from the raw estimates (alternating 14 / 16 bpm -> std 1),
    // not from the smoothed rate.
    {
        VitalTracker T;
        FixedVec<SpectralPeak, MAX_PEAKS> a, b, none;
        a.push_back({14.0f, 30.0f}); b.push_back({16.0f, 30.0f});
        for (int i = 0; i < 60; i++) T.update(dt, (i & 1) ? a : b, none);
        const auto& t = T.tracks()[0];
        printf("T5: rate %.2f ci %.2f\n", t.breath_bpm, t.breath_ci);
        check(std::fabs(t.breath_ci - 1.0f) < 0.05f, "CI = spread of raw estimates");
    }

    // Test 6: the heart confirms only at detection SNR (12 dB).
    {
        VitalTracker strong, weak;
        feed(strong, 48, dt, {{15.0f, 30.0f}}, {{70.0f, 16.0f}});
        feed(weak, 48, dt, {{15.0f, 30.0f}}, {{70.0f, 9.0f}});
        const auto& s = strong.tracks()[0];
        const auto& w = weak.tracks()[0];
        printf("T6: strong heart %.1f confirmed=%d, weak heart %.1f confirmed=%d\n", s.heart_bpm, s.heart_confirmed, w.heart_bpm, w.heart_confirmed);
        check(s.heart_confirmed && std::fabs(s.heart_bpm - 70.0f) < 1.0f, "strong heart confirmed");
        check(!w.heart_confirmed && w.heart_bpm > 0, "weak heart stays a hypothesis");
    }

    printf(fails ? "\n%d FAILED\n" : "\nALL TRACKER CHECKS PASSED\n", fails);
    return fails ? 1 : 0;
}
