// Host-side unit test for the DSP core. No hardware or IDF needed.
//   c++ -std=c++17 -O2 -I../src test_dsp.cpp ../src/dsp.cpp -o /tmp/td && /tmp/td
//
// Synthesizes raw CSI I/Q streams with known breathing/heart rates, motion and
// pure noise, drives the processor like the firmware does (frames at 100 Hz,
// a snapshot tick every 250 ms), and checks the decisions.

#include "dsp.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>

static const int NSUB = 56;  // HT20
static unsigned s_rng = 12345;
static float frand() {  // deterministic uniform [-1,1]
    s_rng = s_rng * 1664525u + 1013904223u;
    return ((int)(s_rng >> 9) / (float)(1 << 22)) - 1.0f;
}
static float base_amp(int k) { return 20.f + 15.f * ((k % 7) / 6.f); }

// Encode an amplitude as raw I/Q (I = amp, Q = 0), as the radio delivers it.
static void put_amp(int8_t* iq, int k, float amp) {
    const float a = amp < 0 ? 0 : (amp > 127 ? 127 : amp);
    iq[2 * k] = (int8_t)(a + 0.5f); iq[2 * k + 1] = 0;
}

static float s_y[DSP_BREATH_LEN];
static LongWork s_long;

// One long-window analysis as the firmware runs it after a snapshot.
static LongResult long_eval(LinkProcessor& p, int64_t now_us) {
    const int n = p.stillLongRows(now_us, s_long.rows, (int)(g_dsp.long_window_s * LONG_HZ));
    return analyzeLong(s_long, n, p.nSel());
}

// One firmware tick: snapshot (under the lock in firmware) + analysis.
static LinkResult eval(LinkProcessor& p, int64_t now_us) {
    const LinkSnapshot s = p.snapshot(now_us, s_y);
    return analyzeLink(s, s_y);
}

// `seconds` of 100 Hz frames with amplitude amp(k, t_s), ticking every 250 ms.
template <class AmpFn>
static uint32_t run(LinkProcessor& p, uint32_t t0_us, float seconds, AmpFn amp) {
    int8_t iq[2 * MAX_SUBCARRIERS];
    uint32_t t = t0_us;
    for (int i = 0; i < (int)(seconds * 100); i++, t += 10000) {
        for (int k = 0; k < NSUB; k++) put_amp(iq, k, amp(k, t * 1e-6f));
        p.push(CsiFrame{t, -55, (uint8_t)NSUB, 256, iq});
        if (i % 25 == 24) p.snapshot(t, s_y);
    }
    return t;
}

static auto breathing(float bpm, float depth) {
    return [=](int k, float ts) {
        return base_amp(k) * (1.0f + depth * sinf(2.0f * (float)M_PI * bpm / 60.0f * ts + 0.3f * k)) + 0.5f * frand();
    };
}
static auto still() { return [](int k, float) { return base_amp(k) + 0.5f * frand(); }; }

// A detection (SNR at the detection threshold) at bpm +- tol.
static bool has_rate(const FixedVec<SpectralPeak, MAX_PEAKS>& v, float bpm, float tol, float min_snr) {
    for (const auto& p : v) if (fabsf(p.rate_bpm - bpm) <= tol && p.snr >= min_snr) return true;
    return false;
}
static int detections(const FixedVec<SpectralPeak, MAX_PEAKS>& v, float min_snr) {
    int n = 0;
    for (const auto& p : v) n += p.snr >= min_snr;
    return n;
}

static int s_fail = 0;
static void check(bool ok, const char* msg) {
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", msg);
    if (!ok) s_fail++;
}

int main() {
    // --- Test 1: single breather at 15 bpm ---
    {
        LinkProcessor p;
        uint32_t t = run(p, 0, 30.0f, breathing(15.0f, 0.08f));
        LinkResult r = eval(p, t);
        for (auto& b : r.breaths) printf("Test1 breath %.1f bpm, %.1f dB\n", b.rate_bpm, b.snr);
        check(r.valid && !r.motion, "link valid and still");
        check(has_rate(r.breaths, 15.0f, 1.5f, g_dsp.breath_snr_db), "recovered ~15 bpm");
    }

    // --- Test 2: two breathers on two links fuse to count==2 ---
    {
        LinkProcessor p1, p2;
        uint32_t t1 = run(p1, 0, 30.0f, breathing(15.0f, 0.08f));
        uint32_t t2 = run(p2, 0, 30.0f, breathing(22.0f, 0.08f));
        FixedVec<LinkResult, MAX_LINKS * 2> links;
        links.push_back(eval(p1, t1)); links.push_back(eval(p2, t2));
        SceneResult scene = fuseLinks(links);
        printf("Test2 two breathers: count=%d\n", scene.breather_count);
        check(scene.breather_count == 2, "two distinct breathers");
    }

    // --- Test 3: motion against a calibrated still baseline, with debouncing ---
    {
        LinkProcessor p;
        uint32_t t = run(p, 0, 3.0f, still());
        p.beginCalibration();
        t = run(p, t, 10.0f, still());
        p.finalizeCalibration();
        check(p.calibrated(), "calibration finalized");
        t = run(p, t, 5.0f, still());
        LinkResult r = eval(p, t);
        printf("Test3 still: motion_index=%.2f motion=%d\n", r.motion_index, r.motion);
        check(!r.motion, "still -> no motion");

        // Gross motion: each subcarrier's amplitude wanders independently.
        float walk[NSUB] = {};
        t = run(p, t, 3.0f, [&](int k, float) {
            walk[k] += 0.15f * base_amp(k) * frand();
            if (fabsf(walk[k]) > 0.6f * base_amp(k)) walk[k] *= 0.5f;
            return base_amp(k) + walk[k] + 0.5f * frand();
        });
        r = eval(p, t);
        printf("Test3 moving: motion_index=%.2f motion=%d breaths=%zu\n", r.motion_index, r.motion, r.breaths.size());
        check(r.motion, "motion burst -> motion");
        check(r.breaths.empty(), "no vital signs while moving");
        FixedVec<LinkResult, MAX_LINKS * 2> ml; ml.push_back(r);
        check(fuseLinks(ml).presence, "scene presence on motion");

        t = run(p, t, 4.0f, still());
        r = eval(p, t);
        printf("Test3 still again: motion_index=%.2f motion=%d\n", r.motion_index, r.motion);
        check(!r.motion, "motion clears after stillness");
    }

    // --- Test 4: heartbeat recovered alongside breathing ---
    {
        LinkProcessor p;
        uint32_t t = run(p, 0, 30.0f, [](int k, float ts) {
            const float ph = 0.3f * k;
            return base_amp(k) * (1.0f + 0.08f * sinf(2.0f * (float)M_PI * 0.25f * ts + ph)
                                       + 0.02f * sinf(2.0f * (float)M_PI * 1.1f * ts + ph)) + 0.5f * frand();
        });
        LinkResult r = eval(p, t);
        for (auto& b : r.breaths) printf("Test4 breath %.1f bpm, %.1f dB\n", b.rate_bpm, b.snr);
        for (auto& h : r.hearts) printf("Test4 heart %.1f bpm, %.1f dB\n", h.rate_bpm, h.snr);
        check(has_rate(r.breaths, 15.0f, 1.5f, g_dsp.breath_snr_db), "recovered ~15 bpm breathing");
        check(has_rate(r.hearts, 66.0f, 2.0f, g_dsp.heart_snr_db), "recovered ~66 bpm heartbeat");
    }

    // --- Test 5: integer amplitude matches round(16*gain*|I+jQ|) over all int8 I/Q ---
    {
        const uint16_t gains[] = {64, 256, 300, 1024, 4096};
        int worst = 0;
        for (uint16_t g : gains)
            for (int re = -128; re <= 127; re++)
                for (int im = -128; im <= 127; im++) {
                    const int8_t iq[2] = {(int8_t)re, (int8_t)im};
                    const long ref = lroundf(16.0f * g / 256.0f * sqrtf((float)(re * re + im * im)));
                    const int d = abs((int)csi_amp_q4(iq, 0, g) - (int)(ref > 32767 ? 32767 : ref));
                    if (d > worst) worst = d;
                }
        printf("Test5 csi_amp_q4: worst |int - float| = %d (Q4 LSB)\n", worst);
        check(worst <= 1, "integer Q4 amplitude within 1 LSB of float reference");
    }

    // --- Test 6: subcarriers breathing with opposite signs (multipath). A plain
    // mean across subcarriers cancels this; the waveform must still show it. ---
    {
        LinkProcessor p;
        const float f = 15.0f / 60.0f;
        uint32_t t = run(p, 0, 30.0f, [&](int k, float ts) {
            const float b = 0.5f * sinf(2.0f * (float)M_PI * f * ts);   // 1.7% depth, opposite signs
            return 30.0f + ((k & 1) ? b : -b) + 0.5f * frand();
        });
        const LinkSnapshot s = p.snapshot(t, s_y);
        // y[i] is grid slot (last - n + 1 + i); the newest slot is still open.
        const int D = DSP_DECIM_HZ, last = 30 * D - 1;
        double sxy = 0, sxx = 0, syy = 0;
        for (int i = 0; i < s.n; i++) {
            const double tc = (double)(last - s.n + 1 + i) / D + (1.0 / D - 0.01) / 2;   // mean frame time of the slot
            const double ref = sin(2.0 * M_PI * f * tc);
            sxy += s_y[i] * ref; sxx += (double)s_y[i] * s_y[i]; syy += ref * ref;
        }
        const double corr = fabs(sxy) / sqrt(sxx * syy + 1e-12);
        printf("Test6 opposite-sign subcarriers: n=%d |corr| = %.3f\n", s.n, corr);
        check(s.n >= 20 * D, "breathing component spans the still window");
        check(corr > 0.95, "component tracks breathing despite opposite-sign subcarriers");
    }

    // --- Test 7: breathing below one amplitude unit (0.2-0.35), as measured on
    // the bench: Q4 storage + slot averaging must keep it. ---
    {
        LinkProcessor p;
        uint32_t t = run(p, 0, 30.0f, breathing(15.0f, 0.01f));
        LinkResult r = eval(p, t);
        for (auto& b : r.breaths) printf("Test7 breath %.1f bpm, %.1f dB\n", b.rate_bpm, b.snr);
        check(has_rate(r.breaths, 15.0f, 1.5f, g_dsp.breath_snr_db), "recovered ~15 bpm from sub-LSB modulation");
    }

    // --- Test 12: breathing riding on large slow intensity changes (fading of
    // ~10-15 % at 1-2 bpm, different per subcarrier, plus a gain step): the
    // low-cut must keep PC1 on the breathing, not on the drift. ---
    {
        auto drifting = [](int k, float ts) {
            const float slow = 1.0f + 0.15f * sinf(2.0f * (float)M_PI * 0.025f * ts + 0.4f * k)
                                    + 0.10f * sinf(2.0f * (float)M_PI * 0.012f * ts + 1.1f * k)
                                    + (ts > 18.0f ? 0.08f : 0.0f);
            return base_amp(k) * slow * (1.0f + 0.02f * sinf(2.0f * (float)M_PI * 15.0f / 60.0f * ts + 0.3f * k)) + 0.5f * frand();
        };
        float snr[2] = {0, 0};
        const float keep = g_dsp.hp_window_s;
        for (int pass = 0; pass < 2; pass++) {
            g_dsp.hp_window_s = pass == 0 ? 0.0f : keep;
            LinkProcessor p;
            uint32_t t = run(p, 0, 40.0f, drifting);
            LinkResult r = eval(p, t);
            for (const auto& b : r.breaths) if (fabsf(b.rate_bpm - 15.0f) <= 1.5f) snr[pass] = std::max(snr[pass], b.snr);
            printf("Test12 slow drift, low-cut %s: 15 bpm at %.1f dB (%d peaks)\n", pass ? "8 s" : "off", snr[pass], (int)r.breaths.size());
        }
        g_dsp.hp_window_s = keep;
        check(snr[1] >= g_dsp.breath_snr_db, "low-cut: breathing detected despite slow intensity changes");
        check(snr[1] > snr[0] + 3.0f, "low-cut improves the breathing SNR over mean removal alone");
    }

    // --- Test 8: no person. Noise plus slow per-subcarrier drift must not
    // produce breathers or heartbeats (the old detector fired 40/40). ---
    {
        int nb = 0, nh = 0, nhyp = 0;
        const int runs = 20;
        for (int run_i = 0; run_i < runs; run_i++) {
            LinkProcessor p;
            float drift[NSUB] = {};
            uint32_t t = run(p, (uint32_t)run_i * 7, 30.0f, [&](int k, float) {
                drift[k] += 0.01f * frand();
                return base_amp(k) + drift[k] + 0.7f * frand();
            });
            LinkResult r = eval(p, t);
            nb += detections(r.breaths, g_dsp.breath_snr_db) > 0;
            nh += detections(r.hearts, g_dsp.heart_snr_db) > 0;
            nhyp += !r.breaths.empty();
        }
        printf("Test8 noise only: %d/%d windows with a breathing detection, %d/%d heart detection, %d/%d with a hypothesis\n",
               nb, runs, nh, runs, nhyp, runs);
        check(nb <= 1, "breathing false alarms <= 1 in 20 noise windows");
        check(nh == 0, "no heartbeat false alarms");
    }

    // --- Test 9: a single corrupted frame is not motion (Hampel + voting). ---
    {
        LinkProcessor p;
        uint32_t t = run(p, 0, 3.0f, still());
        p.beginCalibration();
        t = run(p, t, 10.0f, still());
        p.finalizeCalibration();
        int i = 0;
        t = run(p, t, 3.0f, [&](int k, float) { return (i++ / NSUB == 150 && k % 2) ? 120.0f : base_amp(k) + 0.5f * frand(); });
        LinkResult r = eval(p, t);
        printf("Test9 one-frame spike: motion_index=%.2f motion=%d\n", r.motion_index, r.motion);
        check(!r.motion, "single-frame spike -> no motion");
    }

    // --- Test 10: frames arriving at random (~30/s, ~22% of 50 ms slots empty)
    // must keep the breathing time base: 15 bpm stays 15 bpm. ---
    {
        LinkProcessor p;
        int8_t iq[2 * MAX_SUBCARRIERS];
        uint32_t t = 0, next_tick = 250000;
        auto amp = breathing(15.0f, 0.08f);
        while (t < 30000000) {
            t += (uint32_t)(-logf(0.5f * (frand() + 1.0f) + 1e-6f) * 33333.0f) + 1;   // exponential gaps
            for (int k = 0; k < NSUB; k++) put_amp(iq, k, amp(k, t * 1e-6f));
            p.push(CsiFrame{t, -55, (uint8_t)NSUB, 256, iq});
            if (t >= next_tick) { p.snapshot(t, s_y); next_tick += 250000; }
        }
        LinkResult r = eval(p, t);
        for (auto& b : r.breaths) printf("Test10 random arrivals (rate %.0f/s): breath %.1f bpm, %.1f dB\n", r.pkt_rate, b.rate_bpm, b.snr);
        check(has_rate(r.breaths, 15.0f, 1.0f, g_dsp.breath_snr_db), "breathing rate correct with empty slots");
    }

    // --- Test 11: tuning cannot produce an inverted or oversize band, or NaN. ---
    {
        int n; const DspParamInfo* t = dsp_param_table(&n);
        auto id = [&](float DspParams::* f) { for (int i = 0; i < n; i++) if (t[i].field == f) return i; return -1; };
        dsp_param_set(id(&DspParams::breath_fmax_bpm), 20.0f);
        dsp_param_set(id(&DspParams::breath_fmin_bpm), 30.0f);    // past max: max must follow
        const bool b_ok = g_dsp.breath_fmax_bpm >= g_dsp.breath_fmin_bpm + 4 &&
                          g_dsp.breath_fmax_bpm - g_dsp.breath_fmin_bpm <= SPEC_MAX - 1;
        dsp_param_set(id(&DspParams::heart_fmin_bpm), 30.0f);
        dsp_param_set(id(&DspParams::heart_fmax_bpm), 200.0f);    // 170 bpm wide: min must follow
        const bool h_ok = g_dsp.heart_fmax_bpm - g_dsp.heart_fmin_bpm <= 127;
        printf("Test11 params: breath %.0f-%.0f, heart %.0f-%.0f after clamping\n",
               g_dsp.breath_fmin_bpm, g_dsp.breath_fmax_bpm, g_dsp.heart_fmin_bpm, g_dsp.heart_fmax_bpm);
        const bool nan_rejected = !dsp_param_set(id(&DspParams::hampel_k), NAN) && g_dsp.hampel_k == 3.0f;
        dsp_param_set(id(&DspParams::votes_enter), 3.6f);
        const bool rounded = g_dsp.votes_enter == 4.0f;
        dsp_params_defaults();
        check(b_ok && h_ok, "bands stay ordered and within their bin buffers");
        check(nan_rejected && rounded, "NaN rejected, integer params rounded");
        check(g_dsp.breath_snr_db == BREATH_SNR_DB && g_dsp.hampel_x1e4 == 44478, "defaults restore derived values");
    }

    // --- Test 13: a weak, steady breather. The 24 s window has no detection near
    // it; 120 s of still history finds 13.4 bpm at detection SNR, on 0.25 bpm bins. ---
    {
        LinkProcessor p;
        uint32_t t = run(p, 0, 135.0f, breathing(13.4f, 0.0015f));
        const LinkResult r = eval(p, t);
        const LongResult L = long_eval(p, t);
        for (auto& b : r.breaths) printf("Test13 24 s window: %.2f bpm, %.1f dB\n", b.rate_bpm, b.snr);
        for (auto& b : L.breaths) printf("Test13 long window (%.0f s, %.2f bpm bins): %.2f bpm, %.1f dB\n", L.window_s, s_long.spec_bin_bpm, b.rate_bpm, b.snr);
        check(!has_rate(r.breaths, 13.4f, 0.5f, g_dsp.breath_snr_db), "24 s window: no detection within 0.5 bpm");
        check(L.window_s == 120.0f && s_long.spec_bin_bpm == 0.25f, "long window = 120 s of still history, 0.25 bpm bins");
        check(has_rate(L.breaths, 13.4f, 0.1f, g_dsp.long_snr_db), "long window: 13.4 bpm within 0.1 bpm at detection SNR");
    }

    // --- Test 14: the long window only covers stillness: none before LONG_MIN_SEC,
    // none while moving, and motion restarts it. ---
    {
        LinkProcessor p;
        uint32_t t = run(p, 0, 40.0f, breathing(15.0f, 0.08f));
        const LongResult early = long_eval(p, t);
        t = run(p, t, 60.0f, breathing(15.0f, 0.08f));
        const LongResult ready = long_eval(p, t);
        float walk[NSUB] = {};
        t = run(p, t, 3.0f, [&](int k, float) {
            walk[k] += 0.15f * base_amp(k) * frand();
            if (fabsf(walk[k]) > 0.6f * base_amp(k)) walk[k] *= 0.5f;
            return base_amp(k) + walk[k] + 0.5f * frand();
        });
        eval(p, t);
        const int moving_rows = p.stillLongRows(t, s_long.rows, DSP_LONG_LEN);
        t = run(p, t, 30.0f, breathing(15.0f, 0.08f));
        const LongResult after = long_eval(p, t);
        printf("Test14 windows: 40 s still %.0f s, 100 s still %.0f s, moving rows %d, 30 s after motion %.0f s\n",
               early.window_s, ready.window_s, moving_rows, after.window_s);
        check(early.window_s == 0 && early.breaths.empty(), "no long result before 48 s of stillness");
        check(ready.window_s >= 99.0f && has_rate(ready.breaths, 15.0f, 0.3f, g_dsp.long_snr_db), "long result once still long enough");
        check(moving_rows == 0, "no long-history rows while moving");
        check(after.window_s == 0, "motion restarts the long window");
    }

    // --- Test 15: the 10 min history wraps and still caps the window. ---
    {
        LinkProcessor p;
        const float saved = g_dsp.long_window_s;
        g_dsp.long_window_s = (float)LONG_HIST_SEC;
        uint32_t t = run(p, 0, 700.0f, breathing(16.2f, 0.02f));
        const LongResult L = long_eval(p, t);
        g_dsp.long_window_s = saved;
        for (auto& b : L.breaths) printf("Test15 %.0f s window (%.2f bpm bins): %.2f bpm, %.1f dB\n", L.window_s, s_long.spec_bin_bpm, b.rate_bpm, b.snr);
        check(L.window_s == (float)LONG_HIST_SEC, "window capped at the history length after it wrapped");
        check(has_rate(L.breaths, 16.2f, 0.05f, g_dsp.long_snr_db), "rate exact across the wrap");
    }

    // --- Test 16: long-window peaks cluster across links and decide nothing. ---
    {
        FixedVec<LinkResult, MAX_LINKS * 2> links;
        LinkResult a, b, c;
        a.valid = b.valid = c.valid = true;
        a.long_window_s = 120; a.long_breaths.push_back({13.4f, 16.0f});
        b.long_window_s = 300; b.long_breaths.push_back({13.6f, 19.0f});
        c.long_window_s = 120; c.long_breaths.push_back({21.0f, 9.0f});
        links.push_back(a); links.push_back(b); links.push_back(c);
        const SceneResult s = fuseLinks(links);
        printf("Test16 long clusters: %zu, count %d, window %.0f s\n", s.long_breaths.size(), s.long_count, s.long_window_s);
        check(s.long_breaths.size() == 2 && s.long_breaths[0].snr == 19.0f && fabsf(s.long_breaths[0].rate_bpm - 13.5f) < 0.1f,
              "one cluster per rate, SNR the max, rate SNR-weighted");
        check(s.long_count == 1 && s.long_window_s == 300, "long-window detections counted apart");
        check(!s.presence && s.breather_count == 0, "long-window peaks set neither presence nor breathers");
    }

    printf(s_fail ? "\n%d CHECK(S) FAILED\n" : "\nALL CHECKS PASSED\n", s_fail);
    return s_fail ? 1 : 0;
}
