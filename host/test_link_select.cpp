// Link selection: smoothing, dwell, margin + hold, and immediate drop of links
// that become unusable. Build: c++ -std=c++17 -O2 -I../src test_link_select.cpp ../src/dsp.cpp -o /tmp/tls && /tmp/tls
#include "link_select.h"
#include "dsp.h"
#include <cstdio>
#include <cmath>
#include <random>

static int fails = 0;
static void check(bool ok, const char* what) { printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what); fails += !ok; }

static LinkInput link(uint8_t id, float q, bool valid = true) {
    LinkInput in{}; in.key.mac[5] = id; in.quality_db = q; in.valid = valid; return in;
}

int main() {
    const float dt = 0.25f;
    SelectParams now_p; now_p.settle_s = 0;   // tests 2-4: selection without the settle time
    std::mt19937 rng(7); std::normal_distribution<float> N(0, 1);

    // 1. Two links 1 dB apart with 3 dB of per-tick jitter: a per-tick ranking
    //    would swap often; the selector keeps one for the whole run.
    {
        LinkSelector<8> s; int switches = 0, prev = -1;
        for (int t = 0; t < 4 * 300; t++) {
            LinkInput in[2] = { link(1, 80 + 3 * N(rng)), link(2, 79 + 3 * N(rng)) };
            bool sel[2]; s.update(dt, in, 2, 1, sel);
            const int cur = sel[0] ? 0 : (sel[1] ? 1 : -1);
            if (prev >= 0 && cur != prev) switches++;
            prev = cur;
        }
        printf("Test1 close links, 5 min: %d switches\n", switches);
        check(switches == 0, "no switching between links of similar quality");
    }
    // 2. A clearly better link appears: it takes over, but not before the
    //    incumbent's dwell (20 s) and its own hold (5 s).
    {
        LinkSelector<8> s; float took = -1;
        for (int t = 0; t < 4 * 60; t++) {
            const float ts = t * dt;
            LinkInput in[2] = { link(1, 70), link(2, ts >= 2 ? 90 : 0, ts >= 2) };
            bool sel[2]; s.update(dt, in, 2, 1, sel, now_p);
            if (sel[1] && took < 0) took = ts;
        }
        printf("Test2 better link from t=2 s: selected at t=%.2f s\n", took);
        check(took >= 20.0f && took <= 26.0f, "better link replaces the incumbent after dwell + hold");
    }
    // 3. A selected link becomes unusable: dropped at once, replaced by the next.
    {
        LinkSelector<8> s; bool ok = true;
        for (int t = 0; t < 4 * 30; t++) {
            const float ts = t * dt;
            LinkInput in[2] = { link(1, 90, ts < 10), link(2, 70) };
            bool sel[2]; s.update(dt, in, 2, 1, sel, now_p);
            if (ts >= 10 && (sel[0] || !sel[1])) ok = false;
        }
        check(ok, "invalid link dropped immediately, next best takes over");
    }
    // 4. Free places fill immediately; lowering max_n drops the worst first.
    {
        LinkSelector<8> s;
        LinkInput in[3] = { link(1, 60), link(2, 80), link(3, 70) };
        bool sel[3]; s.update(dt, in, 3, 3, sel, now_p);
        check(sel[0] && sel[1] && sel[2], "all three selected when three places are free");
        s.update(dt, in, 3, 1, sel, now_p);
        check(!sel[0] && sel[1] && !sel[2], "max 1 keeps only the best");
    }
    // 5. Quality: rate and RSSI both count; compression caps strength.
    {
        const float strong_fast = link_quality_db(100, -25), weak_fast = link_quality_db(100, -55);
        const float strong_slow = link_quality_db(5, -25), saturated = link_quality_db(100, -5);
        printf("Test5 quality dB: -25/100=%.1f -55/100=%.1f -25/5=%.1f -5/100=%.1f\n", strong_fast, weak_fast, strong_slow, saturated);
        check(strong_fast > weak_fast && strong_fast > strong_slow, "stronger and faster both rank higher");
        check(fabsf(saturated - link_quality_db(100, -15)) < 1e-3f, "no credit above -15 dBm (receiver compression)");
    }
    // 6. A bursty link (valid 6 s, invalid 4 s, repeating) never gets in; a
    //    steady one does, 10 s after it became valid.
    {
        LinkSelector<8> s; bool burst_in = false; float steady_at = -1;
        for (int t = 0; t < 4 * 120; t++) {
            const float ts = t * dt;
            LinkInput in[2] = { link(1, 90, fmodf(ts, 10.0f) < 6.0f), link(2, 60, ts >= 5) };
            bool sel[2]; s.update(dt, in, 2, 2, sel);
            burst_in |= sel[0];
            if (sel[1] && steady_at < 0) steady_at = ts;
        }
        printf("Test6 steady link from t=5 s selected at t=%.2f s\n", steady_at);
        check(!burst_in, "a link valid only in 6 s bursts is never selected");
        check(steady_at >= 14.5f && steady_at <= 15.5f, "a steadily valid link is selected after the 10 s settle time");
    }
    printf(fails ? "\n%d CHECK(S) FAILED\n" : "\nALL LINK SELECTION CHECKS PASSED\n", fails);
    return fails != 0;
}
