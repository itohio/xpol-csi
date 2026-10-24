// Offline replay of a recorded session (host/record_session.py) through the
// exact firmware DSP (src/dsp.cpp), ticking every 250 ms of radio time like the
// firmware main loop.
//
//   c++ -std=c++17 -O2 -I../src replay.cpp ../src/dsp.cpp -o /tmp/replay
//   /tmp/replay data/present-1101-*.log [--thr <mac>=<variance>]... [--param "<tune name>=<value>"]... [--spectrum] [--long]
//
// Output (stdout), one row per link per tick:
//   T,t_s,mac,rate,motion_index,motion,motion_var,idle_s,n,breaths(bpm:dB;..),hearts
// with --spectrum also, for still links:
//   S,t_s,mac,fmin_bpm,p0 p1 ...   (breathing band, 1 bin per bpm)
// with --long, each link's long-window analysis on the firmware's schedule
// (every max(2 s, window / 30) while still; "long window" is a --param):
//   L,t_s,mac,window_s,bin_bpm,breaths(bpm:dB;..)
// and with --spectrum too:
//   LS,t_s,mac,fmin_bpm,bin_bpm,p0 p1 ...
// --thr sets a link's calibrated motion threshold (moving variance, as printed by
// the firmware's "# calibrated <mac> motion_threshold=..." line).
#include "dsp.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>

static int hexval(char c) { return c <= '9' ? c - '0' : c - 'a' + 10; }

int main(int argc, char** argv) {
    if (argc < 2) { fprintf(stderr, "usage: replay <session.log> [--thr mac=var]... [--param name=value]... [--spectrum] [--long]\n"); return 2; }
    std::map<std::string, double> thr;
    bool spectrum = false, long_window = false;
    for (int i = 2; i < argc; i++) {
        if (!strcmp(argv[i], "--spectrum")) spectrum = true;
        else if (!strcmp(argv[i], "--long")) long_window = true;
        else if (!strcmp(argv[i], "--thr") && i + 1 < argc) {
            const char* eq = strchr(argv[++i], '=');
            if (eq) thr[std::string(argv[i], eq - argv[i])] = atof(eq + 1);
        } else if (!strcmp(argv[i], "--param") && i + 1 < argc) {   // tune-tab name=value, e.g. "low-cut window=0"
            const char* eq = strchr(argv[++i], '=');
            int n; const DspParamInfo* t = dsp_param_table(&n); bool ok = false;
            for (int k = 0; eq && k < n; k++)
                if (!strncmp(t[k].name, argv[i], eq - argv[i]) && t[k].name[eq - argv[i]] == 0) ok = dsp_param_set(k, (float)atof(eq + 1));
            if (!ok) { fprintf(stderr, "unknown --param %s\n", argv[i]); return 2; }
        }
    }
    FILE* fp = fopen(argv[1], "r");
    if (!fp) { perror(argv[1]); return 1; }

    std::map<std::string, LinkProcessor*> links;
    std::map<std::string, int64_t> long_due;
    static float y[DSP_BREATH_LEN];
    static LongWork work;
    static char line[4096];
    int64_t base = -1, last_raw = -1, wrap = 0, next_tick = -1;
    long frames = 0;
    printf("# T,t_s,mac,rate,motion_index,motion,motion_var,idle_s,n,breaths,hearts\n");
    while (fgets(line, sizeof(line), fp)) {
        char* p = strstr(line, " IQ,");
        if (!p) continue;
        char mac[16]; unsigned long t_us; int rssi, noise; unsigned gain, nsub, fmt = 0; char hex[1100];
        // Newer logs carry a PHY-format column before the hex; older ones do not.
        int got = sscanf(p + 4, "%lu,%12[0-9a-f],%d,%d,%u,%u,%u,%1099[0-9a-f]", &t_us, mac, &rssi, &noise, &gain, &nsub, &fmt, hex);
        if (got != 8) { fmt = 0; got = sscanf(p + 4, "%lu,%12[0-9a-f],%d,%d,%u,%u,%1099[0-9a-f]", &t_us, mac, &rssi, &noise, &gain, &nsub, hex); if (got != 7) continue; }
        if (nsub == 0 || nsub > MAX_SUBCARRIERS || strlen(hex) < 4 * nsub) continue;
        char key[24]; snprintf(key, sizeof key, "%s/%02x", mac, fmt);   // a link = transmitter + PHY format
        if (last_raw >= 0 && (int64_t)t_us < last_raw - 1000000000LL) wrap += 1LL << 32;   // 32-bit radio clock
        last_raw = (int64_t)t_us;
        const int64_t t = (int64_t)t_us + wrap;
        if (base < 0) { base = t; next_tick = t + 250000; }

        int8_t iq[2 * MAX_SUBCARRIERS];
        for (unsigned k = 0; k < 2 * nsub; k++) iq[k] = (int8_t)(hexval(hex[2 * k]) << 4 | hexval(hex[2 * k + 1]));
        LinkProcessor*& lp = links[key];
        if (!lp) {
            lp = new LinkProcessor();
            auto it = thr.find(mac);
            if (it != thr.end()) lp->setMotionThreshold((uint64_t)(it->second * 4294967296.0));
        }
        lp->push(CsiFrame{(uint32_t)t_us, (int8_t)rssi, (uint8_t)nsub, (uint16_t)gain, iq, (uint8_t)fmt});
        frames++;

        while (t >= next_tick) {
            const double ts = (next_tick - base) / 1e6;
            for (auto& [m, proc] : links) {
                const LinkSnapshot s = proc->snapshot(next_tick, y);
                const LinkResult r = analyzeLink(s, y);
                std::string b, h;
                char tmp[32];
                for (auto& x : r.breaths) { snprintf(tmp, sizeof tmp, "%s%.1f:%.1f", b.empty() ? "" : ";", x.rate_bpm, x.snr); b += tmp; }
                for (auto& x : r.hearts)  { snprintf(tmp, sizeof tmp, "%s%.1f:%.1f", h.empty() ? "" : ";", x.rate_bpm, x.snr); h += tmp; }
                const double var = s.motion_index * (double)proc->motionThreshold() * g_dsp.thr_scale / 4294967296.0;
                printf("T,%.2f,%s,%.1f,%.3f,%d,%.3g,%.1f,%d,%s,%s\n", ts, m.c_str(), s.pkt_rate, s.motion_index,
                       s.motion, var, s.idle_s, s.n, b.c_str(), h.c_str());
                if (spectrum && s.n > 0) {
                    auto sp = breathSpectrum(y, s.n);
                    printf("S,%.2f,%s,%.1f,", ts, m.c_str(), g_dsp.breath_fmin_bpm);
                    for (size_t i = 0; i < sp.size(); i++) printf(i ? " %.4g" : "%.4g", sp[i]);
                    printf("\n");
                }
            }
            for (auto& [m, proc] : links) {
                if (!long_window || next_tick < long_due[m]) continue;
                const int n = proc->stillLongRows(next_tick, work.rows, (int)(g_dsp.long_window_s * LONG_HZ));
                const LongResult r = analyzeLong(work, n, proc->nSel());
                long_due[m] = next_tick + (int64_t)(std::max(2.0f, r.window_s / 30.0f) * 1e6f);
                if (r.window_s == 0) continue;
                std::string b;
                char tmp[32];
                for (auto& x : r.breaths) { snprintf(tmp, sizeof tmp, "%s%.2f:%.1f", b.empty() ? "" : ";", x.rate_bpm, x.snr); b += tmp; }
                printf("L,%.2f,%s,%.1f,%.3f,%s\n", (next_tick - base) / 1e6, m.c_str(), r.window_s, work.spec_bin_bpm, b.c_str());
                if (spectrum) {
                    printf("LS,%.2f,%s,%.1f,%.3f,", (next_tick - base) / 1e6, m.c_str(), work.spec_fmin_bpm, work.spec_bin_bpm);
                    for (int i = 0; i < work.n_spec; i++) printf(i ? " %.4g" : "%.4g", work.spectrum[i]);
                    printf("\n");
                }
            }
            next_tick += 250000;
        }
    }
    fprintf(stderr, "replayed %ld frames, %zu links, %.1f s\n", frames, links.size(), base < 0 ? 0.0 : (last_raw + wrap - base) / 1e6);
    return 0;
}
