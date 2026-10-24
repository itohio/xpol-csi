#include "sensing.h"
#include <math.h>
#include "telemetry.h"
#include "esp_timer.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <stdio.h>
#include <string.h>

namespace {

// A link = one transmitter in one CSI format (subcarrier count), as heard by
// this board: an AP sends HT data and legacy beacons, and those are different
// channel estimates, so they are processed separately.
struct Link {
    uint8_t       mac[6];
    uint8_t       fmt = 0;
    bool          used = false;
    int64_t       last_seen_us = 0;
    int64_t       admitted_us = 0;
    // AGC compensation relative to this link's first frame: stays near 1.0
    // (Q8 cannot underflow), and the soft-float call runs only on index changes.
    bool          gain_ref_set = false;
    float         gain_ref = 1.0f;
    uint8_t       agc_last = 0;
    int8_t        fft_last = 0;
    uint16_t      gain_q8_last = 256;
    uint32_t      last_hash = 0;     // FNV-1a of the last frame's I/Q (frozen-CSI detection)
    uint16_t      repeats = 0;       // consecutive frames identical to the previous one
    LinkProcessor proc;
};

Link            s_links[MAX_LINKS];

// Link admission (consumer task only): which transmitter/format gets one of the
// MAX_LINKS slots. Every transmitter/format not yet a link is a candidate,
// measured over kCandWindowUs windows (frame rate, mean RSSI). It needs half the
// sensing rate (one-off corrupted MACs and beacon-only APs never qualify); it
// takes a free slot, or replaces the weakest link once its link_quality_db has
// beaten that link's by kReplaceMarginDb for kReplaceHoldUs without a break.
// New links are protected for kLinkGraceUs (they need history before they are
// useful); silent links are freed after kLinkIdleUs.
constexpr int64_t kCandWindowUs    = 2000000;
constexpr float   kReplaceMarginDb = 3.0f;
constexpr int64_t kReplaceHoldUs   = 10000000;
constexpr int64_t kLinkGraceUs     = 30000000;
constexpr int64_t kLinkIdleUs      = 10000000;
struct Candidate {
    uint8_t  mac[6];
    uint8_t  fmt;
    bool     used;
    uint16_t cnt;            // frames in the current window
    int32_t  rssi_sum;       // over the current window
    int64_t  win_start_us, last_us, better_since_us;
    float    rate;           // frames/s over the last full window
    float    rssi;           // mean RSSI over the last full window, dBm
};
Candidate s_cand[12];
SemaphoreHandle_t s_mtx;   // guards s_links; held only for push() / snapshot()
volatile Mode   s_mode = DEFAULT_MODE;

// CSI ring: >= 16 worst-case frames (8 B ring header + 16 B RadioCsi + 256 B
// I/Q). Static, so it costs no heap; smaller HT20 frames fit proportionally more.
constexpr size_t kRingBytes = 16 * (8 + sizeof(RadioCsi) + 2 * MAX_SUBCARRIERS);
StaticRingbuffer_t s_ring_ctl;
uint8_t           s_ring_mem[kRingBytes];
RingbufHandle_t   s_rb = nullptr;

// Reader-vs-writer health: frames written by the callback minus frames consumed,
// and ring occupancy in bytes, peaks since the last sensing_health().
volatile uint32_t s_consumed = 0;
volatile uint32_t s_max_backlog = 0;
volatile uint32_t s_ring_peak = 0;

// Main-task scratch: the link being analysed, and the best (highest coupling)
// still link's series that the web views are drawn from.
float s_y[DSP_BREATH_LEN];
float s_best[DSP_BREATH_LEN];
int   s_best_n = 0;

// Empty-room motion thresholds per link (MAC + CSI format), persisted in NVS
// ("xpol"/"t<mac><fmt>") so a reboot or reflash keeps the calibration.
// Loaded once at init; keys of the older MAC-only format are erased.
struct StoredThr { uint8_t mac[6]; uint8_t fmt; uint64_t thr; };
FixedVec<StoredThr, 16> s_stored;

void nvs_key(const uint8_t* mac, uint8_t fmt, char* key) {   // "f" + 12 + 2 hex = 15 chars (NVS max)
    static const char* hex = "0123456789abcdef";
    key[0] = 'f';
    for (int i = 0; i < 6; i++) { key[1 + 2 * i] = hex[mac[i] >> 4]; key[2 + 2 * i] = hex[mac[i] & 15]; }
    key[13] = hex[fmt >> 4]; key[14] = hex[fmt & 15];
    key[15] = 0;
}

void load_thresholds() {
    nvs_handle_t h;
    if (nvs_open("xpol", NVS_READWRITE, &h) != ESP_OK) return;
    char legacy[16][16]; int nlegacy = 0;
    nvs_iterator_t it = nullptr;
    for (esp_err_t e = nvs_entry_find(NVS_DEFAULT_PART_NAME, "xpol", NVS_TYPE_U64, &it); e == ESP_OK;
         e = nvs_entry_next(&it)) {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);
        // 't…' keys (MAC-only, then MAC + subcarrier count) belong to older link
        // definitions and would never match a link again: erase them.
        if (info.key[0] == 't') { if (nlegacy < 16) strlcpy(legacy[nlegacy++], info.key, 16); continue; }
        if (info.key[0] != 'f') continue;
        StoredThr st{};
        bool ok = strlen(info.key) == 15 && s_stored.size() < s_stored.capacity() &&
                  nvs_get_u64(h, info.key, &st.thr) == ESP_OK;
        for (int i = 0; ok && i < 6; i++) ok = sscanf(info.key + 1 + 2 * i, "%2hhx", &st.mac[i]) == 1;
        if (ok) ok = sscanf(info.key + 13, "%2hhx", &st.fmt) == 1;
        if (ok) s_stored.push_back(st);
    }
    nvs_release_iterator(it);
    for (int i = 0; i < nlegacy; i++) nvs_erase_key(h, legacy[i]);
    if (nlegacy) nvs_commit(h);
    nvs_close(h);
}

void save_threshold(const uint8_t* mac, uint8_t fmt, uint64_t thr) {
    nvs_handle_t h;
    if (nvs_open("xpol", NVS_READWRITE, &h) != ESP_OK) return;
    char key[16]; nvs_key(mac, fmt, key);
    nvs_set_u64(h, key, thr);
    nvs_commit(h);
    nvs_close(h);
    for (auto& st : s_stored) if (!memcmp(st.mac, mac, 6) && st.fmt == fmt) { st.thr = thr; return; }
    StoredThr st{}; memcpy(st.mac, mac, 6); st.fmt = fmt; st.thr = thr;
    s_stored.push_back(st);
}

// Admission score of a transmitter/format: link_quality_db (RSSI + frame-rate
// averaging gain), 3 dB less if the same transmitter is already sensed in
// another format (same geometry, little new information). Caller holds s_mtx.
float link_score(const uint8_t* mac, uint8_t fmt, float rate, float rssi) {
    float s = link_quality_db(rate, rssi);
    for (const auto& l : s_links)
        if (l.used && !memcmp(l.mac, mac, 6) && l.fmt != fmt) { s -= 3.0f; break; }
    return s;
}

void open_link(int slot, const uint8_t* mac, uint8_t fmt, int64_t now) {
    Link& l = s_links[slot];
    memcpy(l.mac, mac, 6);
    l.fmt = fmt;
    l.used = true;
    l.last_seen_us = l.admitted_us = now;
    l.proc.reset();
    l.gain_ref_set = false;
    l.last_hash = 0; l.repeats = 0;
    for (const auto& st : s_stored)
        if (!memcmp(st.mac, mac, 6) && st.fmt == fmt) l.proc.setMotionThreshold(st.thr);
}

// Count a frame of a transmitter/format that is not a link yet; once per
// window decide whether it becomes one. Returns the new link's slot or -1.
// Caller holds s_mtx.
int consider_candidate(const uint8_t* mac, uint8_t fmt, int8_t rssi, int64_t now) {
    constexpr int kN = sizeof(s_cand) / sizeof(s_cand[0]);
    // Eviction prefers a free entry, then the one with the fewest expected
    // frames per window (current count or last window's rate), oldest first on
    // ties: a stream of one-off addresses replaces itself and never displaces
    // a steady transmitter mid-window.
    auto weight = [](const Candidate& c) { return c.used ? std::max<float>(c.cnt, c.rate * kCandWindowUs / 1e6f) : -1.0f; };
    int ci = -1, victim = 0;
    for (int i = 0; i < kN; i++) {
        if (s_cand[i].used && !memcmp(s_cand[i].mac, mac, 6) && s_cand[i].fmt == fmt) { ci = i; break; }
        const float wi = weight(s_cand[i]), wv = weight(s_cand[victim]);
        if (wi < wv || (wi == wv && s_cand[i].last_us < s_cand[victim].last_us)) victim = i;
    }
    if (ci < 0) {
        ci = victim;
        s_cand[ci] = Candidate{};
        memcpy(s_cand[ci].mac, mac, 6);
        s_cand[ci].fmt = fmt;
        s_cand[ci].used = true;
        s_cand[ci].win_start_us = now;
    }
    Candidate& c = s_cand[ci];
    c.cnt++; c.rssi_sum += rssi; c.last_us = now;
    if (now - c.win_start_us < kCandWindowUs) return -1;
    c.rate = c.cnt * 1e6f / (float)(now - c.win_start_us);
    c.rssi = (float)c.rssi_sum / c.cnt;
    c.cnt = 0; c.rssi_sum = 0; c.win_start_us = now;
    // Half the sensing rate: the router's data (~15-30 fr/s) qualifies, a
    // beacon-only AP (~3-10 fr/s) does not take one of the few slots.
    if (c.rate < 0.5f * g_dsp.min_link_rate) { c.better_since_us = 0; return -1; }

    int slot = -1;
    for (int i = 0; i < MAX_LINKS && slot < 0; i++) if (!s_links[i].used) slot = i;
    if (slot < 0) {   // full: the weakest evictable link, if this candidate clearly beats it
        float weakest = 1e9f;
        for (int i = 0; i < MAX_LINKS; i++) {
            const Link& l = s_links[i];
            if (now - l.admitted_us < kLinkGraceUs) continue;
            const float s = link_score(l.mac, l.fmt, l.proc.pktRate(), l.proc.rssi());
            if (s < weakest) { weakest = s; slot = i; }
        }
        if (slot < 0 || link_score(mac, fmt, c.rate, c.rssi) < weakest + kReplaceMarginDb) { c.better_since_us = 0; return -1; }
        if (!c.better_since_us) { c.better_since_us = now; return -1; }
        if (now - c.better_since_us < kReplaceHoldUs) return -1;
    }
    open_link(slot, mac, fmt, now);
    c.used = false;
    return slot;
}

// Caller holds s_mtx.
int find_or_add(const uint8_t* mac, uint8_t fmt, int8_t rssi, int64_t now) {
    for (int i = 0; i < MAX_LINKS; i++) {
        Link& l = s_links[i];
        if (l.used && l.fmt == fmt && !memcmp(l.mac, mac, 6)) { l.last_seen_us = now; return i; }
    }
    for (auto& l : s_links)    // free links that went silent
        if (l.used && now - l.last_seen_us > kLinkIdleUs) l.used = false;
    return consider_candidate(mac, fmt, rssi, now);
}

// Q8 amplitude compensation for a frame of link l at gain indices (agc, fft).
uint16_t link_gain_q8(Link& l, uint8_t agc, int8_t fft) {
    if (l.gain_ref_set && agc == l.agc_last && fft == l.fft_last) return l.gain_q8_last;
    const float g = radio_gain_compensation(agc, fft);
    if (!l.gain_ref_set) { l.gain_ref = g > 0 ? g : 1.0f; l.gain_ref_set = true; }
    const long q = lrintf(256.0f * g / l.gain_ref);
    l.agc_last = agc; l.fft_last = fft;
    l.gain_q8_last = (uint16_t)(q < 1 ? 1 : (q > 65535 ? 65535 : q));
    return l.gain_q8_last;
}

// A live channel estimate never repeats byte for byte; a latched CSI engine
// (seen on both boards, starting and ending at runtime) repeats one payload
// for every frame and transmitter. Such frames never reach the DSP, and a run
// of kFrozenRun on one link asks the main loop to restart CSI capture.
constexpr uint16_t kFrozenRun = 50;
volatile bool     s_csi_restart_req = false;
volatile int64_t  s_last_frozen_us = -10000000;   // last frame of a run of >= 5 repeats (status bar)

static uint32_t fnv1a(const int8_t* p, int n) {
    uint32_t h = 2166136261u;
    for (int i = 0; i < n; i++) h = (h ^ (uint8_t)p[i]) * 16777619u;
    return h;
}

void consume_task(void*) {
    for (;;) {
        size_t len = 0;
        auto* rc = static_cast<const RadioCsi*>(xRingbufferReceive(s_rb, &len, pdMS_TO_TICKS(100)));
        if (!rc) continue;
        const uint32_t backlog = radio_csi_written() - s_consumed;
        if (backlog > s_max_backlog) s_max_backlog = backlog;
        const uint32_t used = sizeof(s_ring_mem) - xRingbufferGetCurFreeSize(s_rb);
        if (used > s_ring_peak) s_ring_peak = used;
        const uint32_t h = fnv1a(rc->iq(), 2 * rc->n_sub);
        xSemaphoreTake(s_mtx, portMAX_DELAY);
        const int i = find_or_add(rc->mac, rc->fmt, rc->rssi, esp_timer_get_time());
        const uint16_t gq8 = i >= 0 ? link_gain_q8(s_links[i], rc->agc_gain, rc->fft_gain) : 256;
        const CsiFrame f{rc->t_us, rc->rssi, rc->n_sub, gq8, rc->iq(), rc->fmt};
        if (i >= 0) {
            Link& l = s_links[i];
            if (h == l.last_hash) {
                if (++l.repeats >= 5) s_last_frozen_us = esp_timer_get_time();
                if (l.repeats == kFrozenRun) s_csi_restart_req = true;
            } else {
                l.repeats = 0;
                l.proc.push(f);
            }
            l.last_hash = h;
        }
        xSemaphoreGive(s_mtx);
        telem_csi(rc->mac, rc->noise, f);  // no-op unless verbose
        telem_iq(rc->mac, rc->noise, f);   // no-op unless raw dump is on
        vRingbufferReturnItem(s_rb, const_cast<RadioCsi*>(rc));
        s_consumed = s_consumed + 1;
    }
}

// Tuned parameters persisted as one NVS blob: a version byte, then every table
// entry's float in table order (a changed table invalidates old blobs).
constexpr uint8_t kParamsVersion = 6;   // 6: "links fused" (1..2N) became "links per board" (1..N)

void load_params() {
    int n; const DspParamInfo* t = dsp_param_table(&n);
    uint8_t blob[1 + 4 * 32];
    size_t len = sizeof(blob);
    nvs_handle_t h;
    if (nvs_open("xpol", NVS_READONLY, &h) != ESP_OK) return;
    const bool ok = nvs_get_blob(h, "params", blob, &len) == ESP_OK && len == 1 + 4 * (size_t)n && blob[0] == kParamsVersion;
    nvs_close(h);
    if (!ok) return;
    for (int i = 0; i < n; i++) { float v; memcpy(&v, blob + 1 + 4 * i, 4); dsp_param_set(i, v); }
    (void)t;
}

QueueHandle_t   s_cmd_q = nullptr;
StaticQueue_t   s_cmd_q_ctl;
uint8_t         s_cmd_q_mem[8];
volatile bool   s_calibrating = false;

} // namespace

RingbufHandle_t sensing_init() {
    load_thresholds();
    load_params();
    s_cmd_q = xQueueCreateStatic(sizeof(s_cmd_q_mem), 1, s_cmd_q_mem, &s_cmd_q_ctl);
    s_rb = xRingbufferCreateStatic(sizeof(s_ring_mem), RINGBUF_TYPE_NOSPLIT, s_ring_mem, &s_ring_ctl);
    s_mtx = xSemaphoreCreateMutex();
    xTaskCreate(consume_task, "csi_consume", 6144, nullptr, 7, nullptr);
    return s_rb;
}

int sensing_stored_calibrations() { return (int)s_stored.size(); }

bool sensing_save_params() {
    int n; const DspParamInfo* t = dsp_param_table(&n);
    uint8_t blob[1 + 4 * 32];
    if (n > 32) return false;
    blob[0] = kParamsVersion;
    for (int i = 0; i < n; i++) { const float v = g_dsp.*(t[i].field); memcpy(blob + 1 + 4 * i, &v, 4); }
    nvs_handle_t h;
    if (nvs_open("xpol", NVS_READWRITE, &h) != ESP_OK) return false;
    const bool ok = nvs_set_blob(h, "params", blob, 1 + 4 * n) == ESP_OK && nvs_commit(h) == ESP_OK;
    nvs_close(h);
    return ok;
}

void sensing_post_command(char c) {
    if (c == 'C') s_calibrating = true;   // visible to /api/params before the main loop picks it up
    if (s_cmd_q) xQueueSend(s_cmd_q, &c, 0);
}
char sensing_take_command() {
    char c = 0;
    return (s_cmd_q && xQueueReceive(s_cmd_q, &c, 0) == pdTRUE) ? c : 0;
}

void sensing_reset_buffers() {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    for (auto& l : s_links) if (l.used) l.proc.resetBuffers();
    xSemaphoreGive(s_mtx);
    s_best_n = 0;
}

bool sensing_calibrating() { return s_calibrating; }

void sensing_set_mode(Mode m) { s_mode = m; }

bool sensing_take_csi_restart() {
    if (!s_csi_restart_req) return false;
    s_csi_restart_req = false;
    for (auto& l : s_links) l.repeats = 0;   // a fresh run is needed to ask again
    return true;
}

void sensing_begin_calibration() {
    s_calibrating = true;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    for (auto& l : s_links) if (l.used) l.proc.beginCalibration();
    xSemaphoreGive(s_mtx);
}

// Finalize, then persist every calibrated link's threshold (flash write, main task).
void sensing_finalize_calibration() {
    struct Done { uint8_t mac[6]; uint8_t fmt; uint64_t thr; };
    FixedVec<Done, MAX_LINKS> done;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    for (auto& l : s_links) {
        if (!l.used) continue;
        l.proc.finalizeCalibration();
        if (l.proc.calibrated()) {
            Done d; memcpy(d.mac, l.mac, 6); d.fmt = l.fmt; d.thr = l.proc.motionThreshold();
            done.push_back(d);
        }
    }
    xSemaphoreGive(s_mtx);
    s_calibrating = false;
    for (const auto& d : done) {
        save_threshold(d.mac, d.fmt, d.thr);
        telem_comment("calibrated %02x%02x%02x%02x%02x%02x/%02x motion_threshold=%.3g",
                      d.mac[0], d.mac[1], d.mac[2], d.mac[3], d.mac[4], d.mac[5], d.fmt,
                      (double)d.thr / 4294967296.0);
    }
}

// One link at a time: snapshot under the lock (motion decision + 1-D breathing
// series, a few ms), then the spectral analysis with the lock released so the
// CSI consumer never waits on it.
int sensing_snapshot(LinkView* out, int max) {
    const int64_t now = esp_timer_get_time();
    int n = 0;
    float best_amp = -1;
    s_best_n = 0;
    for (int i = 0; i < MAX_LINKS && n < max; i++) {
        xSemaphoreTake(s_mtx, portMAX_DELAY);
        if (s_links[i].used && now - s_links[i].last_seen_us > kLinkIdleUs) s_links[i].used = false;   // went silent
        if (!s_links[i].used) { xSemaphoreGive(s_mtx); continue; }
        memcpy(out[n].mac, s_links[i].mac, 6);
        out[n].fmt = s_links[i].fmt;
        const LinkSnapshot snap = s_links[i].proc.snapshot(now, s_y);
        xSemaphoreGive(s_mtx);

        out[n].result = analyzeLink(snap, s_y);
        if (snap.n > 0 && snap.mean_amp > best_amp) {
            best_amp = snap.mean_amp;
            memcpy(s_best, s_y, sizeof(float) * snap.n);
            s_best_n = snap.n;
        }
        n++;
    }
    return n;
}

FixedVec<float, SPEC_MAX> sensing_spectrum() { return breathSpectrum(s_best, s_best_n); }
FixedVec<float, HSPEC_MAX> sensing_heart_spectrum() { return heartSpectrum(s_best, s_best_n); }
float sensing_wave_point(bool* valid) {
    *valid = s_best_n > 0;
    return breathWavePoint(s_best, s_best_n, DSP_DECIM_HZ / TELEM_SCENE_HZ);
}
float sensing_phase(float bpm) { return breathPhase(s_best, s_best_n, bpm); }
bool  sensing_csi_frozen() { return esp_timer_get_time() - s_last_frozen_us < 3000000; }

SensingHealth sensing_health() {
    SensingHealth h{radio_csi_drops(), radio_csi_written(), s_max_backlog,
                    (uint32_t)sizeof(s_ring_mem), (uint32_t)(sizeof(s_ring_mem) - xRingbufferGetCurFreeSize(s_rb)),
                    s_ring_peak};
    s_max_backlog = 0;
    s_ring_peak = 0;
    return h;
}

int sensing_link_macs(uint8_t (*out)[6], int max) {
    int n = 0;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    for (int i = 0; i < MAX_LINKS && n < max; i++)
        if (s_links[i].used) memcpy(out[n++], s_links[i].mac, 6);
    xSemaphoreGive(s_mtx);
    return n;
}

// Index of the strongest link (highest mean amplitude). Caller holds s_mtx.
static int strongest_locked() {
    int best = -1; float bm = -1;
    for (int i = 0; i < MAX_LINKS; i++)
        if (s_links[i].used) { float m = s_links[i].proc.meanAmp(); if (m > bm) { bm = m; best = i; } }
    return best;
}

int sensing_csi_rows(int* slot, uint32_t* cursor, int16_t* out, int max_rows, int* n_sub, int* rssi) {
    int rows = 0;
    *n_sub = 0; *rssi = 0;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    const int i = strongest_locked();
    if (i >= 0) {
        const LinkProcessor& p = s_links[i].proc;
        if (i != *slot) { *slot = i; *cursor = p.rowsTotal(); }   // new link: stream starts now
        *n_sub = p.nSel(); *rssi = p.rssi();
        rows = p.rowsSince(cursor, out, max_rows);
    } else {
        *slot = -1;
    }
    xSemaphoreGive(s_mtx);
    return rows;
}
