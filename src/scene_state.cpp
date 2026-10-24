#include "scene_state.h"
#include "psram.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>

static SemaphoreHandle_t s_mtx;
static WebScene* s_scene = nullptr;   // PSRAM (~3 KB, mostly the long-window spectrum)

void scene_state_init() {
    s_scene = psram_new<WebScene>("web scene");
    s_mtx = xSemaphoreCreateMutex();
}

WebScene& scene_state_begin() {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    return *s_scene;
}

void scene_state_end() { xSemaphoreGive(s_mtx); }

void scene_state_set(const SceneResult& s, Mode mode, uint8_t channel) {
    if (!s_mtx) return;
    WebScene& w = scene_state_begin();
    w.presence = s.presence; w.motion = s.motion_index;
    w.channel = channel; w.mode = (int)mode;
    w.active_links = s.active_links;
    w.breather_count = s.breather_count; w.heart_count = s.heart_count;
    w.breathers.clear();
    for (auto& b : s.breaths) {
        WebBreather t;
        t.breath_bpm = b.rate_bpm; t.breath_amp = b.snr; t.confidence = 1;
        t.confirmed = b.snr >= g_dsp.breath_snr_db;
        w.breathers.push_back(t);
    }
    w.long_window_s = s.long_window_s; w.long_detect_db = g_dsp.long_snr_db; w.long_breaths = s.long_breaths;
    scene_state_end();
}

namespace {
struct Wire {
    uint8_t* p; int cap; int n = 0;
    void u8v(uint8_t v)  { if (n + 1 <= cap) p[n] = v; n += 1; }
    void i8v(int8_t v)   { u8v((uint8_t)v); }
    void i16v(int16_t v) { if (n + 2 <= cap) memcpy(p + n, &v, 2); n += 2; }
    void u16v(uint16_t v) { if (n + 2 <= cap) memcpy(p + n, &v, 2); n += 2; }
    void f32v(float v)   { if (n + 4 <= cap) memcpy(p + n, &v, 4); n += 4; }
};

// Power bins as i8 half-dB over the median bin (the detector's noise floor
// reference): -64..+63.5 dB, 1 byte per bin instead of a 4-byte float.
// Serializers run in the WebSocket task under the scene mutex: one scratch.
float s_med[LONG_SPEC_MAX];
void half_db(Wire& w, const float* p, int n) {
    if (n == 0) return;
    std::copy(p, p + n, s_med);
    std::nth_element(s_med, s_med + n / 2, s_med + n);
    const float med = s_med[n / 2] > 0 ? s_med[n / 2] : 1e-30f;
    for (int i = 0; i < n; i++) {
        const float db = p[i] > 0 ? 10.0f * log10f(p[i] / med) : -64.0f;
        w.i8v((int8_t)std::max(-128.0f, std::min(127.0f, lrintf(2.0f * db) * 1.0f)));
    }
}
template <typename V>
void bins_half_db(Wire& w, float fmin, const V& p) {
    w.f32v(fmin);
    const int n = std::min((int)p.size(), 255);
    w.u8v((uint8_t)n);
    half_db(w, p.begin(), n);
}
}  // namespace

// SCENE (0xC5): u8 type, u8 presence, u8 channel, u8 mode, u8 active_links,
// u8 heart_count, u8 n_tracks, f32 motion;
// tracks[n]: u8 id, u8 confirmed, f32 bpm, ci, conf, snr, heart_bpm, phase, age,
//            heart_ci, heart_snr, u8 heart_confirmed;
// rf: u8 n, then [u8 len, label bytes, f32 coupling, i16 rssi, u8 flags (bit0
//     feeds the scene), f32 long_bpm (0: none), f32 long_snr];
// long window: f32 window_s, f32 detect_db, u8 n, then [f32 bpm, f32 snr].
int scene_state_scene(uint8_t* buf, int cap) {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    const WebScene& s = *s_scene;         // serialize in place: no copy, no heap
    Wire w{buf, cap};
    const int nb = std::min((int)s.breathers.size(), 255);
    w.u8v(0xC5); w.u8v(s.presence ? 1 : 0); w.u8v((uint8_t)s.channel); w.u8v((uint8_t)s.mode);
    w.u8v((uint8_t)s.active_links); w.u8v((uint8_t)s.heart_count); w.u8v((uint8_t)nb);
    w.f32v(s.motion);
    for (int i = 0; i < nb; i++) {
        const WebBreather& t = s.breathers[i];
        w.u8v((uint8_t)t.id); w.u8v(t.confirmed ? 1 : 0);
        w.f32v(t.breath_bpm); w.f32v(t.breath_ci); w.f32v(t.confidence); w.f32v(t.breath_amp);
        w.f32v(t.heart_bpm); w.f32v(t.phase); w.f32v(t.age_s);
        w.f32v(t.heart_ci); w.f32v(t.heart_amp); w.u8v(t.heart_confirmed ? 1 : 0);
    }
    const int nrf = std::min((int)s.rf.size(), 255);
    w.u8v((uint8_t)nrf);
    for (int i = 0; i < nrf; i++) {
        const WebRf& r = s.rf[i];
        const int ml = std::min((int)strlen(r.mac), (int)sizeof(r.mac) - 1);
        w.u8v((uint8_t)ml);
        for (int k = 0; k < ml; k++) w.u8v((uint8_t)r.mac[k]);
        w.f32v(r.coupling); w.i16v((int16_t)r.rssi);
        w.u8v(r.used ? 1 : 0); w.f32v(r.long_bpm); w.f32v(r.long_snr);
    }
    w.f32v(s.long_window_s); w.f32v(s.long_detect_db);
    w.u8v((uint8_t)s.long_breaths.size());
    for (const auto& p : s.long_breaths) { w.f32v(p.rate_bpm); w.f32v(p.snr); }
    xSemaphoreGive(s_mtx);
    return w.n;
}

// SPECTRA (0xC9): u8 type, breath bins, heart bins (each: f32 fmin_bpm, u8 n,
// i8[n] half-dB over the median bin), u8 wave_valid, f32 wave_point; then the
// long-window spectrum: f32 fmin_bpm, f32 bin_bpm, f32 window_s, u16 n, i8[n].
int scene_state_spectra(uint8_t* buf, int cap) {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    const WebScene& s = *s_scene;
    Wire w{buf, cap};
    w.u8v(0xC9);
    bins_half_db(w, s.spectrum_fmin_bpm, s.spectrum);
    bins_half_db(w, s.hspectrum_fmin_bpm, s.hspectrum);
    w.u8v(s.wave_valid ? 1 : 0); w.f32v(s.wave_point);
    w.f32v(s.lspec_fmin_bpm); w.f32v(s.lspec_bin_bpm); w.f32v(s.lspec_window_s);
    w.u16v((uint16_t)s.lspectrum.size());
    half_db(w, s.lspectrum.begin(), (int)s.lspectrum.size());
    xSemaphoreGive(s_mtx);
    return w.n;
}
