#include "scene_state.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>

static SemaphoreHandle_t s_mtx;
static WebScene s_scene;

void scene_state_init() { s_mtx = xSemaphoreCreateMutex(); }

WebScene& scene_state_begin() {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    return s_scene;
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
    scene_state_end();
}

namespace {
struct Wire {
    uint8_t* p; int cap; int n = 0;
    void u8v(uint8_t v)  { if (n + 1 <= cap) p[n] = v; n += 1; }
    void i8v(int8_t v)   { u8v((uint8_t)v); }
    void i16v(int16_t v) { if (n + 2 <= cap) memcpy(p + n, &v, 2); n += 2; }
    void f32v(float v)   { if (n + 4 <= cap) memcpy(p + n, &v, 4); n += 4; }
};

// Power bins as i8 half-dB over the median bin (the detector's noise floor
// reference): -64..+63.5 dB, 1 byte per bin instead of a 4-byte float.
template <typename V>
void bins_half_db(Wire& w, float fmin, const V& p) {
    w.f32v(fmin);
    int n = (int)p.size(); if (n > 255) n = 255;
    w.u8v((uint8_t)n);
    if (n == 0) return;
    float tmp[HSPEC_MAX];
    for (int i = 0; i < n; i++) tmp[i] = p[i];
    std::nth_element(tmp, tmp + n / 2, tmp + n);
    const float med = tmp[n / 2] > 0 ? tmp[n / 2] : 1e-30f;
    for (int i = 0; i < n; i++) {
        const float db = p[i] > 0 ? 10.0f * log10f(p[i] / med) : -64.0f;
        w.i8v((int8_t)std::max(-128.0f, std::min(127.0f, lrintf(2.0f * db) * 1.0f)));
    }
}
}  // namespace

// SCENE (0xC5): u8 type, u8 presence, u8 channel, u8 mode, u8 active_links,
// u8 heart_count, u8 n_tracks, f32 motion;
// tracks[n]: u8 id, u8 confirmed, f32 bpm, ci, conf, snr, heart_bpm, phase, age,
//            heart_ci, heart_snr, u8 heart_confirmed;
// rf: u8 n, then [u8 len, label bytes, f32 coupling, i16 rssi].
int scene_state_scene(uint8_t* buf, int cap) {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    const WebScene& s = s_scene;          // serialize in place: no copy, no heap
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
    }
    xSemaphoreGive(s_mtx);
    return w.n;
}

// SPECTRA (0xC9): u8 type, breath bins, heart bins (each: f32 fmin_bpm, u8 n,
// i8[n] half-dB over the median bin), u8 wave_valid, f32 wave_point.
int scene_state_spectra(uint8_t* buf, int cap) {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    const WebScene& s = s_scene;
    Wire w{buf, cap};
    w.u8v(0xC9);
    bins_half_db(w, s.spectrum_fmin_bpm, s.spectrum);
    bins_half_db(w, s.hspectrum_fmin_bpm, s.hspectrum);
    w.u8v(s.wave_valid ? 1 : 0); w.f32v(s.wave_point);
    xSemaphoreGive(s_mtx);
    return w.n;
}
