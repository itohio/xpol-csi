#include "telemetry.h"
#include "radio.h"
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

static SemaphoreHandle_t s_mtx;
static bool s_verbose = VERBOSE_USB ? true : false;
static volatile bool s_raw = false;   // raw I/Q dump ('r' on, 'q' off)
static void (*s_on_status)() = nullptr;
static void (*s_on_calibrate)() = nullptr;

static uint32_t now_ms() { return (uint32_t)(esp_timer_get_time() / 1000); }

static void mac_hex(const uint8_t* m, char out[13]) {
    static const char* h = "0123456789abcdef";
    for (int i = 0; i < 6; i++) { out[2*i] = h[m[i] >> 4]; out[2*i+1] = h[m[i] & 0xF]; }
    out[12] = 0;
}

static void emit(const char* line) {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    fputs(line, stdout);
    fflush(stdout);
    xSemaphoreGive(s_mtx);
}

void telem_init() {
    s_mtx = xSemaphoreCreateMutex();
    setvbuf(stdout, nullptr, _IONBF, 0);  // USB Serial/JTAG stdout is block-buffered
}
void telem_set_verbose(bool on) { s_verbose = on; }
bool telem_verbose() { return s_verbose; }

void telem_comment(const char* fmt, ...) {
    char body[200];
    va_list ap; va_start(ap, fmt);
    vsnprintf(body, sizeof(body), fmt, ap);
    va_end(ap);
    char line[224];
    snprintf(line, sizeof(line), "# %s\n", body);
    emit(line);
}

void telem_header(const uint8_t self_mac[6], bool band_5g) {
    char sm[13]; mac_hex(self_mac, sm);
    telem_comment("xpol-csi telemetry v1");
    telem_comment("boot self=%s band=%s rate_hz=%d", sm, band_5g ? "5G" : "2.4G", ILLUMINATE_RATE_HZ);
    telem_comment("schema EVT,t_ms,text");
    telem_comment("schema ROLE,t_ms,role,self_mac,peer_mac,peer_count");
    telem_comment("schema CH,t_ms,channel,second,band5g");
    telem_comment("schema CSI,t_ms,src_mac,rssi,noise,nsub,amp0 amp1 ...");
    telem_comment("schema IQ,t_us,src_mac,rssi,noise,gain_q8,nsub,fmt,hex(int8 I0 Q0 I1 Q1 ...)   (raw dump, 'r'; fmt = PHY format id, radio.h)");
    telem_comment("schema FEAT,t_ms,src_mac,valid,motion,is_motion,rate,nb,bpm:snr;..,nh,bpm:snr;..");
    telem_comment("schema LONG,t_ms,src_mac,window_s,nb,bpm:snr;..   (long-window breathing; evidence only)");
    telem_comment("schema SCENE,t_ms,presence,motion,links,nb,bpm:snr;..,nh,bpm:snr;..");
}

void telem_event(const char* fmt, ...) {
    char body[192];
    va_list ap; va_start(ap, fmt);
    vsnprintf(body, sizeof(body), fmt, ap);
    va_end(ap);
    char line[224];
    snprintf(line, sizeof(line), "EVT,%lu,%s\n", (unsigned long)now_ms(), body);
    emit(line);
}

void telem_role(Role r, const uint8_t self_mac[6], const uint8_t peer_mac[6], int peer_count) {
    char sm[13], pm[13];
    mac_hex(self_mac, sm); mac_hex(peer_mac, pm);
    char line[96];
    snprintf(line, sizeof(line), "ROLE,%lu,%s,%s,%s,%d\n",
             (unsigned long)now_ms(), role_name(r), sm, pm, peer_count);
    emit(line);
}

void telem_channel(uint8_t channel, uint8_t second, bool band_5g) {
    char line[64];
    snprintf(line, sizeof(line), "CH,%lu,%u,%u,%u\n",
             (unsigned long)now_ms(), channel, second, band_5g ? 1 : 0);
    emit(line);
}

void telem_csi(const uint8_t src_mac[6], int8_t noise, const CsiFrame& f) {
    if (!s_verbose) return;
    char mac[13]; mac_hex(src_mac, mac);
    // Build in a stack buffer; subcarrier amplitudes are integers (0..~180).
    static char buf[1400];
    int n = snprintf(buf, sizeof(buf), "CSI,%lu,%s,%d,%d,%u,",
                     (unsigned long)now_ms(), mac, f.rssi, noise, f.n_sub);
    for (int i = 0; i < f.n_sub && n < (int)sizeof(buf) - 8; i++)
        n += snprintf(buf + n, sizeof(buf) - n, i ? " %d" : "%d", (csi_amp_q4(f.iq, i, f.gain_q8) + 8) >> AMP_FRAC_BITS);
    if (n < (int)sizeof(buf) - 1) { buf[n++] = '\n'; buf[n] = 0; }
    emit(buf);
}

// Raw CSI for offline analysis: the frame's int8 I/Q as hex (no per-value
// printf, so the consumer task stays cheap). One line per frame.
void telem_iq(const uint8_t src_mac[6], int8_t noise, const CsiFrame& f) {
    if (!s_raw) return;
    static char buf[96 + 4 * MAX_SUBCARRIERS];
    static const char* hex = "0123456789abcdef";
    char mac[13]; mac_hex(src_mac, mac);
    int n = snprintf(buf, sizeof(buf), "IQ,%lu,%s,%d,%d,%u,%u,%u,",
                     (unsigned long)f.t_us, mac, f.rssi, noise, f.gain_q8, f.n_sub, f.fmt);
    const uint8_t* p = reinterpret_cast<const uint8_t*>(f.iq);
    for (int i = 0; i < 2 * f.n_sub && n + 3 < (int)sizeof(buf); i++) {
        buf[n++] = hex[p[i] >> 4]; buf[n++] = hex[p[i] & 15];
    }
    buf[n++] = '\n'; buf[n] = 0;
    emit(buf);
}

static int peaks_str(char* out, int cap, const FixedVec<SpectralPeak, MAX_PEAKS>& v) {
    int n = 0;
    for (size_t i = 0; i < v.size() && n < cap - 16; i++)
        n += snprintf(out + n, cap - n, i ? ";%.1f:%.1f" : "%.1f:%.1f", v[i].rate_bpm, v[i].snr);
    if (n == 0) { out[0] = 0; }
    return n;
}

void telem_feat(const uint8_t src_mac[6], const LinkResult& r) {
    char mac[13]; mac_hex(src_mac, mac);
    char bs[128], hs[128];
    peaks_str(bs, sizeof(bs), r.breaths);
    peaks_str(hs, sizeof(hs), r.hearts);
    char line[320];
    snprintf(line, sizeof(line), "FEAT,%lu,%s,%d,%.4f,%d,%.1f,%d,%s,%d,%s\n",
             (unsigned long)now_ms(), mac, r.valid ? 1 : 0, r.motion_index,
             r.motion ? 1 : 0, r.pkt_rate,
             (int)r.breaths.size(), bs, (int)r.hearts.size(), hs);
    emit(line);
}

void telem_long(const uint8_t src_mac[6], const LongResult& r) {
    char mac[13]; mac_hex(src_mac, mac);
    char bs[160];
    peaks_str(bs, sizeof(bs), r.breaths);
    char line[220];
    snprintf(line, sizeof(line), "LONG,%lu,%s,%.0f,%d,%s\n",
             (unsigned long)now_ms(), mac, r.window_s, (int)r.breaths.size(), bs);
    emit(line);
}

void telem_scene(const SceneResult& s, uint8_t channel, Mode mode) {
    char bs[160], hs[160];
    peaks_str(bs, sizeof(bs), s.breaths);
    peaks_str(hs, sizeof(hs), s.hearts);
    char line[400];
    snprintf(line, sizeof(line),
             "SCENE,%lu,%d,%.4f,%d,%d,%s,%d,%s\n",
             (unsigned long)now_ms(), s.presence ? 1 : 0, s.motion_index,
             s.active_links, s.breather_count, bs, s.heart_count, hs);
    (void)channel; (void)mode;
    emit(line);
}

static void input_task(void*) {
    char line[32];
    for (;;) {
        if (fgets(line, sizeof(line), stdin)) {
            switch (line[0]) {
                case 'v': case 'V':            // CSI stream ON (host asked for the flood)
                    s_verbose = true;
                    telem_event("verbose=1");
                    break;
                case 'q': case 'Q':            // CSI stream + raw dump OFF (back to quiet)
                    s_verbose = false; s_raw = false;
                    telem_event("verbose=0 raw=0");
                    break;
                case 'r':                      // raw I/Q dump ON (offline analysis)
                    s_raw = true;
                    telem_event("raw=1");
                    break;
                case 's': case 'S':
                    if (s_on_status) s_on_status();
                    break;
                case 'c': case 'C':
                    if (s_on_calibrate) s_on_calibrate();
                    break;
                case 't': {                    // t<dBm>: max TX power, e.g. "t2"
                    const int q = radio_set_tx_power_dbm(atoi(line + 1));
                    telem_event("tx_power=%d.%02d dBm", q / 4, 25 * (q % 4));
                    break;
                }
                default: break;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void telem_start_input(void (*on_status)(), void (*on_calibrate)()) {
    s_on_status = on_status;
    s_on_calibrate = on_calibrate;
    xTaskCreate(input_task, "telem_in", 3072, nullptr, 4, nullptr);
}
