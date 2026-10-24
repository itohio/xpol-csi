#include "radio.h"
#include "config.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_rom_sys.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/ip4_addr.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>
#include <algorithm>
#include <stdio.h>

#if __has_include("esp_csi_gain_ctrl.h")
#include "esp_csi_gain_ctrl.h"
#define HAVE_GAIN_CTRL 1
#endif

static const char* TAG = "radio";

// Non-fatal: on the C5 some radio calls can fail (band/channel/CSI combos); we
// log and continue instead of aborting, so the app always reaches its main loop
// and the failure is visible rather than a silent reboot loop.
#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x) do { esp_err_t _e_ = (x); \
    if (_e_ != ESP_OK) ESP_LOGE(TAG, "%s -> %s", #x, esp_err_to_name(_e_)); } while (0)
static RingbufHandle_t s_csi_rb = nullptr;
static volatile uint32_t s_csi_drops = 0, s_csi_written = 0, s_csi_stale = 0;
uint32_t radio_csi_drops() { return s_csi_drops; }
uint32_t radio_csi_written() { return s_csi_written; }
uint32_t radio_csi_stale() { return s_csi_stale; }
static const uint8_t BCAST[6] = {0xff,0xff,0xff,0xff,0xff,0xff};

// --------------------------------------------------------------------------
// RF source table — promiscuous RX callback, runs in the Wi-Fi task. Header
// reads, one linear lookup in 24 entries and integer updates under a spinlock.
// --------------------------------------------------------------------------
namespace {
struct SrcEntry {
    uint8_t  mac[6];
    bool     used, beacon;
    int8_t   rssi_last;
    int16_t  rssi_avg_q4;
    uint16_t cnt_cur, cnt_prev;     // frames in the current / previous whole second
    uint32_t win_s, last_ms;
    uint8_t  ssid_len;
    char     ssid[32];
};
constexpr int kSrcMax = 24;
SrcEntry     s_src[kSrcMax];
portMUX_TYPE s_src_mux = portMUX_INITIALIZER_UNLOCKED;
}  // namespace

static void promisc_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
    const wifi_promiscuous_pkt_t* p = static_cast<const wifi_promiscuous_pkt_t*>(buf);
    // Control frames (ACK/CTS) carry no transmitter address; failed frames a garbled one.
    if (type == WIFI_PKT_CTRL || p->rx_ctrl.rx_state != 0 || p->rx_ctrl.sig_len < 16) return;
    const uint8_t* f = p->payload;
    const uint8_t* ta = f + 10;
    const int len = p->rx_ctrl.sig_len;
    const bool beacon = type == WIFI_PKT_MGMT && f[0] == 0x80;
    const int8_t rssi = (int8_t)p->rx_ctrl.rssi;
    const int64_t now_us = esp_timer_get_time();
    const uint32_t now_ms = (uint32_t)(now_us / 1000), now_s = (uint32_t)(now_us / 1000000);

    portENTER_CRITICAL(&s_src_mux);
    int slot = -1, victim = 0;
    for (int i = 0; i < kSrcMax; i++) {
        if (s_src[i].used && memcmp(s_src[i].mac, ta, 6) == 0) { slot = i; break; }
        if (!s_src[i].used || (s_src[victim].used && s_src[i].last_ms < s_src[victim].last_ms)) victim = i;
    }
    if (slot < 0) {
        slot = victim;
        SrcEntry& n = s_src[slot];
        memcpy(n.mac, ta, 6);
        n.used = true; n.beacon = false; n.ssid_len = 0;
        n.rssi_avg_q4 = (int16_t)(rssi * 16);
        n.cnt_cur = n.cnt_prev = 0; n.win_s = now_s;
    }
    SrcEntry& e = s_src[slot];
    if (now_s != e.win_s) { e.cnt_prev = now_s == e.win_s + 1 ? e.cnt_cur : 0; e.cnt_cur = 0; e.win_s = now_s; }
    if (e.cnt_cur < 0xFFFF) e.cnt_cur++;
    e.rssi_last = rssi;
    e.rssi_avg_q4 = (int16_t)(e.rssi_avg_q4 + ((rssi * 16 - e.rssi_avg_q4) >> 3));
    e.last_ms = now_ms;
    if (beacon) {
        e.beacon = true;
        // Beacon body: 24 B header + 12 B fixed fields, then IEs; the SSID IE (id 0) comes first.
        if (len >= 38 && f[36] == 0 && f[37] <= 32 && 38 + f[37] <= len) {
            e.ssid_len = f[37];
            memcpy(e.ssid, f + 38, e.ssid_len);
        }
    }
    portEXIT_CRITICAL(&s_src_mux);
}

int radio_sources(RfSource* out, int max) {
    const int64_t now_us = esp_timer_get_time();
    const uint32_t now_ms = (uint32_t)(now_us / 1000), now_s = (uint32_t)(now_us / 1000000);
    int n = 0;
    portENTER_CRITICAL(&s_src_mux);   // 24 entries, copies into out only: short
    for (const auto& e : s_src) {
        if (!e.used || now_ms - e.last_ms > 10000) continue;
        const uint16_t rate = e.win_s == now_s ? e.cnt_prev : (e.win_s + 1 == now_s ? e.cnt_cur : 0);
        if (!e.beacon && rate < 2 && e.cnt_cur + e.cnt_prev < 2) continue;   // one-off frame
        const int8_t avg = (int8_t)(e.rssi_avg_q4 / 16);
        int slot = n < max ? n++ : -1;
        if (slot < 0) {   // full: keep the strongest, replace the weakest if this one beats it
            int weakest = 0;
            for (int i = 1; i < n; i++) if (out[i].rssi_avg < out[weakest].rssi_avg) weakest = i;
            if (out[weakest].rssi_avg >= avg) continue;
            slot = weakest;
        }
        RfSource& o = out[slot];
        memcpy(o.mac, e.mac, 6);
        o.rssi_last = e.rssi_last;
        o.rssi_avg = avg;
        o.rate_x10 = (uint16_t)(rate * 10);
        o.beacon = e.beacon;
        o.ssid_len = e.ssid_len;
        memcpy(o.ssid, e.ssid, e.ssid_len);
    }
    portEXIT_CRITICAL(&s_src_mux);
    std::sort(out, out + n, [](const RfSource& a, const RfSource& b) { return a.rssi_avg > b.rssi_avg; });
    return n;
}

int radio_tx_power_q() {
    int8_t q = 0;
    esp_wifi_get_max_tx_power(&q);
    return q;
}

// --------------------------------------------------------------------------
// PHY format of a received frame. One transmitter sends several (an AP: HT data
// and legacy beacons; this project's boards: MCS0 ESP-NOW broadcasts and
// higher-MCS data frames), and each is a different channel estimate (different
// LTF, AGC state and PHY processing), so links are keyed by it: mixing them in
// one link made the per-frame spread jump between two values and the motion
// detector never settle.
// --------------------------------------------------------------------------
uint8_t radio_csi_format(const wifi_pkt_rx_ctrl_t* rx) {
    uint8_t fmt = (uint8_t)(rx->cur_bb_format << 4);
    if (rx->cur_bb_format == RX_BB_FORMAT_HT) fmt |= (uint8_t)(rx->he_siga1 & 0xF);   // HT-SIG MCS
    else if (rx->cur_bb_format == RX_BB_FORMAT_11G) fmt |= (uint8_t)(rx->rate & 0xF);  // L-SIG rate code
    return fmt;
}

const char* radio_format_label(uint8_t fmt, char out[8]) {
    switch (fmt >> 4) {
        case RX_BB_FORMAT_11B: snprintf(out, 8, "11b"); break;
        case RX_BB_FORMAT_11G: snprintf(out, 8, "11g"); break;
        case RX_BB_FORMAT_HT:  snprintf(out, 8, "ht%u", fmt & 0xF); break;
        case RX_BB_FORMAT_VHT: snprintf(out, 8, "vht"); break;
        default:               snprintf(out, 8, "he%u", fmt >> 4); break;
    }
    return out;
}

// --------------------------------------------------------------------------
// CSI receive callback — runs in the Wi-Fi task. Keep it short.
// --------------------------------------------------------------------------
static void csi_cb(void* ctx, wifi_csi_info_t* info) {
    if (!info || !info->buf || !s_csi_rb) return;
    // A frame that failed reception (bad FCS) has an untrustworthy transmitter
    // address: with legacy CSI on, noise-level triggers produced ~140 one-off
    // garbage MACs per second. Control frames carry no transmitter (all-zero).
    if (info->rx_ctrl.rx_state != 0) return;
    static const uint8_t kZero[6] = {};
    if (!memcmp(info->mac, kZero, 6)) return;
    // The C5 flags whether this frame produced a channel estimate; without one
    // info->buf holds the last estimate the hardware made (a leader board kept
    // delivering one frozen payload for every frame and transmitter).
    if (!info->rx_ctrl.rx_channel_estimate_info_vld) { s_csi_stale = s_csi_stale + 1; return; }

    int nsub = info->len / 2;
    if (nsub > MAX_SUBCARRIERS) nsub = MAX_SUBCARRIERS;
    RadioCsi* rc = nullptr;
    if (xRingbufferSendAcquire(s_csi_rb, (void**)&rc, sizeof(RadioCsi) + 2 * nsub, 0) != pdTRUE) {
        s_csi_drops++;
        return;  // ring full: drop this frame rather than stall the Wi-Fi task
    }

    // Only the raw gain indices here: no float math in the Wi-Fi task. The
    // consumer turns them into a per-link compensation (radio_gain_compensation).
    const wifi_pkt_rx_ctrl_t* rx = &info->rx_ctrl;
    uint8_t agc = 0; int8_t fft = 0;
#if defined(HAVE_GAIN_CTRL) && WIFI_CSI_PHY_GAIN_ENABLE
    static int s_cnt = 0;
    esp_csi_gain_ctrl_get_rx_gain(rx, &agc, &fft);
    if (s_cnt < 100) { esp_csi_gain_ctrl_record_rx_gain(agc, fft); s_cnt++; }
#endif

    memcpy(rc->mac, info->mac, 6);
    rc->rssi     = (int8_t)rx->rssi;
    rc->noise    = (int8_t)rx->noise_floor;
    rc->t_us     = (uint32_t)rx->timestamp;
    rc->agc_gain = agc;
    rc->fft_gain = fft;
    rc->n_sub   = (uint8_t)nsub;
    rc->fmt     = radio_csi_format(rx);
    memcpy(rc->iq(), info->buf, 2 * nsub);  // the one unavoidable copy out of the driver buffer
    s_csi_written = s_csi_written + 1;     // before publishing: backlog never reads negative
    xRingbufferSendComplete(s_csi_rb, rc);
}

float radio_gain_compensation(uint8_t agc, int8_t fft) {
#if defined(HAVE_GAIN_CTRL) && WIFI_CSI_PHY_GAIN_ENABLE
    float g = 1.0f;
    esp_csi_gain_ctrl_get_gain_compensation(&g, agc, fft);
    return g;
#else
    (void)agc; (void)fft;
    return 1.0f;
#endif
}

// --------------------------------------------------------------------------
// Wi-Fi bring-up
// --------------------------------------------------------------------------
void radio_wifi_start(bool apsta) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    if (apsta) { esp_netif_create_default_wifi_ap(); }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(apsta ? WIFI_MODE_APSTA : WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Band / protocol / bandwidth (C5 dual band).
#if SENSE_BAND_5G
    esp_wifi_set_band_mode(WIFI_BAND_MODE_5G_ONLY);
#else
    esp_wifi_set_band_mode(WIFI_BAND_MODE_2G_ONLY);
#endif
    wifi_protocols_t protos = { .ghz_2g = WIFI_PROTOCOL_11N, .ghz_5g = WIFI_PROTOCOL_11N };
    esp_wifi_set_protocols(WIFI_IF_STA, &protos);
#if SENSE_USE_HT40
    wifi_bandwidths_t bw = { .ghz_2g = WIFI_BW_HT40, .ghz_5g = WIFI_BW_HT40 };
#else
    wifi_bandwidths_t bw = { .ghz_2g = WIFI_BW_HT20, .ghz_5g = WIFI_BW_HT20 };
#endif
    esp_wifi_set_bandwidths(WIFI_IF_STA, &bw);

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    // Identity is the factory MAC (unique per board); no override needed.
    ESP_LOGI(TAG, "wifi up (%s), band=%s", apsta ? "apsta" : "sta",
             SENSE_BAND_5G ? "5G" : "2.4G");
}

void radio_set_sensing_channel(uint8_t primary, uint8_t second, bool) {
    ESP_ERROR_CHECK(esp_wifi_set_channel(primary, (wifi_second_chan_t)second));
    ESP_LOGI(TAG, "sensing channel=%u second=%u", primary, second);
}

void radio_enable_ap(const char* ssid, const char* pass, uint8_t channel) {
    // Followers run STA-only; the leader calls this after election to bring up
    // the AP interface, switch to AP+STA, and serve the dashboard at 10.0.0.1.
    esp_netif_create_default_wifi_ap();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    wifi_config_t ap = {};
    snprintf((char*)ap.ap.ssid, sizeof(ap.ap.ssid), "%s", ssid);
    ap.ap.ssid_len = strlen((char*)ap.ap.ssid);
    snprintf((char*)ap.ap.password, sizeof(ap.ap.password), "%s", pass);
    ap.ap.channel = channel;
    ap.ap.max_connection = 4;
    ap.ap.authmode = strlen(pass) >= 8 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));

    // Serve the dashboard at 10.0.0.1 instead of the default 192.168.4.1.
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (netif) {
        esp_netif_dhcps_stop(netif);
        esp_netif_ip_info_t ip = {};
        IP4_ADDR(&ip.ip, 10, 0, 0, 1);
        IP4_ADDR(&ip.gw, 10, 0, 0, 1);
        IP4_ADDR(&ip.netmask, 255, 255, 255, 0);
        esp_netif_set_ip_info(netif, &ip);
        esp_netif_dhcps_start(netif);
    }
    ESP_LOGI(TAG, "softap '%s' up at 10.0.0.1", ssid);
}

bool radio_sta_connect(const char* ssid, const char* pass, uint8_t* out_ch, char* out_ip, int ip_len) {
    wifi_config_t sta = {};
    strlcpy((char*)sta.sta.ssid, ssid, sizeof(sta.sta.ssid));
    strlcpy((char*)sta.sta.password, pass, sizeof(sta.sta.password));
    if (esp_wifi_set_config(WIFI_IF_STA, &sta) != ESP_OK) return false;
    esp_wifi_connect();
    esp_netif_t* nif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip = {};
    for (int i = 0; i < 120; i++) {              // up to ~12 s for assoc + DHCP
        vTaskDelay(pdMS_TO_TICKS(100));
        if (nif && esp_netif_get_ip_info(nif, &ip) == ESP_OK && ip.ip.addr != 0) {
            if (out_ch) { wifi_second_chan_t sec = WIFI_SECOND_CHAN_NONE; esp_wifi_get_channel(out_ch, &sec); }
            if (out_ip) snprintf(out_ip, ip_len, IPSTR, IP2STR(&ip.ip));
            ESP_LOGI(TAG, "debug STA joined '%s' ip=" IPSTR, ssid, IP2STR(&ip.ip));
            return true;
        }
    }
    return false;
}

// --------------------------------------------------------------------------
// ESP-NOW illuminator
// --------------------------------------------------------------------------
static volatile int s_illum_rate = 0;   // 0 = off; settable at runtime
static bool s_illum_started = false;    // ESP-NOW is up and the illuminator task exists
static portMUX_TYPE s_illum_mux = portMUX_INITIALIZER_UNLOCKED;

static void illuminator_task(void*) {
    uint32_t seq = 0;
    // vTaskDelayUntil yields so IDLE/other tasks run (a busy-wait starves the
    // watchdog). With FREERTOS_HZ=1000, 100 Hz -> 10 ticks. The rate is re-read
    // every period so the web can change it or switch it off.
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        const int rate = s_illum_rate;
        if (rate <= 0) { vTaskDelay(pdMS_TO_TICKS(100)); last = xTaskGetTickCount(); continue; }
        esp_now_send(BCAST, (const uint8_t*)&seq, sizeof(seq));
        seq++;
        TickType_t period = pdMS_TO_TICKS(1000 / rate);
        vTaskDelayUntil(&last, period < 1 ? 1 : period);
    }
}

// ESP-NOW + broadcast peer + illuminator task, once, on the first nonzero rate.
// Not at 0 Hz: the leader never brought ESP-NOW up while it ran clean
// (2026-10-19), so it stays down until illumination is actually requested.
static void start_espnow() {
    portENTER_CRITICAL(&s_illum_mux);
    const bool first = !s_illum_started;
    s_illum_started = true;
    portEXIT_CRITICAL(&s_illum_mux);
    if (!first) return;
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, BCAST, 6);
    peer.ifidx = WIFI_IF_STA;
    peer.channel = 0;      // use current channel
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    esp_now_rate_config_t rc = {};
#if SENSE_USE_HT40
    rc.phymode = WIFI_PHY_MODE_HT40;
#else
    rc.phymode = WIFI_PHY_MODE_HT20;
#endif
    rc.rate = WIFI_PHY_RATE_MCS0_LGI;
    esp_now_set_peer_rate_config(BCAST, &rc);
    xTaskCreate(illuminator_task, "illum", 3072, nullptr, 6, nullptr);
}

void radio_set_illuminator_rate(int rate_hz) {
    const int r = rate_hz < 0 ? 0 : (rate_hz > ILLUMINATE_MAX_HZ ? ILLUMINATE_MAX_HZ : rate_hz);
    if (r > 0) start_espnow();
    if (r != s_illum_rate) ESP_LOGI(TAG, "illuminator %d Hz", r);
    s_illum_rate = r;
}
int radio_illuminator_rate() { return s_illum_started ? s_illum_rate : 0; }

void radio_start_illuminator(int rate_hz) { radio_set_illuminator_rate(rate_hz); }

int radio_set_tx_power_dbm(int dbm) {
    const int q = dbm * 4 < 8 ? 8 : (dbm * 4 > 84 ? 84 : dbm * 4);   // driver range, 0.25 dBm units
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power((int8_t)q));
    int8_t got = 0;
    esp_wifi_get_max_tx_power(&got);
    return got;
}

// --------------------------------------------------------------------------
// CSI enable (C5 / Wi-Fi 6 acquire config)
// --------------------------------------------------------------------------
static wifi_csi_config_t csi_config() {
    wifi_csi_config_t csi = {};
    csi.enable                   = true;
    csi.acquire_csi_legacy       = true;    // beacons etc.: every AP on the channel is a candidate (own link per format)
    csi.acquire_csi_force_lltf   = false;
    csi.acquire_csi_ht20         = true;
    csi.acquire_csi_ht40         = true;
    csi.acquire_csi_vht          = false;
    csi.acquire_csi_su           = false;
    csi.acquire_csi_mu           = false;
    csi.acquire_csi_dcm          = false;
    csi.acquire_csi_beamformed   = false;
    csi.acquire_csi_he_stbc_mode = 2;
    csi.val_scale_cfg            = 0;
    csi.dump_ack_en              = false;
    // L-LTF I/Q as int8 pairs (1), like every other format. The default (0) is
    // 12-bit packed (4 bytes per subcarrier: our 11g links were misparsed) and,
    // per esp-idf#18493, left the C5's CSI buffer frozen on 5 GHz for a station
    // connected to an AP (the leader joins the router).
    csi.lltf_bit_mode            = 1;
    return csi;
}

void radio_start_csi(RingbufHandle_t rb) {
    s_csi_rb = rb;
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(promisc_cb));   // RF source table
    const wifi_csi_config_t csi = csi_config();
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_cb, nullptr));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
    ESP_LOGI(TAG, "CSI capture on");
}

static volatile uint32_t s_csi_restarts = 0;
uint32_t radio_csi_restarts() { return s_csi_restarts; }

static void restart_csi() {
    const wifi_csi_config_t csi = csi_config();
    esp_wifi_set_csi(false);
    vTaskDelay(pdMS_TO_TICKS(20));
    esp_wifi_set_csi_config(&csi);
    esp_wifi_set_csi(true);
    s_csi_restarts = s_csi_restarts + 1;
}

unsigned radio_reset_reason() { return (unsigned)esp_rom_get_reset_reason(0); }

// A latched CSI engine (one payload repeated for every frame) is not cleared by
// restarting capture, a USB/software reset, an RTC-domain reset or deep sleep;
// only a power cut has cleared it (README, "Frozen CSI"). Restarting capture is
// cheap and harmless, so retry it up to kRestarts times within kStreakUs and
// then at most every kBackoffUs; the board keeps running and reports `frozen`.
bool radio_recover_csi() {
    constexpr int      kRestarts   = 3;
    constexpr int64_t  kStreakUs   = 10000000;
    constexpr int64_t  kBackoffUs  = 30000000;
    static int64_t last_us = 0, last_restart_us = -kBackoffUs;
    static int     streak = 0;
    const int64_t now = esp_timer_get_time();
    if (now - last_us > kStreakUs) streak = 0;
    last_us = now;
    if (++streak <= kRestarts || now - last_restart_us >= kBackoffUs) { restart_csi(); last_restart_us = now; }
    return false;
}

// --------------------------------------------------------------------------
// Channel survey
// --------------------------------------------------------------------------
static volatile uint16_t s_survey_count = 0;

static void survey_cb(void* buf, wifi_promiscuous_pkt_type_t type) {
    (void)buf; (void)type;
    s_survey_count = s_survey_count + 1;
}

void radio_survey(const uint8_t* chans, int n, int total_ms, uint16_t* counts) {
    wifi_promiscuous_filter_t filt = {};
    filt.filter_mask = WIFI_PROMIS_FILTER_MASK_ALL;
    esp_wifi_set_promiscuous_filter(&filt);
    esp_wifi_set_promiscuous_rx_cb(survey_cb);
    esp_wifi_set_promiscuous(true);

    const int dwell = total_ms / (n > 0 ? n : 1);
    for (int i = 0; i < n; i++) {
        esp_wifi_set_channel(chans[i], WIFI_SECOND_CHAN_NONE);
        s_survey_count = 0;
        vTaskDelay(pdMS_TO_TICKS(dwell));
        counts[i] = s_survey_count;
        ESP_LOGI(TAG, "survey ch %u -> %u frames", chans[i], counts[i]);
    }
    esp_wifi_set_promiscuous_rx_cb(nullptr);
}
