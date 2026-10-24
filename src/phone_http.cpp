#include "phone_http.h"
#include "scene_state.h"
#include "sensing.h"
#include "diag.h"
#include "election.h"
#include "node_common.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_spiffs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char* TAG = "http";
#define WEB_BASE "/web"

// Mount the SPIFFS image baked from web/dist (partition label "storage").
static void webfs_mount() {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = WEB_BASE,
        .partition_label = "storage",
        .max_files = 6,
        .format_if_mount_failed = false,
    };
    esp_err_t e = esp_vfs_spiffs_register(&conf);
    if (e != ESP_OK) { ESP_LOGE(TAG, "spiffs mount failed: %s", esp_err_to_name(e)); return; }
    size_t total = 0, used = 0;
    esp_spiffs_info("storage", &total, &used);
    ESP_LOGI(TAG, "web fs mounted: %u/%u bytes used", (unsigned)used, (unsigned)total);
}

static const char* content_type(const char* path) {
    const char* dot = strrchr(path, '.');
    if (!dot) return "text/plain";
    if (!strcmp(dot, ".html")) return "text/html";
    if (!strcmp(dot, ".js"))   return "text/javascript";
    if (!strcmp(dot, ".css"))  return "text/css";
    if (!strcmp(dot, ".json")) return "application/json";
    if (!strcmp(dot, ".svg"))  return "image/svg+xml";
    if (!strcmp(dot, ".ico"))  return "image/x-icon";
    if (!strcmp(dot, ".png"))  return "image/png";
    if (!strcmp(dot, ".woff2")) return "font/woff2";
    return "application/octet-stream";
}

// Try to open <path>, else <path>.gz. Reports whether the hit was gzipped.
static FILE* open_asset(const char* fullpath, bool* gz) {
    FILE* f = fopen(fullpath, "r");
    if (f) { *gz = false; return f; }
    char gzp[300];
    strlcpy(gzp, fullpath, sizeof(gzp)); strlcat(gzp, ".gz", sizeof(gzp));  // no vfprintf
    f = fopen(gzp, "r");
    if (f) { *gz = true; return f; }
    return nullptr;
}

// The Wi-Fi driver takes its dynamic receive buffers from the heap. When the
// web drained it (a dashboard load took the leader's free heap from ~21 KB to
// under 1 KB), the CSI engine latched onto one channel estimate and kept
// delivering it until the radio was powered down. Below kHeapFloor the server
// refuses requests (503, retry) and pauses graph frames instead.
static constexpr uint32_t kHeapFloor = 12 * 1024;
static bool heap_low(httpd_req_t* req) {
    if (esp_get_free_heap_size() >= kHeapFloor) return false;
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_set_hdr(req, "Retry-After", "1");
    httpd_resp_send(req, nullptr, 0);
    return true;
}

// The one binary GET left (params, rare): httpd runs handlers one at a time.
alignas(4) static uint8_t s_reply[1536];

static esp_err_t static_get(httpd_req_t* req) {
    if (heap_low(req)) return ESP_OK;
    // Strip query string and map "/" (or any extensionless route) to index.html.
    char uri[200];
    strncpy(uri, req->uri, sizeof(uri) - 1);
    uri[sizeof(uri) - 1] = 0;
    char* q = strchr(uri, '?');
    if (q) *q = 0;

    const char* last = strrchr(uri, '/');
    bool has_ext = last && strchr(last, '.');
    if (!strcmp(uri, "/") || !has_ext) strcpy(uri, "/index.html");  // SPA fallback

    char fullpath[256];
    strlcpy(fullpath, WEB_BASE, sizeof(fullpath)); strlcat(fullpath, uri, sizeof(fullpath));  // no vfprintf

    bool gz = false;
    FILE* f = open_asset(fullpath, &gz);
    if (!f) { httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "not found"); return ESP_FAIL; }

    httpd_resp_set_type(req, content_type(uri));
    if (gz) httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

    char buf[1024];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
        // Each chunk sits in TCP buffers (heap) until acknowledged: let the
        // heap recover before queueing more, rather than starve the Wi-Fi driver.
        for (int w = 0; w < 200 && esp_get_free_heap_size() < kHeapFloor; w++) vTaskDelay(pdMS_TO_TICKS(10));
        if (httpd_resp_send_chunk(req, buf, n) != ESP_OK) { fclose(f); return ESP_FAIL; }
    }
    fclose(f);
    httpd_resp_send_chunk(req, nullptr, 0);
    return ESP_OK;
}

// --- WebSocket ---------------------------------------------------------------
// One socket per browser tab. The page tells the board which view it shows
// ('V' u8 view); the board then pushes only what that view needs, as compact
// binary messages (first byte = type; layouts below, decoded by web/src/wire.js).
// Histories (rate charts, waveform, waterfall, raw CSI) are built by the
// browser from the stream, so the board keeps no display history.
//
//   view       STATUS 1 Hz  SCENE 4 Hz  SPECTRA 4 Hz  RAW 4 Hz  RADIO 1 Hz
//   overview        x           x
//   graphs          x           x            x
//   raw             x                                     x
//   radio           x                                               x
//   tune            x           x            x
//
// IDF owns the client table. The handshake never reaches ws_handler (httpd_uri.c:
// "do not call the uri->handler"), so live clients are enumerated with
// httpd_get_client_list + httpd_ws_get_fd_info, and each client's view lives in
// its session ctx (an integer tag, not an allocation), dropped with the socket.
#define WS_MAX_CLIENTS 3              // == cfg.max_open_sockets (each open socket holds up to TCP_SND_BUF of heap)
static httpd_handle_t s_srv = nullptr;
enum View : uint8_t { VIEW_OVERVIEW = 0, VIEW_GRAPHS, VIEW_RAW, VIEW_RADIO, VIEW_TUNE, VIEW_COUNT };
static void keep_ctx(void*) {}
static View view_of(int fd) {
    const uintptr_t v = (uintptr_t)httpd_sess_get_ctx(s_srv, fd);   // stored as view + 1; 0 = unset
    return v >= 1 && v <= VIEW_COUNT ? (View)(v - 1) : VIEW_OVERVIEW;
}

// Inbound frames (binary or text, little-endian):
//   'V' u8 view         the tab shown (0 overview, 1 graphs, 2 raw, 3 radio, 4 tune)
//   'P' u8 id f32 value set a DSP parameter (applied immediately, clamped)
//   'R' 'C' 'S' 'D'     reset buffers / calibrate / save params / defaults
//                       (queued for the main loop)
//   'I' u16 hz i8 dBm   peer (follower) illumination rate (0 = off) and max TX
//                       power (-128 = unchanged); re-sent to it every 2 s
//   'T' i8 dBm          this board's (leader's) max TX power
//   'L' u16 hz          this board's (leader's) illumination rate (0 = off)
static esp_err_t ws_handler(httpd_req_t* req) {
    uint8_t m[8] = {};
    httpd_ws_frame_t f = {};
    f.payload = m;
    if (httpd_ws_recv_frame(req, &f, sizeof(m)) != ESP_OK) return ESP_FAIL;   // oversize: close, never desync
    if ((f.type != HTTPD_WS_TYPE_TEXT && f.type != HTTPD_WS_TYPE_BINARY) || f.len == 0) return ESP_OK;
    switch (m[0]) {
        case 'V':
            if (f.len == 2 && m[1] < VIEW_COUNT) { req->sess_ctx = (void*)(uintptr_t)(m[1] + 1); req->free_ctx = keep_ctx; }
            break;
        case 'P':
            if (f.len == 6) { float v; memcpy(&v, m + 2, 4); dsp_param_set(m[1], v); }
            break;
        case 'R': case 'C': case 'S': case 'D':
            sensing_post_command((char)m[0]);
            break;
        case 'I':
            if (f.len == 4) { uint16_t hz; memcpy(&hz, m + 1, 2); peer_illum_request(hz, (int8_t)m[3]); }
            break;
        case 'T':
            if (f.len == 2) radio_set_tx_power_dbm((int8_t)m[1]);
            break;
        case 'L':
            if (f.len == 3) { uint16_t hz; memcpy(&hz, m + 1, 2); radio_set_illuminator_rate(hz); }
            break;
        default: break;
    }
    return ESP_OK;
}

// Tune tab: every DSP parameter, self-describing. Binary LE: u8 magic 0xC7,
// u8 count, per param: u8 id, f32 value, f32 min, f32 max, f32 step,
// u8 len + name, u8 len + unit, u8 len + group; then u8 calibrating.
static esp_err_t params_get(httpd_req_t* req) {
    if (heap_low(req)) return ESP_OK;
    auto& buf = s_reply;
    int n; const DspParamInfo* t = dsp_param_table(&n);
    int o = 0;
    auto put = [&](const void* p, int len) { if (o + len <= (int)sizeof(buf)) memcpy(buf + o, p, len); o += len; };
    auto str = [&](const char* s) { uint8_t l = (uint8_t)strlen(s); put(&l, 1); put(s, l); };
    buf[o++] = 0xC7; buf[o++] = (uint8_t)n;
    for (int i = 0; i < n; i++) {
        const uint8_t id = (uint8_t)i;
        const float v[4] = {g_dsp.*(t[i].field), t[i].min, t[i].max, t[i].step};
        put(&id, 1); put(v, sizeof(v));
        str(t[i].name); str(t[i].unit); str(t[i].group);
    }
    const uint8_t cal = sensing_calibrating() ? 1 : 0;
    put(&cal, 1);
    if (o > (int)sizeof(buf)) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "params"); return ESP_FAIL; }
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char*)buf, o);
}

namespace {
struct Out {
    uint8_t* p; int cap; int n = 0;
    void put(const void* v, int len) { if (n + len <= cap) memcpy(p + n, v, len); n += len; }
    void u8(uint8_t v)   { put(&v, 1); }
    void u16(uint32_t v) { const uint16_t x = (uint16_t)(v > 0xFFFF ? 0xFFFF : v); put(&x, 2); }
    void u32(uint32_t v) { put(&v, 4); }
};

// STATUS (0xC0), 40 B: u8 type, u8 channel, u8 links_per_board, u8 reserved,
// then two boards (this one = leader, then the peer), 18 B each:
//   u8 present, u8 cpu_busy %, u32 heap_free, u32 heap_min (last 2 s),
//   i8 tx_q (0.25 dBm), u16 illum_hz, u16 csi_fps, u16 csi_restarts, u8 frozen.
int encode_status(uint8_t* buf, int cap) {
    Out o{buf, cap};
    uint8_t primary = 0; wifi_second_chan_t second;
    esp_wifi_get_channel(&primary, &second);
    o.u8(0xC0); o.u8(primary); o.u8((uint8_t)g_dsp.links_per_board); o.u8(0);
    auto board = [&](bool present, const BoardStatus& b) {
        o.u8(present ? 1 : 0); o.u8(b.cpu_busy); o.u32(b.heap_free); o.u32(b.heap_min);
        o.u8((uint8_t)b.tx_q); o.u16(b.illum_hz); o.u16(b.csi_fps); o.u16(b.csi_restarts); o.u8(b.frozen);
    };
    board(true, diag_board_status());
    BoardStatus peer{};
    board(diag_peer_status(&peer), peer);
    return o.n;
}

// RAW (0xC6): u8 type, u8 frac_bits, i8 rssi, i8 link slot (-1 none), u16 row_ms,
// u8 sub, u8 rows, then rows*sub i16 amplitudes in Q(frac_bits): the strongest
// link's breathing-grid rows written since the previous RAW message. A changed
// slot or sub means a new link: the browser restarts its history.
constexpr int kRawRows = 16;   // > rows per 250 ms tick at the 20 Hz grid (5)
int encode_raw(uint8_t* buf, int cap, int* slot, uint32_t* cursor) {
    static int16_t rows[kRawRows * DSP_NSEL];
    int sub = 0, rssi = 0;
    const int n = sensing_csi_rows(slot, cursor, rows, kRawRows, &sub, &rssi);
    Out o{buf, cap};
    o.u8(0xC6); o.u8(AMP_FRAC_BITS); o.u8((uint8_t)(int8_t)rssi); o.u8((uint8_t)(int8_t)*slot);
    o.u16(1000 / DSP_DECIM_HZ); o.u8((uint8_t)sub); o.u8((uint8_t)n);
    o.put(rows, n * sub * (int)sizeof(int16_t));
    return o.n;
}

// RADIO (0xC8): transmit powers, RF sources heard by this board and by the
// peer, per-task CPU and CSI ring load. Layout: web/src/wire.js decodeRadio.
int encode_radio(uint8_t* buf, int cap) {
    static RfSource     src[16];
    static MsgRfSource  peers[RF_SRC_MAX];
    Out o{buf, cap};
    uint8_t primary = 0; wifi_second_chan_t second;
    esp_wifi_get_channel(&primary, &second);
    wifi_ap_record_t ap = {};
    const bool sta = esp_wifi_sta_get_ap_info(&ap) == ESP_OK;
    const DiagStats d = diag_get();
    BoardStatus ps{};
    const bool peer_ok = diag_peer_status(&ps);

    o.u8(0xC8); o.u8(primary);
    o.u8((uint8_t)(int8_t)radio_tx_power_q()); o.u8((uint8_t)(peer_ok ? ps.tx_q : (int8_t)-128));
    // follower illumination: reported rate (0xFFFF unknown), desired rate, desired TX (-128 = unchanged);
    // then this board's own illumination rate
    const MsgIllumSet want = peer_illum_desired();
    o.u16(peer_ok ? ps.illum_hz : 0xFFFF); o.u16(want.rate_hz); o.u8((uint8_t)want.tx_dbm);
    o.u16((uint32_t)radio_illuminator_rate());
    if (sta) o.put(ap.bssid, 6); else { const uint8_t z[6] = {}; o.put(z, 6); }
    o.u8((uint8_t)(int8_t)(sta ? ap.rssi : -128));
    o.u32(d.heap_free); o.u32(d.heap_min); o.u16(d.loop_ms);
    o.u32(d.csi.written); o.u32(d.csi.drops);
    o.u32(d.csi.ring_size); o.u32(d.csi.ring_used); o.u32(d.csi.ring_peak); o.u16(d.csi.max_backlog);
    o.u8(d.ntask);
    for (int i = 0; i < d.ntask; i++) {
        const uint8_t l = (uint8_t)strnlen(d.task[i].name, sizeof(d.task[i].name));
        o.u8(l); o.put(d.task[i].name, l); o.u16(d.task[i].permille);
    }

    uint8_t links[MAX_LINKS][6];
    const int nl = sensing_link_macs(links, MAX_LINKS);
    const uint8_t* peer = election_have_peer() ? election_peer_mac() : nullptr;
    const int ns = radio_sources(src, sizeof(src) / sizeof(src[0]));
    o.u8((uint8_t)ns);
    // flags: bit0 its CSI fed the scene on the last tick, bit1 the STA's AP,
    // bit2 beacons (AP), bit3 the peer board, bit4 an admitted link on this board.
    for (int i = 0; i < ns; i++) {
        const RfSource& s = src[i];
        uint8_t flags = s.beacon ? 4 : 0;
        for (int k = 0; k < nl; k++) if (!memcmp(links[k], s.mac, 6)) flags |= 16;
        if (used_link(s.mac, false)) flags |= 1;
        if (sta && !memcmp(ap.bssid, s.mac, 6)) flags |= 2;
        if (peer && !memcmp(peer, s.mac, 6)) flags |= 8;
        o.put(s.mac, 6); o.u8((uint8_t)s.rssi_last); o.u8((uint8_t)s.rssi_avg); o.u16(s.rate_x10);
        o.u8(flags); o.u8(s.ssid_len); o.put(s.ssid, s.ssid_len);
    }

    // What the peer hears. flags: bit0 valid, bit1 this board (its STA MAC or
    // its SoftAP MAC = STA + 1), bit2 its CSI fed the scene on the last tick,
    // bit3 beacons (AP), bit4 an admitted link on the peer.
    const uint8_t* self = election_self_mac();
    const int np = diag_peer_sources(peers, RF_SRC_MAX);
    o.u8((uint8_t)np);
    for (int i = 0; i < np; i++) {
        const MsgRfSource& p = peers[i];
        const bool is_self = !memcmp(p.mac, self, 5) && (uint8_t)(p.mac[5] - self[5]) <= 1;
        o.put(p.mac, 6); o.u8((uint8_t)p.rssi_avg); o.u16(p.rate_x10);
        o.u8(1 | (is_self ? 2 : 0) | (used_link(p.mac, true) ? 4 : 0) | ((p.flags & 4) ? 8 : 0) | ((p.flags & 1) ? 16 : 0));
    }
    return o.n;
}
}  // namespace

// 4 Hz. Each message is encoded at most once per tick, however many clients
// want it, and only if one of them does. Below kHeapFloor only STATUS and
// SCENE (small) go out.
static void ws_broadcast_task(void*) {
    constexpr int kSpectra = 1 + 5 + SPEC_MAX + 5 + HSPEC_MAX + 5;
    static uint8_t status_buf[48], scene_buf[512], spectra_buf[kSpectra],
                   raw_buf[8 + kRawRows * DSP_NSEL * 2], radio_buf[1280];
    static_assert(12 + TRACK_MAX * 39 + 1 + MAX_LINKS * 30 <= sizeof(scene_buf), "scene message does not fit");
    int raw_slot = -1; uint32_t raw_cursor = 0;
    int fds[WS_MAX_CLIENTS];
    for (uint32_t tick = 0;; tick++) {
        vTaskDelay(pdMS_TO_TICKS(250));
        size_t cnt = WS_MAX_CLIENTS;
        if (httpd_get_client_list(s_srv, &cnt, fds) != ESP_OK) continue;
        const bool slow = tick % 4 == 0;   // 1 Hz messages
        const bool roomy = esp_get_free_heap_size() >= kHeapFloor;
        int st_n = -1, sc_n = -1, sp_n = -1, raw_n = -1, rad_n = -1;
        for (size_t i = 0; i < cnt; i++) {
            if (httpd_ws_get_fd_info(s_srv, fds[i]) != HTTPD_WS_CLIENT_WEBSOCKET) continue;
            const View v = view_of(fds[i]);
            auto send = [&](uint8_t* buf, int n, int cap) {
                if (n > cap) return;   // never over-read a truncated encoding
                httpd_ws_frame_t f = {};
                f.type = HTTPD_WS_TYPE_BINARY; f.payload = buf; f.len = (size_t)n;
                if (httpd_ws_send_frame_async(s_srv, fds[i], &f) != ESP_OK) httpd_sess_trigger_close(s_srv, fds[i]);
            };
            if (slow) {
                if (st_n < 0) st_n = encode_status(status_buf, sizeof(status_buf));
                send(status_buf, st_n, sizeof(status_buf));
            }
            if (v == VIEW_OVERVIEW || v == VIEW_GRAPHS || v == VIEW_TUNE) {
                if (sc_n < 0) sc_n = scene_state_scene(scene_buf, sizeof(scene_buf));
                send(scene_buf, sc_n, sizeof(scene_buf));
            }
            if (!roomy) continue;
            if (v == VIEW_GRAPHS || v == VIEW_TUNE) {
                if (sp_n < 0) sp_n = scene_state_spectra(spectra_buf, sizeof(spectra_buf));
                send(spectra_buf, sp_n, sizeof(spectra_buf));
            }
            if (v == VIEW_RAW) {
                if (raw_n < 0) raw_n = encode_raw(raw_buf, sizeof(raw_buf), &raw_slot, &raw_cursor);
                send(raw_buf, raw_n, sizeof(raw_buf));
            }
            if (v == VIEW_RADIO && slow) {
                if (rad_n < 0) rad_n = encode_radio(radio_buf, sizeof(radio_buf));
                send(radio_buf, rad_n, sizeof(radio_buf));
            }
        }
    }
}

void phone_http_start() {
    webfs_mount();

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.lru_purge_enable = true;
    cfg.max_open_sockets = WS_MAX_CLIENTS;   // bounds the heap the web can take from the Wi-Fi driver
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.stack_size = 6144;   // our handlers avoid vfprintf, but httpd internals still use it for headers/errors
    if (httpd_start(&s_srv, &cfg) != ESP_OK) { ESP_LOGE(TAG, "httpd start failed"); return; }

    httpd_uri_t ws  = { .uri = "/ws",         .method = HTTP_GET, .handler = ws_handler, .user_ctx = nullptr, .is_websocket = true };
    httpd_uri_t prm = { .uri = "/api/params", .method = HTTP_GET, .handler = params_get, .user_ctx = nullptr };
    httpd_uri_t all = { .uri = "/*",          .method = HTTP_GET, .handler = static_get, .user_ctx = nullptr };
    httpd_register_uri_handler(s_srv, &ws);
    httpd_register_uri_handler(s_srv, &prm);
    httpd_register_uri_handler(s_srv, &all);
    if (xTaskCreate(ws_broadcast_task, "ws_tx", 4096, nullptr, 5, nullptr) != pdPASS)
        ESP_LOGE(TAG, "ws_tx task create failed (heap free=%u)", (unsigned)esp_get_free_heap_size());
    ESP_LOGI(TAG, "http+ws gateway ready");
}
