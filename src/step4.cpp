// STEP 4 — full pipeline: election + survey + AP/web + CSI capture + ESP-NOW
// illumination + motion/breathing/heartbeat DSP + fusion, all on the surveyed
// 5 GHz channel. Verbose build streams raw CSI; the leader serves the dashboard.

#include "config.h"
#include "election.h"
#include "telemetry.h"
#include "link.h"
#include "led.h"
#include "radio.h"
#include "sensing.h"
#include "node_common.h"
#include "link_select.h"
#include "scene_state.h"
#include "phone_http.h"
#include "diag.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "lwip/ip4_addr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <vector>

static volatile bool s_promote_pending = false;
static bool s_ap_up = false;
static bool s_debug_sta = false;   // DEBUG_STA: STA joined an external AP -> channel pinned by it
static uint8_t s_channel = SENSE_CH_FALLBACK;
static esp_netif_t* s_ap_netif = nullptr;

static volatile bool s_agreed = false;
static volatile bool s_retune_pending = false;   // follower: leader's channel changed
static MsgChannelAgree s_agree = {};
static volatile bool s_have_peer_survey = false;
static MsgSurvey s_peer_survey = {};
static LinkResult s_peer_links[MAX_LINKS];
static uint8_t    s_peer_mac[MAX_LINKS][6];
static uint8_t    s_peer_fmt[MAX_LINKS];
static int64_t    s_peer_ts[MAX_LINKS] = {0};
static Mode s_mode = DEFAULT_MODE;
static VitalTracker s_tracker;

static void on_msg(MsgType t, const uint8_t* p, uint16_t len) {
    if (t == MsgType::HELLO && len >= sizeof(MsgHello)) election_feed_hello(((const MsgHello*)p)->mac);
    else if (t == MsgType::HEARTBEAT) election_feed_heartbeat();
    else if (t == MsgType::SURVEY_RESULT && len >= sizeof(MsgSurvey)) { memcpy(&s_peer_survey, p, sizeof(s_peer_survey)); s_have_peer_survey = true; }
    else if (t == MsgType::CHANNEL_AGREE && len >= sizeof(MsgChannelAgree)) {
        // The leader repeats this every 2 s, so a follower that reboots (or a
        // leader that re-plans a different channel) converges without a restart.
        const MsgChannelAgree* a = (const MsgChannelAgree*)p;
        if (s_agreed && a->channel != s_agree.channel) s_retune_pending = true;
        memcpy(&s_agree, p, sizeof(s_agree));
        s_agreed = true;
    }
    else if (t == MsgType::FEATURE_REPORT && len >= sizeof(MsgFeature)) {
        const MsgFeature* f = (const MsgFeature*)p;
        if (f->link_id < MAX_LINKS) {
            s_peer_links[f->link_id] = link_from_feature(*f);
            memcpy(s_peer_mac[f->link_id], f->mac, 6);
            s_peer_fmt[f->link_id] = f->fmt;
            s_peer_ts[f->link_id] = esp_timer_get_time();
        }
    }
    else if (t == MsgType::RF_SOURCES && len >= offsetof(MsgRfSources, src))
        diag_set_peer_sources(*(const MsgRfSources*)p, len);
    else if (t == MsgType::ILLUM_SET && len >= sizeof(MsgIllumSet) && !election_is_leader())
        illum_apply(*(const MsgIllumSet*)p);
}

static void hexmac(const uint8_t* m, char out[13]) {
    snprintf(out, 13, "%02x%02x%02x%02x%02x%02x", m[0], m[1], m[2], m[3], m[4], m[5]);
}
static void status_cb() {
    telem_role(election_role(), election_self_mac(), election_peer_mac(), election_have_peer() ? 1 : 0);
    // Recording provenance: record_session.py sends 's' first, so every log starts with this.
    telem_comment("radio tx_power_dbm=%.2f illum_hz=%d channel=%u", radio_tx_power_q() / 4.0, radio_illuminator_rate(), s_channel);
}
static volatile bool s_recalib = false;
static void calibrate_cb() { s_recalib = true; }
static void promote_cb() { s_promote_pending = true; telem_comment("promoted FOLLOWER->LEADER telemetry_source=LEADER"); }

static void wifi_apsta_5g_start() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_ap_netif = esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_start());
    telem_comment("wifi country=%s", esp_err_to_name(esp_wifi_set_country_code("US", false)));
    telem_comment("wifi band_5g=%s", esp_err_to_name(esp_wifi_set_band_mode(WIFI_BAND_MODE_5G_ONLY)));
    wifi_protocols_t protos = { .ghz_2g = WIFI_PROTOCOL_11N, .ghz_5g = WIFI_PROTOCOL_11N };
    esp_wifi_set_protocols(WIFI_IF_STA, &protos);
    esp_wifi_set_protocols(WIFI_IF_AP, &protos);
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
}

static void configure_ap_on(uint8_t channel) {
    wifi_config_t ap = {};
    snprintf((char*)ap.ap.ssid, sizeof(ap.ap.ssid), "%s", AP_SSID);
    ap.ap.ssid_len = strlen(AP_SSID);
    snprintf((char*)ap.ap.password, sizeof(ap.ap.password), "%s", AP_PASS);
    ap.ap.channel = channel;
    ap.ap.max_connection = 4;
    ap.ap.authmode = strlen(AP_PASS) >= 8 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    telem_comment("ap set_config=%s ssid=%s ch=%u", esp_err_to_name(esp_wifi_set_config(WIFI_IF_AP, &ap)), AP_SSID, channel);
    esp_netif_dhcps_stop(s_ap_netif);
    esp_netif_ip_info_t ip = {};
    IP4_ADDR(&ip.ip, 10, 0, 0, 1); IP4_ADDR(&ip.gw, 10, 0, 0, 1); IP4_ADDR(&ip.netmask, 255, 255, 255, 0);
    esp_netif_set_ip_info(s_ap_netif, &ip);
    esp_netif_dhcps_start(s_ap_netif);
}

// Leader networking, one or the other per build (never both):
//   AP build (no DEBUG_STA_SSID, env step4_*): SoftAP "xpol-csi" at 10.0.0.1.
//   STA build (DEBUG_STA_SSID, env debug_*): the leader joins the router and is
//     reached on its LAN address; no SoftAP.
// Router + SoftAP together froze the leader's CSI engine from power-on
// (README, "Frozen CSI"); the AP build ran clean.
static void bring_up_ap() {
    if (s_ap_up) return;
    scene_state_init();
#ifdef DEBUG_STA_SSID
    esp_wifi_set_mode(WIFI_MODE_STA);
    telem_comment(s_debug_sta ? "net=sta: dashboard on the LAN address, no SoftAP"
                              : "net=sta: router join failed, no dashboard (STA build has no SoftAP)");
#else
    configure_ap_on(s_channel);
    telem_comment("net=ap ip=10.0.0.1 ssid=%s channel=%u band=5G", AP_SSID, s_channel);
#endif
    phone_http_start();
    s_ap_up = true;
}

void step4_main() {
    led_init();
    led_set(LedMode::BLINK_FAST);
    telem_init();
    // 0x01 = power-on; a reset that keeps power (USB/flash 0x15, software,
    // panic, watchdog) is the case where the leader's CSI has re-latched.
    telem_comment("boot reset_reason=0x%02x", radio_reset_reason());
    election_init();
    link_init(on_msg);

    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    char self[13], peer[13];
    hexmac(election_self_mac(), self);
    telem_header(election_self_mac(), true);
    telem_comment("STEP4 full pipeline self=%s", self);

    wifi_apsta_5g_start();
    radio_set_tx_power_dbm(TX_POWER_DBM);   // both roles (config.h)

    telem_comment("election start self=%s window_ms=%d", self, ELECTION_WINDOW_MS);
    Role role = election_resolve(ELECTION_WINDOW_MS);
    hexmac(election_peer_mac(), peer);
    telem_comment("telemetry_source=%s self=%s peer=%s", role_name(role), self, election_have_peer() ? peer : "none");
    led_set(LedMode::SOLID);
    election_start_maintenance(promote_cb);
    telem_start_input(status_cb, calibrate_cb);

    // --- survey + channel agreement ---
    led_set(LedMode::DOUBLE_BLINK);
    const uint8_t chans[] = SURVEY_CHANNELS;
    const int nch = sizeof(chans) / sizeof(chans[0]);
    uint16_t counts[16] = {0};
    radio_survey(chans, nch, SURVEY_TOTAL_MS, counts);
    for (int i = 0; i < nch; i++) telem_comment("survey ch=%u frames=%u", chans[i], counts[i]);

    if (election_is_leader()) {
        for (int i = 0; i < 20 && !s_have_peer_survey; i++) vTaskDelay(pdMS_TO_TICKS(100));
        uint16_t merged[16];
        for (int i = 0; i < nch; i++) {
            merged[i] = counts[i];
            if (s_have_peer_survey)
                for (int j = 0; j < s_peer_survey.n; j++)
                    if (s_peer_survey.channel[j] == chans[i]) merged[i] += s_peer_survey.count[j];
        }
        s_channel = choose_channel(chans, merged, nch);
#ifdef DEBUG_STA_SSID
        { uint8_t ch = 0; char ip[20] = "?";
          if (DEBUG_STA_SSID[0] && radio_sta_connect(DEBUG_STA_SSID, DEBUG_STA_PASS, &ch, ip, sizeof(ip))) {
              s_channel = ch; s_debug_sta = true;
              telem_comment("DEBUG_STA joined ip=%s channel=%u (sensing pinned to this channel)", ip, ch);
          } else {
              telem_comment("DEBUG_STA connect failed; using surveyed channel=%u", s_channel);
          } }
#endif
        s_agree = {}; s_agree.channel = s_channel; s_agree.band_5g = 1; s_agree.mode = (uint8_t)s_mode;
        for (int i = 0; i < 6; i++) { link_send(MsgType::CHANNEL_AGREE, &s_agree, sizeof(s_agree)); vTaskDelay(pdMS_TO_TICKS(100)); }
        telem_comment("channel_plan role=leader channel=%u", s_channel);
    } else {
        MsgSurvey sv = {}; sv.n = (uint8_t)nch;
        for (int i = 0; i < nch; i++) { sv.channel[i] = chans[i]; sv.count[i] = counts[i]; }
        while (!s_agreed) { link_send(MsgType::SURVEY_RESULT, &sv, sizeof(sv)); vTaskDelay(pdMS_TO_TICKS(400)); }
        s_channel = s_agree.channel; s_mode = (Mode)s_agree.mode;
        telem_comment("channel_plan role=follower channel=%u", s_channel);
    }
    telem_channel(s_channel, 0, true);

    // --- settle the radio on the sensing channel ---
    if (election_is_leader()) {
        bring_up_ap();
    } else {
        esp_wifi_set_mode(WIFI_MODE_STA);
    }
    esp_wifi_set_promiscuous(false);
    if (!s_debug_sta) esp_wifi_set_channel(s_channel, WIFI_SECOND_CHAN_NONE);  // STA already owns the channel

    // --- CSI capture + illumination + sensing ---
    // Both boards illuminate (ESP-NOW broadcasts) and each senses the other:
    // ILLUMINATE_RATE_HZ on the follower, LEADER_ILLUMINATE_RATE_HZ on the leader.
    RingbufHandle_t rb = sensing_init();
    sensing_set_mode(s_mode);
    radio_start_csi(rb);
    radio_start_illuminator(election_is_leader() ? LEADER_ILLUMINATE_RATE_HZ : ILLUMINATE_RATE_HZ);
    led_set(LedMode::SOLID);

    // --- calibration: only if no empty-room baseline is stored (NVS); otherwise
    // a boot with the person in the room would poison it. Runs through the main
    // loop below (calibration samples are taken in sensing_snapshot), a few
    // seconds in so links are admitted and have a frame rate.
    int64_t boot_calib_at = 0;
    if (s_mode == Mode::RUN && sensing_stored_calibrations() == 0) {
        boot_calib_at = esp_timer_get_time() + 6000000;
        telem_comment("no stored calibration: calibrating in 6 s (keep the room empty)");
    } else {
        telem_comment("calibration loaded links=%d (send 'c' in an empty room to redo)", sensing_stored_calibrations());
    }
    led_set(LedMode::SOLID);

    // --- main loop: fuse + report + serve ---
    // ~3 KB with 6 links: static, off the 8 KB main stack (single main loop).
    static LinkView views[MAX_LINKS];
    static FixedVec<LinkResult, MAX_LINKS * 2> all;
    int tick = 0;
    int64_t recalib_until = 0;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        if (s_promote_pending) { s_promote_pending = false; bring_up_ap(); }
        if (sensing_take_csi_restart()) {
            telem_comment("csi frozen (identical frames): recovering, restarts=%u heap=%u min=%u",
                          (unsigned)radio_csi_restarts(), (unsigned)esp_get_free_heap_size(), (unsigned)esp_get_minimum_free_heap_size());
            vTaskDelay(pdMS_TO_TICKS(20));   // let the line out before a possible deep sleep
            radio_recover_csi();
        }
        if (s_retune_pending && !election_is_leader()) {
            s_retune_pending = false;
            s_channel = s_agree.channel;
            esp_wifi_set_channel(s_channel, WIFI_SECOND_CHAN_NONE);
            telem_comment("retuned to the leader's channel=%u", s_channel);
        }
        if (boot_calib_at && esp_timer_get_time() > boot_calib_at) { boot_calib_at = 0; s_recalib = true; }
        // Web tune tab commands (queued by the WebSocket handler).
        for (char c; (c = sensing_take_command()) != 0;) {
            switch (c) {
                case 'R': sensing_reset_buffers(); s_tracker = VitalTracker(); telem_comment("buffers reset"); break;
                case 'C': s_recalib = true; break;
                case 'S': telem_comment("params saved=%d", sensing_save_params() ? 1 : 0); break;
                case 'D': dsp_params_defaults(); telem_comment("params defaults"); break;
                default: break;
            }
        }
        if (s_recalib) { s_recalib = false; sensing_begin_calibration(); recalib_until = esp_timer_get_time() + (int64_t)CALIB_SECONDS * 1000000; led_set(LedMode::BLINK_SLOW); telem_comment("recalibrating"); }
        if (recalib_until && esp_timer_get_time() > recalib_until) { sensing_finalize_calibration(); recalib_until = 0; led_set(LedMode::SOLID); telem_comment("recalibration done"); }

        const int64_t tick_t0 = esp_timer_get_time();
        int n = sensing_snapshot(views, MAX_LINKS);
        all.clear();
        bool emit_feat = (tick % 2) == 0;
        // Fusion candidates: every link on both boards, keyed by transmitter,
        // PHY format and receiving board.
        static LinkResult cand_r[MAX_LINKS * 2];
        static LinkInput  cand_in[MAX_LINKS * 2];
        int nc = 0;
        auto add = [&](const LinkResult& r, const uint8_t* mac, uint8_t fmt, bool peer) {
            LinkInput& in = cand_in[nc];
            memcpy(in.key.mac, mac, 6); in.key.fmt = fmt; in.key.peer = peer ? 1 : 0;
            in.quality_db = link_quality_db(r.pkt_rate, r.rssi);
            in.valid = r.valid;
            cand_r[nc++] = r;
        };
        for (int i = 0; i < n; i++) {
            add(views[i].result, views[i].mac, views[i].fmt, false);
            if (emit_feat) telem_feat(views[i].mac, views[i].result);
            if (!election_is_leader()) {
                MsgFeature f = feature_from_link((uint8_t)i, views[i].mac, views[i].fmt, views[i].result);
                link_send(MsgType::FEATURE_REPORT, &f, sizeof(f));
            }
        }
        if (election_is_leader()) {
            int64_t now = esp_timer_get_time();
            for (int i = 0; i < MAX_LINKS; i++)
                if (s_peer_ts[i] && now - s_peer_ts[i] < 2000000) add(s_peer_links[i], s_peer_mac[i], s_peer_fmt[i], true);
        }
        // Which links feed the scene, chosen per receiving board: the best
        // "links per board" (tune tab, 1..MAX_LINKS, same cap for both boards)
        // by link_quality_db (RSSI + frame-rate gain), smoothed over 5 s, held
        // for at least 20 s, replaced only by a link 3 dB better for 5 s straight.
        static LinkSelector<MAX_LINKS + 4> s_select[2];   // [0] this board, [1] the peer
        bool sel[MAX_LINKS * 2] = {};
        for (int b = 0; b < 2; b++) {
            LinkInput in[MAX_LINKS * 2]; int idx[MAX_LINKS * 2]; int m = 0;
            for (int i = 0; i < nc; i++) if (cand_in[i].key.peer == b) { idx[m] = i; in[m++] = cand_in[i]; }
            bool s[MAX_LINKS * 2];
            s_select[b].update(1.0f / TELEM_SCENE_HZ, in, m, (int)g_dsp.links_per_board, s);
            for (int k = 0; k < m; k++) sel[idx[k]] = s[k];
        }
        UsedLink used[MAX_LINKS * 2]; int nused = 0;
        for (int i = 0; i < nc; i++) {
            if (!sel[i]) continue;
            all.push_back(cand_r[i]);
            memcpy(used[nused].mac, cand_in[i].key.mac, 6); used[nused].peer = cand_in[i].key.peer; nused++;
        }
        used_links_set(used, nused);
        SceneResult scene = fuseLinks(all);

        // --- tracker + web scene (leader serves it) ---
        // Presence and counts come from confirmed tracks: a per-tick detection is
        // one 24 s window (~7 independent trials a minute per link) and would
        // give a few false presence events an hour on noise alone. fuseLinks only
        // contributes gross motion to presence.
        s_tracker.update(1.0f / TELEM_SCENE_HZ, scene.breaths, scene.hearts);
        const auto& tracks = s_tracker.tracks();   // confirmed first
        const int confirmed = s_tracker.confirmedCount();
        int hearts = 0;
        for (const auto& t : tracks) hearts += t.heart_confirmed;
        scene.presence = scene.presence || confirmed > 0;
        scene.breather_count = confirmed;
        scene.heart_count = hearts;
        telem_scene(scene, s_channel, s_mode);
        if (s_ap_up) {
            WebScene& ws = scene_state_begin();   // fills the one shared scene in place (few ms)
            ws.presence = scene.presence;
            ws.motion = scene.motion_index;
            ws.channel = s_channel;
            ws.mode = (int)s_mode;
            ws.active_links = scene.active_links;
            ws.heart_count = scene.heart_count;
            ws.breathers.clear();
            for (const auto& t : tracks) {
                ws.breathers.push_back(WebBreather::from(t));
                ws.breathers.back().phase = sensing_phase(t.breath_bpm);
            }
            ws.breather_count = confirmed;
            ws.spectrum = sensing_spectrum();
            ws.spectrum_fmin_bpm = g_dsp.breath_fmin_bpm;
            ws.wave_point = sensing_wave_point(&ws.wave_valid);
            ws.hspectrum = sensing_heart_spectrum();
            ws.hspectrum_fmin_bpm = g_dsp.heart_fmin_bpm;
            ws.rf.clear();
            for (int i = 0; i < n; i++) {
                WebRf rf; char m[13], fl[8]; hexmac(views[i].mac, m);
                rf.set_label(m, radio_format_label(views[i].fmt, fl)); rf.coupling = views[i].result.mean_amp; rf.rssi = views[i].result.rssi;
                ws.rf.push_back(rf);
            }
            scene_state_end();
        }

        if (confirmed > 0) {
            led_set_bpm(tracks[0].breath_bpm); led_set(LedMode::PULSE);
        } else if (!recalib_until) {
            led_set(LedMode::SOLID);
        }

        if (tick % 8 == 0) {   // every 2 s
            diag_update((int)((esp_timer_get_time() - tick_t0) / 1000));
            if (!election_is_leader()) { MsgRfSources m; link_send(MsgType::RF_SOURCES, &m, rf_sources_msg(m)); }
            else { peer_illum_resend(); link_send(MsgType::CHANNEL_AGREE, &s_agree, sizeof(s_agree)); }
        }
        tick++;
        xTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / TELEM_SCENE_HZ));
    }
}
