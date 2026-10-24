// Unified application: one firmware for every board. Role (leader / follower /
// standalone) is decided at runtime by leader election. The leader runs the
// channel survey, hosts the AP + web dashboard at 10.0.0.1, and fuses the scene;
// followers illuminate + sense and report features to the leader over UART.
#include "node_common.h"
#include "config.h"
#include "radio.h"
#include "sensing.h"
#include "scene_state.h"
#include "phone_http.h"
#include "election.h"
#include "telemetry.h"
#include "link.h"
#include "led.h"
#include "diag.h"

#include "nvs_flash.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <vector>

static volatile bool    s_agreed = false;
static MsgChannelAgree  s_agree = {};
static volatile bool    s_have_peer_survey = false;
static MsgSurvey        s_peer_survey = {};

static LinkResult s_peer_links[MAX_LINKS];
static int64_t    s_peer_ts[MAX_LINKS] = {0};

static uint8_t s_channel = SENSE_CH_FALLBACK;
static Mode    s_mode = DEFAULT_MODE;

static volatile bool s_promote_pending = false;
static volatile bool s_recalib = false;
static bool s_ap_up = false;

// ---- UART message handler ----
static void on_msg(MsgType t, const uint8_t* p, uint16_t len) {
    switch (t) {
        case MsgType::HELLO:
            if (len >= sizeof(MsgHello)) election_feed_hello(((const MsgHello*)p)->mac);
            break;
        case MsgType::HEARTBEAT:
            election_feed_heartbeat();
            break;
        case MsgType::SURVEY_RESULT:
            if (len >= sizeof(MsgSurvey)) { memcpy(&s_peer_survey, p, sizeof(s_peer_survey)); s_have_peer_survey = true; }
            break;
        case MsgType::CHANNEL_AGREE:
            if (len >= sizeof(MsgChannelAgree)) { memcpy(&s_agree, p, sizeof(s_agree)); s_agreed = true; }
            break;
        case MsgType::FEATURE_REPORT:
            if (len >= sizeof(MsgFeature)) {
                const MsgFeature* f = (const MsgFeature*)p;
                if (f->link_id < MAX_LINKS) { s_peer_links[f->link_id] = link_from_feature(*f); s_peer_ts[f->link_id] = esp_timer_get_time(); }
            }
            break;
        case MsgType::RF_SOURCES:
            if (len >= offsetof(MsgRfSources, src)) diag_set_peer_sources(*(const MsgRfSources*)p, len);
            break;
        case MsgType::ILLUM_SET:
            if (len >= sizeof(MsgIllumSet) && !election_is_leader()) illum_apply(*(const MsgIllumSet*)p);
            break;
        case MsgType::MODE_SET:
            if (len >= sizeof(MsgModeSet)) s_mode = (Mode)((const MsgModeSet*)p)->mode;
            break;
    }
}

static void status_cb() {
    telem_role(election_role(), election_self_mac(), election_peer_mac(), election_have_peer() ? 1 : 0);
    telem_channel(s_channel, s_agree.second, SENSE_BAND_5G);
}
static void calibrate_cb() { s_recalib = true; }
static void promote_cb()   { s_promote_pending = true; }

static void bring_up_ap() {
    if (s_ap_up) return;
    scene_state_init();
    radio_enable_ap(AP_SSID, AP_PASS, s_channel);
    phone_http_start();
    s_ap_up = true;
    telem_event("ap_up ip=10.0.0.1 ssid=%s channel=%u", AP_SSID, s_channel);
}

#define MARK(x) do { printf("#MARK " x "\n"); fflush(stdout); } while (0)

void app_run() {
    // LED first: instant visual proof the board reached app_run, independent of USB.
    led_init();
    led_set(LedMode::BLINK_FAST);

    // Spam an alive marker for ~3 s so a host can attach and confirm boot + console
    // before any heavy init can crash.
    for (int i = 0; i < 10; i++) { printf("#MARK alive %d\n", i); fflush(stdout); vTaskDelay(pdMS_TO_TICKS(300)); }

    MARK("nvs begin");
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);
    MARK("nvs ok");
    telem_init();
    MARK("telem ok");
    election_init();
    MARK("election ok");
    link_init(on_msg);
    MARK("link ok");
    radio_wifi_start(false);
    radio_set_tx_power_dbm(TX_POWER_DBM);   // both roles (config.h)
    MARK("wifi ok");

    telem_header(election_self_mac(), SENSE_BAND_5G);

    char selfmac[13], peermac[13];
    { const uint8_t* m = election_self_mac();
      snprintf(selfmac, sizeof(selfmac), "%02x%02x%02x%02x%02x%02x", m[0],m[1],m[2],m[3],m[4],m[5]); }

    // --- leader election (narrated as CSV comments, this board's POV) ---
    telem_comment("election start self=%s window_ms=%d", selfmac, ELECTION_WINDOW_MS);
    Role role = election_resolve(ELECTION_WINDOW_MS);
    { const uint8_t* p = election_peer_mac();
      snprintf(peermac, sizeof(peermac), "%02x%02x%02x%02x%02x%02x", p[0],p[1],p[2],p[3],p[4],p[5]); }
    if (!election_have_peer()) {
        telem_comment("election heard_peer=0 -> this=STANDALONE (no peer within window)");
    } else if (role == Role::LEADER) {
        telem_comment("election heard_peer=1 peer=%s", peermac);
        telem_comment("election decision: self_mac(%s) > peer_mac(%s) -> this=LEADER peer=FOLLOWER", selfmac, peermac);
    } else {
        telem_comment("election heard_peer=1 peer=%s", peermac);
        telem_comment("election decision: peer_mac(%s) > self_mac(%s) -> this=FOLLOWER peer=LEADER", peermac, selfmac);
    }
    telem_comment("telemetry_source=%s self=%s peer=%s", role_name(role), selfmac,
                  election_have_peer() ? peermac : "none");
    telem_event("elected role=%s have_peer=%d", role_name(role), election_have_peer() ? 1 : 0);

    // --- channel survey (every board surveys, for diagnostics) ---
    led_set(LedMode::DOUBLE_BLINK);  // CSI background measurement / channel selection
    const uint8_t chans[] = SURVEY_CHANNELS;
    const int nch = sizeof(chans) / sizeof(chans[0]);
    uint16_t counts[16] = {0};
    radio_survey(chans, nch, SURVEY_TOTAL_MS, counts);
    for (int i = 0; i < nch; i++) telem_event("survey ch=%u frames=%u", chans[i], counts[i]);

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
        s_agree.channel = s_channel;
        s_agree.second  = ht40_second_for(s_channel);
        s_agree.band_5g = SENSE_BAND_5G;
        s_agree.mode    = (uint8_t)s_mode;
        for (int i = 0; i < 6; i++) { link_send(MsgType::CHANNEL_AGREE, &s_agree, sizeof(s_agree)); vTaskDelay(pdMS_TO_TICKS(100)); }
        telem_event("channel_plan role=leader channel=%u peer_survey=%d", s_channel, s_have_peer_survey ? 1 : 0);
    } else {
        MsgSurvey sv = {};
        sv.n = (uint8_t)nch;
        for (int i = 0; i < nch; i++) { sv.channel[i] = chans[i]; sv.count[i] = counts[i]; }
        while (!s_agreed) { link_send(MsgType::SURVEY_RESULT, &sv, sizeof(sv)); vTaskDelay(pdMS_TO_TICKS(400)); }
        s_channel = s_agree.channel;
        s_mode = (Mode)s_agree.mode;
        telem_event("channel_plan role=follower channel=%u", s_channel);
    }
    telem_channel(s_channel, s_agree.second, SENSE_BAND_5G);
    led_set(LedMode::SOLID);

    // --- sensing ---
    MARK("survey+plan ok");
    RingbufHandle_t rb = sensing_init();
    sensing_set_mode(s_mode);
    radio_set_sensing_channel(s_channel, s_agree.second, SENSE_BAND_5G);
    MARK("channel set ok");
    radio_start_csi(rb);
    MARK("csi ok");
    radio_start_illuminator(election_is_leader() ? LEADER_ILLUMINATE_RATE_HZ : ILLUMINATE_RATE_HZ);
    MARK("illuminator ok");

    if (election_is_leader()) { bring_up_ap(); MARK("ap ok"); }

    // --- calibration window: only if no empty-room baseline is stored (NVS) ---
    if (s_mode == Mode::RUN && sensing_stored_calibrations() == 0) {
        led_set(LedMode::BLINK_SLOW);
        vTaskDelay(pdMS_TO_TICKS(1500));
        sensing_begin_calibration();
        vTaskDelay(pdMS_TO_TICKS(CALIB_SECONDS * 1000));
        sensing_finalize_calibration();
        telem_event("calibration done");
    }
    led_set(LedMode::SOLID);  // role selected / running (pulses once breathing is found)

    election_start_maintenance(promote_cb);
    telem_start_input(status_cb, calibrate_cb);

    // --- main loop ---
    // ~3 KB with 6 links: static, off the 8 KB main stack (single main loop).
    static LinkView views[MAX_LINKS];
    static FixedVec<LinkResult, MAX_LINKS * 2> all;
    int tick = 0;
    int64_t recalib_until = 0;
    for (;;) {
        if (s_promote_pending) { s_promote_pending = false; bring_up_ap(); telem_comment("promoted FOLLOWER->LEADER telemetry_source=LEADER"); telem_role(election_role(), election_self_mac(), election_peer_mac(), 1); }
        if (sensing_take_csi_restart()) radio_recover_csi();

        if (s_recalib) { s_recalib = false; sensing_begin_calibration(); recalib_until = esp_timer_get_time() + (int64_t)CALIB_SECONDS * 1000000; telem_event("recalibrating"); }
        if (recalib_until && esp_timer_get_time() > recalib_until) { sensing_finalize_calibration(); recalib_until = 0; telem_event("recalibration done"); }

        int n = sensing_snapshot(views, MAX_LINKS);
        all.clear();
        bool emit_feat = (tick % 2) == 0;  // ~2 Hz
        for (int i = 0; i < n; i++) {
            all.push_back(views[i].result);
            if (emit_feat) telem_feat(views[i].mac, views[i].result);
            if (!election_is_leader()) {
                MsgFeature f = feature_from_link((uint8_t)i, views[i].mac, views[i].fmt, views[i].result);
                link_send(MsgType::FEATURE_REPORT, &f, sizeof(f));
            }
        }

        if (election_is_leader()) {
            int64_t now = esp_timer_get_time();
            for (int i = 0; i < MAX_LINKS; i++)
                if (s_peer_ts[i] && now - s_peer_ts[i] < 2000000) all.push_back(s_peer_links[i]);
        }

        SceneResult scene = fuseLinks(all);
        scene.presence = scene.presence || scene.breather_count > 0;   // no tracker in this build
        if (s_ap_up) scene_state_set(scene, s_mode, s_channel);
        telem_scene(scene, s_channel, s_mode);

        // LED: pulse at the strongest breathing rate once one is detected, else solid.
        if (scene.breather_count > 0) {
            const SpectralPeak* best = &scene.breaths[0];
            for (auto& b : scene.breaths) if (b.snr > best->snr) best = &b;
            led_set_bpm(best->rate_bpm);
            led_set(LedMode::PULSE);
        } else {
            led_set(LedMode::SOLID);
        }

        if (tick % 8 == 0) {
            if (!election_is_leader()) { MsgRfSources m; link_send(MsgType::RF_SOURCES, &m, rf_sources_msg(m)); }
            else peer_illum_resend();
        }
        tick++;
        vTaskDelay(pdMS_TO_TICKS(1000 / TELEM_SCENE_HZ));
    }
}
