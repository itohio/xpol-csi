// STEP 3 — Step 2 + channel survey + Wi-Fi SoftAP + web dashboard on the leader.
// Still no CSI. Flow: elect -> both boards survey the candidate 5 GHz channels and
// pick the least chatty -> leader hosts AP "xpol-csi" at 10.0.0.1 on that channel
// and serves the gzipped web page from SPIFFS. The AP lives on the sensing channel
// because one radio serves the phone and (in step 4) senses on the same channel.

#include "config.h"
#include "election.h"
#include "telemetry.h"
#include "link.h"
#include "led.h"
#include "radio.h"
#include "node_common.h"
#include "scene_state.h"
#include "phone_http.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "lwip/ip4_addr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static volatile bool s_promote_pending = false;
static bool s_ap_up = false;
static uint8_t s_channel = SENSE_CH_FALLBACK;
static esp_netif_t* s_ap_netif = nullptr;

static volatile bool s_agreed = false;
static MsgChannelAgree s_agree = {};
static volatile bool s_have_peer_survey = false;
static MsgSurvey s_peer_survey = {};

static void on_msg(MsgType t, const uint8_t* p, uint16_t len) {
    if (t == MsgType::HELLO && len >= sizeof(MsgHello)) election_feed_hello(((const MsgHello*)p)->mac);
    else if (t == MsgType::HEARTBEAT) election_feed_heartbeat();
    else if (t == MsgType::SURVEY_RESULT && len >= sizeof(MsgSurvey)) { memcpy(&s_peer_survey, p, sizeof(s_peer_survey)); s_have_peer_survey = true; }
    else if (t == MsgType::CHANNEL_AGREE && len >= sizeof(MsgChannelAgree)) { memcpy(&s_agree, p, sizeof(s_agree)); s_agreed = true; }
}

static void hexmac(const uint8_t* m, char out[13]) {
    snprintf(out, 13, "%02x%02x%02x%02x%02x%02x", m[0], m[1], m[2], m[3], m[4], m[5]);
}
static void status_cb() { telem_role(election_role(), election_self_mac(), election_peer_mac(), election_have_peer() ? 1 : 0); }
static void promote_cb() { s_promote_pending = true; telem_comment("promoted FOLLOWER->LEADER telemetry_source=LEADER"); }

// One Wi-Fi session for the whole flow: AP+STA on 5 GHz. Netifs are created
// before esp_wifi_init; band/protocol are set AFTER esp_wifi_start (C5 rule).
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
    esp_wifi_set_promiscuous(false);  // survey done
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
    IP4_ADDR(&ip.ip, 10, 0, 0, 1);
    IP4_ADDR(&ip.gw, 10, 0, 0, 1);
    IP4_ADDR(&ip.netmask, 255, 255, 255, 0);
    esp_netif_set_ip_info(s_ap_netif, &ip);
    esp_netif_dhcps_start(s_ap_netif);
}

static void bring_up_ap() {
    if (s_ap_up) return;
    scene_state_init();
    configure_ap_on(s_channel);
    phone_http_start();
    s_ap_up = true;
    telem_comment("ap_up ip=10.0.0.1 ssid=%s channel=%u band=5G", AP_SSID, s_channel);
}

void step3_main() {
    led_init();
    led_set(LedMode::BLINK_FAST);
    telem_init();
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
    telem_comment("STEP3 led+election+survey+wifi+web (no csi) self=%s", self);

    wifi_apsta_5g_start();

    // --- election (UART) ---
    telem_comment("election start self=%s window_ms=%d", self, ELECTION_WINDOW_MS);
    Role role = election_resolve(ELECTION_WINDOW_MS);
    hexmac(election_peer_mac(), peer);
    telem_comment("telemetry_source=%s self=%s peer=%s", role_name(role), self,
                  election_have_peer() ? peer : "none");
    led_set(LedMode::SOLID);
    election_start_maintenance(promote_cb);
    telem_start_input(status_cb, nullptr);

    // --- channel survey (both boards) ---
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
        s_agree = {}; s_agree.channel = s_channel; s_agree.band_5g = 1;
        for (int i = 0; i < 6; i++) { link_send(MsgType::CHANNEL_AGREE, &s_agree, sizeof(s_agree)); vTaskDelay(pdMS_TO_TICKS(100)); }
        telem_comment("channel_plan role=leader channel=%u peer_survey=%d", s_channel, s_have_peer_survey ? 1 : 0);
    } else {
        MsgSurvey sv = {};
        sv.n = (uint8_t)nch;
        for (int i = 0; i < nch; i++) { sv.channel[i] = chans[i]; sv.count[i] = counts[i]; }
        while (!s_agreed) { link_send(MsgType::SURVEY_RESULT, &sv, sizeof(sv)); vTaskDelay(pdMS_TO_TICKS(400)); }
        s_channel = s_agree.channel;
        telem_comment("channel_plan role=follower channel=%u", s_channel);
    }
    telem_channel(s_channel, 0, true);
    led_set(LedMode::SOLID);

    if (election_is_leader()) {
        bring_up_ap();
    } else {
        esp_wifi_set_promiscuous(false);
        esp_wifi_set_mode(WIFI_MODE_STA);  // follower drops its (default) AP
    }

    SceneResult empty;  // no CSI yet; the page proves AP + server + /api/state work
    for (;;) {
        if (s_promote_pending) { s_promote_pending = false; bring_up_ap(); }
        if (s_ap_up) scene_state_set(empty, DEFAULT_MODE, s_channel);
        telem_role(election_role(), election_self_mac(), election_peer_mac(), election_have_peer() ? 1 : 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
