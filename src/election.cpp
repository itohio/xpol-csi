#include "election.h"
#include "config.h"
#include "link.h"
#include "telemetry.h"

#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static uint8_t  s_self[6] = {0};
static uint8_t  s_peer[6] = {0};
static volatile bool s_have_peer = false;
static volatile Role s_role = Role::ELECTING;
static volatile int64_t s_last_hb = 0;
static volatile int64_t s_last_peer_hello = 0;
static void (*s_promote_cb)() = nullptr;
static bool s_task_started = false;

const char* role_name(Role r) {
    switch (r) {
        case Role::LEADER: return "LEADER";
        case Role::FOLLOWER: return "FOLLOWER";
        case Role::STANDALONE: return "STANDALONE";
        default: return "ELECTING";
    }
}

void election_init() {
    // esp_efuse_mac_get_default() is NOT unique across C5 chips; the WIFI_STA MAC is.
    esp_read_mac(s_self, ESP_MAC_WIFI_STA);
}

static int mac_cmp(const uint8_t* a, const uint8_t* b) { return memcmp(a, b, 6); }

void election_feed_hello(const uint8_t mac[6]) {
    if (mac_cmp(mac, s_self) == 0) return;      // ignore our own echo
    memcpy(s_peer, mac, 6);
    s_have_peer = true;
    s_last_peer_hello = esp_timer_get_time();
}

void election_feed_heartbeat() { s_last_hb = esp_timer_get_time(); }

static void hexmac(const uint8_t* m, char out[13]) {
    snprintf(out, 13, "%02x%02x%02x%02x%02x%02x", m[0], m[1], m[2], m[3], m[4], m[5]);
}

// Continuous election: announce HELLO forever and (re)decide the role whenever a
// peer appears or disappears. Robust to boards booting at different times.
static void election_task(void*) {
    MsgHello hello = { .role = 0 };
    memcpy(hello.mac, s_self, 6);
    const int64_t start = esp_timer_get_time();

    for (;;) {
        link_send(MsgType::HELLO, &hello, sizeof(hello));

        // Expire a peer we have not heard from in a while (it left / reset).
        if (s_have_peer && esp_timer_get_time() - s_last_peer_hello > (int64_t)LEADER_TIMEOUT_MS * 1000) {
            s_have_peer = false;
        }

        Role nr;
        if (s_have_peer) {
            nr = (mac_cmp(s_self, s_peer) > 0) ? Role::LEADER : Role::FOLLOWER;
        } else if (esp_timer_get_time() - start > (int64_t)ELECTION_WINDOW_MS * 1000) {
            nr = Role::STANDALONE;  // no peer within the window -> go it alone
        } else {
            nr = Role::ELECTING;
        }

        if (nr != s_role && nr != Role::ELECTING) {
            Role old = s_role;
            s_role = nr;
            char self[13], peer[13];
            hexmac(s_self, self); hexmac(s_peer, peer);
            telem_comment("role=%s self=%s peer=%s", role_name(nr), self,
                          s_have_peer ? peer : "none");
            telem_role(nr, s_self, s_peer, s_have_peer ? 1 : 0);
            if (old == Role::FOLLOWER && nr == Role::LEADER && s_promote_cb) s_promote_cb();
        } else if (nr != Role::ELECTING) {
            s_role = nr;
        }

        // Leader announces liveness; follower watches for its loss.
        if (s_role == Role::LEADER || s_role == Role::STANDALONE) {
            uint8_t beat = 0;
            link_send(MsgType::HEARTBEAT, &beat, 1);
        }

        vTaskDelay(pdMS_TO_TICKS(ELECTION_HELLO_MS));
    }
}

static void start_task() {
    if (s_task_started) return;
    s_task_started = true;
    xTaskCreate(election_task, "election", 3072, nullptr, 8, nullptr);
}

Role election_resolve(uint32_t window_ms) {
    s_role = Role::ELECTING;
    start_task();
    int64_t end = esp_timer_get_time() + (int64_t)window_ms * 1000;
    while (esp_timer_get_time() < end && s_role == Role::ELECTING) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    // Give a brief extra moment for a just-booted peer to be seen.
    vTaskDelay(pdMS_TO_TICKS(300));
    return s_role;
}

Role           election_role()      { return s_role; }
bool           election_is_leader() { return s_role == Role::LEADER || s_role == Role::STANDALONE; }
const uint8_t* election_self_mac()  { return s_self; }
const uint8_t* election_peer_mac()  { return s_peer; }
bool           election_have_peer() { return s_have_peer; }

void election_start_maintenance(void (*promote_cb)()) {
    s_promote_cb = promote_cb;
    start_task();  // already running; ensures it is
}
