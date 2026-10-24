// STEP 2 — LED + leader election + telemetry over the UART link / USB console.
// No Wi-Fi, no CSI, no web. Proves the two boards elect a leader deterministically
// and both stream the decision (and heartbeats / failover) over USB.
//
// Build/flash: pio run -e step2 -t upload --upload-port <port>
// LED: fast blink during election, solid once the role is decided.
// USB (read with DTR asserted): CSV header + election POV + periodic ROLE lines.

#include "config.h"
#include "election.h"
#include "telemetry.h"
#include "link.h"
#include "led.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

static void on_msg(MsgType t, const uint8_t* p, uint16_t len) {
    if (t == MsgType::HELLO && len >= sizeof(MsgHello)) election_feed_hello(((const MsgHello*)p)->mac);
    else if (t == MsgType::HEARTBEAT) election_feed_heartbeat();
}

static void hexmac(const uint8_t* m, char out[13]) {
    snprintf(out, 13, "%02x%02x%02x%02x%02x%02x", m[0], m[1], m[2], m[3], m[4], m[5]);
}

static void status_cb() {
    telem_role(election_role(), election_self_mac(), election_peer_mac(), election_have_peer() ? 1 : 0);
}
static void promote_cb() { telem_comment("promoted FOLLOWER->LEADER telemetry_source=LEADER"); }

void step2_main() {
    led_init();
    led_set(LedMode::BLINK_FAST);
    telem_init();
    election_init();
    link_init(on_msg);

    char self[13], peer[13];
    hexmac(election_self_mac(), self);
    telem_header(election_self_mac(), SENSE_BAND_5G);
    telem_comment("STEP2 led+election+telemetry (no csi, no web) self=%s", self);

    telem_comment("election start self=%s window_ms=%d", self, ELECTION_WINDOW_MS);
    Role role = election_resolve(ELECTION_WINDOW_MS);
    hexmac(election_peer_mac(), peer);
    if (!election_have_peer()) {
        telem_comment("election heard_peer=0 -> this=STANDALONE (no peer within window)");
    } else if (role == Role::LEADER) {
        telem_comment("election decision: self_mac(%s) > peer_mac(%s) -> this=LEADER peer=FOLLOWER", self, peer);
    } else {
        telem_comment("election decision: peer_mac(%s) > self_mac(%s) -> this=FOLLOWER peer=LEADER", peer, self);
    }
    telem_comment("telemetry_source=%s self=%s peer=%s", role_name(role), self,
                  election_have_peer() ? peer : "none");

    led_set(LedMode::SOLID);
    election_start_maintenance(promote_cb);
    telem_start_input(status_cb, nullptr);

    // Periodic ROLE heartbeat so a late USB attach still sees the decision.
    for (;;) {
        telem_role(election_role(), election_self_mac(), election_peer_mac(), election_have_peer() ? 1 : 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
