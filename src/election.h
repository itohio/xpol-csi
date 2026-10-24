#pragma once
// Runtime leader election over the inter-board UART.
// Identity is the board's factory MAC (globally unique). The board with the
// numerically higher MAC becomes leader. If no peer is heard within the window,
// the board becomes a standalone leader. The leader then beacons a heartbeat;
// a follower that loses the heartbeat promotes itself.

#include <stdint.h>

enum class Role : uint8_t { ELECTING, LEADER, FOLLOWER, STANDALONE };

const char* role_name(Role r);

void election_init();                         // read self MAC
void election_feed_hello(const uint8_t mac[6]);
void election_feed_heartbeat();

// Broadcast HELLOs for window_ms, listening for a peer, then decide. Returns role.
Role election_resolve(uint32_t window_ms);

Role           election_role();
bool           election_is_leader();
const uint8_t* election_self_mac();
const uint8_t* election_peer_mac();
bool           election_have_peer();

// Start the post-election maintenance task (leader heartbeat / follower failover).
// promote_cb is invoked (once) if a follower promotes to leader.
void election_start_maintenance(void (*promote_cb)());
