#pragma once
// Shared helpers: feature (de)serialization and survey->plan.

#include "link_proto.h"
#include "dsp.h"
#include <stdint.h>

// Unified application entry (same on every board; role decided at runtime).
void app_run();

MsgFeature feature_from_link(uint8_t link_id, const uint8_t* mac, uint8_t fmt, const LinkResult& r);
LinkResult link_from_feature(const MsgFeature& m);

// Leader: the links whose CSI fed the scene on the last tick (chosen by the
// link selector, see link_select.h), by transmitter MAC and receiving board.
struct UsedLink { uint8_t mac[6]; bool peer; };
void used_links_set(const UsedLink* u, int n);
bool used_link(const uint8_t* mac, bool peer);

// This node's RF source table as a UART message. Returns the bytes to send
// (header + n entries).
size_t rf_sources_msg(MsgRfSources& m);

// Leader: desired illumination of the peer (web control). Requesting sends it
// at once; peer_illum_resend() repeats it from the main loop every 2 s.
void        peer_illum_request(uint16_t rate_hz, int8_t tx_dbm);
MsgIllumSet peer_illum_desired();
void        peer_illum_resend();
// Follower: apply an ILLUM_SET from the leader.
void        illum_apply(const MsgIllumSet& m);

uint8_t choose_channel(const uint8_t* chans, const uint16_t* counts, int n);
uint8_t ht40_second_for(uint8_t primary);
