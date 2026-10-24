#pragma once
// Shared helpers: feature (de)serialization, the peer's link results, the
// per-tick fusion scratch, and survey->plan.

#include "link_proto.h"
#include "link_select.h"
#include "sensing.h"
#include "dsp.h"
#include <stdint.h>

// Unified application entry (same on every board; role decided at runtime).
void app_run();

MsgFeature feature_from_link(uint8_t link_id, const uint8_t* mac, uint8_t fmt, const LinkResult& r);
LinkResult link_from_feature(const MsgFeature& m);

// Leader: the peer's latest per-link results (FEATURE_REPORT), by its link
// index, in PSRAM. Call peer_links_init() before link_init(): the UART task
// stores, the main task reads.
struct PeerLink { LinkResult r; uint8_t mac[6]; uint8_t fmt; int64_t ts_us; };
void      peer_links_init();
void      peer_links_store(const MsgFeature& f, int64_t now_us);
PeerLink* peer_links();   // MAX_LINKS entries

// Leader: the links whose CSI fed the scene on the last tick (chosen by the
// link selector, see link_select.h), by transmitter MAC and receiving board.
struct UsedLink { uint8_t mac[6]; bool peer; };
void used_links_set(const UsedLink* u, int n);
bool used_link(const uint8_t* mac, bool peer);

// Per-tick fusion scratch of the main task (PSRAM, ~25 KB at 16 links): this
// board's link views and the fusion candidates of both boards.
struct FusionWork {
    LinkView   views[MAX_LINKS];
    LinkResult cand_r[2 * MAX_LINKS];
    LinkInput  cand_in[2 * MAX_LINKS];
    bool       sel[2 * MAX_LINKS];
    UsedLink   used[2 * MAX_LINKS];
    FixedVec<LinkResult, 2 * MAX_LINKS> all;
};
FusionWork& fusion_work();   // allocated on the first call (main task)

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
