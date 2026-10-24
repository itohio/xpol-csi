#include "node_common.h"
#include "config.h"
#include "esp_wifi.h"
#include "radio.h"
#include "sensing.h"
#include "link.h"
#include "diag.h"
#include <stddef.h>
#include <string.h>

MsgFeature feature_from_link(uint8_t link_id, const uint8_t* mac, uint8_t fmt, const LinkResult& r) {
    MsgFeature m = {};
    m.source = 0;
    m.link_id = link_id;
    memcpy(m.mac, mac, 6);
    m.fmt = fmt;
    m.valid = r.valid ? 1 : 0;
    m.rssi = r.rssi;
    m.pkt_rate = r.pkt_rate;
    m.motion_index = r.motion_index;
    m.motion = r.motion ? 1 : 0;
    int nb = (int)r.breaths.size(); if (nb > FEAT_MAX_BREATH) nb = FEAT_MAX_BREATH;
    m.n_breath = (uint8_t)nb;
    for (int i = 0; i < nb; i++) { m.breath_bpm[i] = r.breaths[i].rate_bpm; m.breath_snr[i] = r.breaths[i].snr; }
    int nh = (int)r.hearts.size(); if (nh > FEAT_MAX_HEART) nh = FEAT_MAX_HEART;
    m.n_heart = (uint8_t)nh;
    for (int i = 0; i < nh; i++) { m.heart_bpm[i] = r.hearts[i].rate_bpm; m.heart_snr[i] = r.hearts[i].snr; }
    return m;
}

LinkResult link_from_feature(const MsgFeature& m) {
    LinkResult r;
    r.valid = m.valid != 0;
    r.rssi = m.rssi;
    r.pkt_rate = m.pkt_rate;
    r.motion_index = m.motion_index;
    r.motion = m.motion != 0;
    for (int i = 0; i < m.n_breath && i < FEAT_MAX_BREATH; i++)
        r.breaths.push_back({ m.breath_bpm[i], m.breath_snr[i] });
    for (int i = 0; i < m.n_heart && i < FEAT_MAX_HEART; i++)
        r.hearts.push_back({ m.heart_bpm[i], m.heart_snr[i] });
    return r;
}

// Which transmitters' CSI the scene used on the last tick, per receiving board.
static uint8_t      s_used[2][2 * MAX_LINKS][6];
static int          s_nused[2];
static portMUX_TYPE s_used_mux = portMUX_INITIALIZER_UNLOCKED;   // main task writes, HTTP task reads

void used_links_set(const UsedLink* u, int n) {
    portENTER_CRITICAL(&s_used_mux);
    s_nused[0] = s_nused[1] = 0;
    for (int i = 0; i < n; i++) {
        const int b = u[i].peer ? 1 : 0;
        if (s_nused[b] < 2 * MAX_LINKS) memcpy(s_used[b][s_nused[b]++], u[i].mac, 6);
    }
    portEXIT_CRITICAL(&s_used_mux);
}

bool used_link(const uint8_t* mac, bool peer) {
    bool hit = false;
    portENTER_CRITICAL(&s_used_mux);
    for (int i = 0; i < s_nused[peer ? 1 : 0] && !hit; i++) hit = !memcmp(s_used[peer ? 1 : 0][i], mac, 6);
    portEXIT_CRITICAL(&s_used_mux);
    return hit;
}

size_t rf_sources_msg(MsgRfSources& m) {
    static RfSource src[RF_SRC_MAX];   // main task only
    uint8_t links[MAX_LINKS][6];
    const int nl = sensing_link_macs(links, MAX_LINKS);
    const int n = radio_sources(src, RF_SRC_MAX);
    m.st = diag_board_status();
    m.n = (uint8_t)n;
    for (int i = 0; i < n; i++) {
        MsgRfSource& o = m.src[i];
        memcpy(o.mac, src[i].mac, 6);
        o.rssi_last = src[i].rssi_last;
        o.rssi_avg = src[i].rssi_avg;
        o.rate_x10 = src[i].rate_x10;
        o.flags = src[i].beacon ? 4 : 0;
        for (int k = 0; k < nl; k++) if (!memcmp(links[k], src[i].mac, 6)) o.flags |= 1;
    }
    return offsetof(MsgRfSources, src) + n * sizeof(MsgRfSource);
}

// Re-sent every 2 s, so the follower runs the configured default even if it was
// told something else by a previous leader instance (reflash, reboot).
static MsgIllumSet  s_peer_illum = {ILLUMINATE_RATE_HZ, TX_POWER_DBM};
static portMUX_TYPE s_illum_mux = portMUX_INITIALIZER_UNLOCKED;   // HTTP task writes, main task reads

MsgIllumSet peer_illum_desired() {
    portENTER_CRITICAL(&s_illum_mux);
    const MsgIllumSet m = s_peer_illum;
    portEXIT_CRITICAL(&s_illum_mux);
    return m;
}

void peer_illum_request(uint16_t rate_hz, int8_t tx_dbm) {
    portENTER_CRITICAL(&s_illum_mux);
    s_peer_illum.rate_hz = rate_hz > ILLUMINATE_MAX_HZ ? ILLUMINATE_MAX_HZ : rate_hz;
    s_peer_illum.tx_dbm = tx_dbm;
    portEXIT_CRITICAL(&s_illum_mux);
    peer_illum_resend();
}

void peer_illum_resend() {
    const MsgIllumSet m = peer_illum_desired();
    link_send(MsgType::ILLUM_SET, &m, sizeof(m));
}

void illum_apply(const MsgIllumSet& m) {
    radio_set_illuminator_rate(m.rate_hz);
    if (m.tx_dbm != -128 && radio_tx_power_q() != m.tx_dbm * 4) radio_set_tx_power_dbm(m.tx_dbm);
}

uint8_t choose_channel(const uint8_t* chans, const uint16_t* counts, int n) {
    int best = 0;
    for (int i = 1; i < n; i++) if (counts[i] < counts[best]) best = i;
    return chans[best];
}

uint8_t ht40_second_for(uint8_t primary) {
#if SENSE_USE_HT40
    (void)primary; return (uint8_t)WIFI_SECOND_CHAN_BELOW;
#else
    (void)primary; return (uint8_t)WIFI_SECOND_CHAN_NONE;
#endif
}
