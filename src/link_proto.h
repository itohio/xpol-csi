#pragma once
// Framed UART link between node_a and node_b.
// Frame: 0x7E | len(2 LE) | type(1) | payload[len] | crc16(2 LE)
// crc16 covers type+payload (CCITT, init 0xFFFF).

#include <stdint.h>
#include <stddef.h>
#include <functional>

enum class MsgType : uint8_t {
    HELLO          = 1,  // node announces itself (election)
    SURVEY_RESULT  = 2,  // per-channel frame counts
    CHANNEL_AGREE  = 3,  // leader's chosen sensing channel + mode
    MODE_SET       = 4,  // change operating mode
    FEATURE_REPORT = 5,  // one node's per-link sensing result
    HEARTBEAT      = 6,  // leader liveness beacon
    RF_SOURCES     = 7,  // what a node's radio hears (diagnostics, every 2 s)
    ILLUM_SET      = 8,  // leader -> follower: illumination rate + TX power (repeated every 2 s)
};

#pragma pack(push, 1)
struct MsgHello {
    uint8_t role;        // 'A' or 'B'
    uint8_t mac[6];
};

#define SURVEY_MAX_CH 16
struct MsgSurvey {
    uint8_t  n;
    uint8_t  channel[SURVEY_MAX_CH];
    uint16_t count[SURVEY_MAX_CH];
};

struct MsgChannelAgree {
    uint8_t  channel;
    uint8_t  second;     // wifi_second_chan_t value
    uint8_t  band_5g;
    uint8_t  mode;       // Mode
};

struct MsgModeSet {
    uint8_t mode;        // Mode
};

#define FEAT_MAX_BREATH 4
#define FEAT_MAX_HEART  4
struct MsgFeature {
    uint8_t  source;        // reserved (self vs peer); fusion keys on link_id
    uint8_t  link_id;       // link slot on the reporting node
    uint8_t  mac[6];        // the link's transmitter (as the reporting node hears it)
    uint8_t  fmt;           // its PHY format id (radio.h)
    uint8_t  valid;
    int8_t   rssi;          // EMA, dBm
    float    pkt_rate;      // CSI frames/s
    float    motion_index;
    uint8_t  motion;
    uint8_t  n_breath;
    float    breath_bpm[FEAT_MAX_BREATH];
    float    breath_snr[FEAT_MAX_BREATH];
    uint8_t  n_heart;
    float    heart_bpm[FEAT_MAX_HEART];
    float    heart_snr[FEAT_MAX_HEART];
};

// Every transmitter the sending node's radio hears (its RF source table).
#define RF_SRC_MAX 16
struct MsgRfSource {
    uint8_t  mac[6];
    int8_t   rssi_last;
    int8_t   rssi_avg;
    uint16_t rate_x10;      // frames per second x10
    uint8_t  flags;         // bit0: a sensing link on the sender, bit2: beacons (AP)
};
// One board's load and radio state (status bar); the follower sends its own
// inside RF_SOURCES every 2 s.
struct BoardStatus {
    uint32_t heap_free;     // bytes now
    uint32_t heap_min;      // lowest free heap over the last 2 s interval
    uint8_t  cpu_busy;      // % of the last 2 s not in IDLE
    int8_t   tx_q;          // configured max TX power, 0.25 dBm units
    uint16_t illum_hz;      // ESP-NOW illumination rate (0 = off)
    uint16_t csi_fps;       // CSI frames/s into the ring (all transmitters)
    uint16_t csi_restarts;  // frozen-CSI recoveries since boot
    uint8_t  frozen;        // a link repeated identical CSI within the last 3 s
};
struct MsgRfSources {
    BoardStatus st;
    uint8_t     n;
    MsgRfSource src[RF_SRC_MAX];   // only n are sent
};

// Desired illumination of the receiving node. Re-sent every 2 s, so a node
// that reboots picks it up again.
struct MsgIllumSet {
    uint16_t rate_hz;       // 0 = off
    int8_t   tx_dbm;        // max TX power; -128 = leave unchanged
};
#pragma pack(pop)

uint16_t crc16_ccitt(const uint8_t* data, size_t len);

// Build a frame into out (must hold payload_len + 6). Returns total bytes.
size_t link_frame(uint8_t* out, MsgType type, const void* payload, uint16_t payload_len);

// Incremental parser. Call feed() with incoming bytes; on a complete, valid
// frame it invokes the handler with the type and payload.
class LinkParser {
public:
    using Handler = std::function<void(MsgType, const uint8_t*, uint16_t)>;
    explicit LinkParser(Handler h) : handler_(h) {}
    void feed(const uint8_t* data, size_t len);

private:
    enum class St { SYNC, LEN0, LEN1, BODY } st_ = St::SYNC;
    Handler  handler_;
    uint16_t need_ = 0, got_ = 0;
    uint8_t  buf_[600];
};
