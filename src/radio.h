#pragma once
// Wi-Fi bring-up, CSI capture, ESP-NOW illumination and channel survey for C5.

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "esp_wifi_types.h"
#include "dsp.h"

// CSI ring item: this header, then 2*n_sub raw interleaved I/Q bytes in place.
// The Wi-Fi callback copies the driver buffer once, straight into the ring
// (xRingbufferSendAcquire); the consumer reads it in place (no queue copies).
struct RadioCsi {
    uint8_t  mac[6];
    int8_t   rssi;
    int8_t   noise;
    uint32_t t_us;
    uint8_t  agc_gain;     // receiver AGC gain index (0 without gain control)
    int8_t   fft_gain;     // receiver FFT gain index
    uint8_t  n_sub;        // valid subcarriers = raw_len/2
    uint8_t  fmt;          // PHY format id: (cur_bb_format << 4) | MCS (HT) or L-SIG rate code (11g)
    int8_t*       iq()       { return reinterpret_cast<int8_t*>(this + 1); }
    const int8_t* iq() const { return reinterpret_cast<const int8_t*>(this + 1); }
};
static_assert(sizeof(RadioCsi) % 4 == 0, "I/Q must follow the header without padding");

// Amplitude compensation factor for a frame received at (agc, fft) gain
// indices, relative to an arbitrary fixed baseline (soft-float: call only when
// a link's indices change). Ratios between two calls cancel the baseline, so a
// per-link reference keeps the factor near 1. Returns 1 without gain control.
float radio_gain_compensation(uint8_t agc, int8_t fft);

// PHY format id of a received frame (RadioCsi::fmt) and a short label for it
// ("ht0", "11g", "11b", "he", ...), for the web and logs.
uint8_t radio_csi_format(const wifi_pkt_rx_ctrl_t* rx);
const char* radio_format_label(uint8_t fmt, char out[8]);

// Bring up Wi-Fi in STA mode. The leader later calls radio_enable_ap().
void radio_wifi_start(bool apsta);

// Tune the radio to the sensing channel. second is a wifi_second_chan_t value.
void radio_set_sensing_channel(uint8_t primary, uint8_t second, bool band_5g);

// Leader brings up the SoftAP (AP+STA) on the given channel, served at 10.0.0.1.
void radio_enable_ap(const char* ssid, const char* pass, uint8_t channel);

// DEBUG ONLY: join an external AP on the STA interface so the board is reachable
// over the LAN for development. Blocks until DHCP (or ~10 s timeout). On success
// writes the connected primary channel and dotted IP. NOTE: one radio -> the
// SoftAP + CSI sensing are pinned to this AP's channel while connected.
bool radio_sta_connect(const char* ssid, const char* pass, uint8_t* out_ch, char* out_ip, int ip_len);

// Illumination: ESP-NOW broadcast frames at rate_hz from the factory MAC.
// ESP-NOW is brought up on the first nonzero rate only (0 Hz leaves it down).
void radio_start_illuminator(int rate_hz);
// Change the illumination rate at runtime (0 = off, clamped to ILLUMINATE_MAX_HZ).
void radio_set_illuminator_rate(int rate_hz);
int  radio_illuminator_rate();

// Set the maximum TX power (all frames, incl. ESP-NOW illumination), clamped to
// the driver's 2..21 dBm. Returns the applied value in 0.25 dBm units. Lowering
// it keeps a close receiver out of saturation; it does not change the ratio of
// body-reflected to direct signal.
int radio_set_tx_power_dbm(int dbm);

// Every transmitter the radio hears on the current channel (all frame types,
// FCS-failed frames excluded), kept by the promiscuous RX callback.
struct RfSource {
    uint8_t  mac[6];
    int8_t   rssi_last;
    int8_t   rssi_avg;     // EMA, 1/8 per frame
    uint16_t rate_x10;     // frames per second x10, last full second
    bool     beacon;       // sends beacons: an AP
    uint8_t  ssid_len;
    char     ssid[32];
};
// The up-to-max strongest sources heard in the last 10 s with more than one
// frame (or beaconing), sorted strongest first.
int radio_sources(RfSource* out, int max);
// Configured max TX power, 0.25 dBm units.
int radio_tx_power_q();

// Enable CSI capture into rb (RINGBUF_TYPE_NOSPLIT). Frames are dropped, never
// waited on, when the ring is full: the Wi-Fi task must not block.
void radio_start_csi(RingbufHandle_t rb);
// Frozen CSI engine (identical frames): restart capture, three times within
// 10 s, then at most every 30 s. Nothing short of a power cut has cleared the
// latch, so the board keeps running and reports it (README, "Frozen CSI").
bool radio_recover_csi();
unsigned radio_reset_reason();   // ROM reset reason code (soc/reset_reasons.h), 0x01 = power-on
uint32_t radio_csi_restarts();
// Frames dropped because the CSI ring was full / frames written into it /
// frames without a valid channel estimate (cumulative).
uint32_t radio_csi_drops();
uint32_t radio_csi_written();
uint32_t radio_csi_stale();

// Blocking channel survey: dwell across chans[], count received frames per
// channel, write counts[]. Restores nothing; caller sets the sensing channel after.
void radio_survey(const uint8_t* chans, int n, int total_ms, uint16_t* counts);
