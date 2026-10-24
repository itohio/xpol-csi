#pragma once
// Verbose USB telemetry. All lines are newline-terminated ASCII on the USB
// Serial/JTAG console (the USB-C port). The host tool (host/monitor.py) parses:
//
//   EVT,<t_ms>,<text>
//   ROLE,<t_ms>,<role>,<self_mac>,<peer_mac>,<peer_count>
//   CH,<t_ms>,<channel>,<second>,<band5g>
//   CSI,<t_ms>,<src_mac>,<rssi>,<noise>,<nsub>,<a0> <a1> ... <aN-1>
//   FEAT,<t_ms>,<src_mac>,<valid>,<motion>,<is_motion>,<cov>,<nb>,<b:snr;..>,<nh>,<h:snr;..>
//   SCENE,<t_ms>,<presence>,<motion>,<links>,<nb>,<b:snr;..>,<nh>,<h:snr;..>
//
// MACs are 12 lowercase hex digits (no colons). EVT/ROLE/CH/FEAT/SCENE always
// stream; CSI streams only when verbose is on (default in a -DVERBOSE_USB build,
// toggle at runtime by sending 'v').

#include <stdint.h>
#include "election.h"
#include "dsp.h"
#include "radio.h"
#include "config.h"

void telem_init();
// CSV comment line ('# ...'), used for the header and election narration.
void telem_comment(const char* fmt, ...);
// One-time header: schema + starting conditions.
void telem_header(const uint8_t self_mac[6], bool band_5g);

void telem_event(const char* fmt, ...);
void telem_role(Role r, const uint8_t self_mac[6], const uint8_t peer_mac[6], int peer_count);
void telem_channel(uint8_t channel, uint8_t second, bool band_5g);
void telem_csi(const uint8_t src_mac[6], int8_t noise, const CsiFrame& f);
void telem_iq(const uint8_t src_mac[6], int8_t noise, const CsiFrame& f);
void telem_feat(const uint8_t src_mac[6], const LinkResult& r);
void telem_scene(const SceneResult& s, uint8_t channel, Mode mode);

// Start the stdin command reader. 's' => on_status, 'c' => on_calibrate,
// 'v' => toggle CSI verbosity.
void telem_start_input(void (*on_status)(), void (*on_calibrate)());
