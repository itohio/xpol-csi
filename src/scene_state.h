#pragma once
// Thread-safe holder for the latest web scene, shared with the HTTP/WS server.
#include "web_scene.h"
#include "config.h"

void scene_state_init();
// The main loop fills the scene in place, holding the scene mutex between begin
// and end (one copy of the scene exists; serializers wait for end).
WebScene& scene_state_begin();
void scene_state_end();
// Used by app (no vital tracker): scene from the fused hypotheses, one per peak.
void scene_state_set(const SceneResult& s, Mode mode, uint8_t channel);

// WebSocket messages (packed little-endian, no heap; layouts in scene_state.cpp,
// decoded by web/src/wire.js). Return the encoded length.
int scene_state_scene(uint8_t* buf, int cap);     // SCENE 0xC5: presence, tracks, links
int scene_state_spectra(uint8_t* buf, int cap);   // SPECTRA 0xC9: spectra + one waveform point
