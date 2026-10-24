#pragma once
// System and RF diagnostics for the web radio tab and the USB health lines.
// diag_update() (main task, every 2 s) is the only writer of the system stats;
// diag_set_peer_sources() (UART task) records what the peer board hears.

#include <stdint.h>
#include <stddef.h>
#include "sensing.h"
#include "link_proto.h"

struct DiagTask { char name[16]; uint16_t permille; };   // CPU share since the previous update

struct DiagStats {
    uint16_t      loop_ms = 0;        // main-loop work per tick
    uint32_t      heap_free = 0;      // now
    uint32_t      heap_min_int = 0;   // lowest over the last update interval (2 s)
    uint32_t      heap_min = 0;       // lowest since boot
    uint8_t       cpu_busy = 0;       // % not IDLE over the last interval
    uint16_t      csi_fps = 0;        // CSI frames/s into the ring
    SensingHealth csi{};
    uint8_t       ntask = 0;
    DiagTask      task[12];           // sorted by share, IDLE included
};

void diag_update(int loop_ms);         // main task, every 2 s; also logs "# health" / "# cpu"
DiagStats diag_get();
BoardStatus diag_board_status();       // this board, as of the last diag_update

// The peer's RF source table and status from an RF_SOURCES message of len bytes.
void diag_set_peer_sources(const MsgRfSources& m, size_t len);
int  diag_peer_sources(MsgRfSource* out, int max);   // empty if older than 5 s
bool diag_peer_status(BoardStatus* out);              // false if older than 5 s
