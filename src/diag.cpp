#include "diag.h"
#include "telemetry.h"
#include "config.h"
#include "radio.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <algorithm>

namespace {
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
DiagStats    s_stats;

MsgRfSources s_peer_src{};
int64_t      s_peer_ts = 0;
BoardStatus  s_self{};

// Per-task CPU share since the previous call (single core: shares incl. IDLE sum to ~1000).
int cpu_shares(DiagTask* out, int max) {
    constexpr int kMax = 24;
    static TaskStatus_t st[kMax];
    static TaskHandle_t prev_h[kMax];
    static uint32_t prev_rt[kMax], prev_total;
    static int prev_n;
    uint32_t total = 0;
    const int n = (int)uxTaskGetSystemState(st, kMax, &total);
    const uint32_t dt = total - prev_total;
    DiagTask sh[kMax];
    for (int i = 0; i < n; i++) {
        uint32_t before = 0;
        for (int k = 0; k < prev_n; k++) if (prev_h[k] == st[i].xHandle) { before = prev_rt[k]; break; }
        strlcpy(sh[i].name, st[i].pcTaskName, sizeof(sh[i].name));
        sh[i].permille = dt ? (uint16_t)((uint64_t)(st[i].ulRunTimeCounter - before) * 1000 / dt) : 0;
    }
    for (int i = 0; i < n; i++) { prev_h[i] = st[i].xHandle; prev_rt[i] = st[i].ulRunTimeCounter; }
    prev_n = n; prev_total = total;
    if (!dt) return 0;
    std::sort(sh, sh + n, [](const DiagTask& a, const DiagTask& b) { return a.permille > b.permille; });
    const int m = std::min(n, max);
    memcpy(out, sh, sizeof(DiagTask) * m);
    return m;
}
}  // namespace

void diag_update(int loop_ms) {
    static int64_t  last_us = 0;
    static uint32_t last_written = 0;
    static bool     monitoring = false;
    const int64_t now = esp_timer_get_time();

    DiagStats s;
    s.loop_ms = (uint16_t)loop_ms;
    s.heap_free = esp_get_free_heap_size();
    // Lowest free heap since the previous update (2 s): the dips that matter
    // (the Wi-Fi driver starving) last milliseconds, so a sample of "free now"
    // misses them and the lifetime minimum never recovers.
    if (monitoring) {
        s.heap_min_int = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
        heap_caps_monitor_local_minimum_free_size_stop();
    } else {
        s.heap_min_int = s.heap_free;
    }
    s.heap_min = esp_get_minimum_free_heap_size();   // lifetime (monitor stopped)
    monitoring = heap_caps_monitor_local_minimum_free_size_start() == ESP_OK;
    s.csi = sensing_health();
    s.csi_fps = last_us ? (uint16_t)((uint64_t)(s.csi.written - last_written) * 1000000 / (uint64_t)(now - last_us)) : 0;
    last_us = now; last_written = s.csi.written;
    s.ntask = (uint8_t)cpu_shares(s.task, sizeof(s.task) / sizeof(s.task[0]));
    uint16_t idle = 0;
    for (int i = 0; i < s.ntask; i++) if (!strncmp(s.task[i].name, "IDLE", 4)) idle += s.task[i].permille;
    s.cpu_busy = (uint8_t)std::min(100, (1000 - std::min<int>(idle, 1000) + 5) / 10);

    BoardStatus b{};
    b.heap_free = s.heap_free; b.heap_min = s.heap_min_int; b.cpu_busy = s.cpu_busy;
    b.tx_q = (int8_t)radio_tx_power_q(); b.illum_hz = (uint16_t)radio_illuminator_rate();
    b.csi_fps = s.csi_fps; b.csi_restarts = (uint16_t)radio_csi_restarts(); b.frozen = sensing_csi_frozen() ? 1 : 0;
    portENTER_CRITICAL(&s_mux);
    s_stats = s;
    s_self = b;
    portEXIT_CRITICAL(&s_mux);

    telem_comment("health loop=%dms cpu=%u%% heap=%u min2s=%u min=%u csi=%u/s csi_drops=%u csi_stale=%u csi_restarts=%u csi_backlog_max=%u ring=%u/%u peak=%u",
                  loop_ms, (unsigned)s.cpu_busy, (unsigned)s.heap_free, (unsigned)s.heap_min_int, (unsigned)s.heap_min,
                  (unsigned)s.csi_fps, (unsigned)s.csi.drops, (unsigned)radio_csi_stale(), (unsigned)radio_csi_restarts(),
                  (unsigned)s.csi.max_backlog, (unsigned)s.csi.ring_used, (unsigned)s.csi.ring_size, (unsigned)s.csi.ring_peak);
    char line[220];
    int len = snprintf(line, sizeof(line), "cpu");
    for (int i = 0; i < s.ntask && i < 8 && len < (int)sizeof(line) - 24; i++)
        len += snprintf(line + len, sizeof(line) - len, " %s=%u.%u%%", s.task[i].name,
                        (unsigned)(s.task[i].permille / 10), (unsigned)(s.task[i].permille % 10));
    telem_comment("%s", line);
}

DiagStats diag_get() {
    portENTER_CRITICAL(&s_mux);
    const DiagStats s = s_stats;
    portEXIT_CRITICAL(&s_mux);
    return s;
}

BoardStatus diag_board_status() {
    portENTER_CRITICAL(&s_mux);
    const BoardStatus b = s_self;
    portEXIT_CRITICAL(&s_mux);
    return b;
}

void diag_set_peer_sources(const MsgRfSources& m, size_t len) {
    const size_t hdr = offsetof(MsgRfSources, src);
    if (len < hdr) return;
    const int n = std::min<int>({m.n, RF_SRC_MAX, (int)((len - hdr) / sizeof(MsgRfSource))});
    portENTER_CRITICAL(&s_mux);
    s_peer_src.st = m.st;
    s_peer_src.n = (uint8_t)n;
    memcpy(s_peer_src.src, m.src, n * sizeof(MsgRfSource));
    s_peer_ts = esp_timer_get_time();
    portEXIT_CRITICAL(&s_mux);
}

static bool peer_fresh(int64_t now) { return s_peer_ts && now - s_peer_ts < 5000000; }

int diag_peer_sources(MsgRfSource* out, int max) {
    const int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_mux);
    const int n = peer_fresh(now) ? std::min<int>(s_peer_src.n, max) : 0;
    memcpy(out, s_peer_src.src, n * sizeof(MsgRfSource));
    portEXIT_CRITICAL(&s_mux);
    return n;
}

bool diag_peer_status(BoardStatus* out) {
    const int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_mux);
    const bool ok = peer_fresh(now);
    if (ok) *out = s_peer_src.st;
    portEXIT_CRITICAL(&s_mux);
    return ok;
}
