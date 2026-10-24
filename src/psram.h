#pragma once
// Bulk state in the external PSRAM (8 MB on the XIAO ESP32-C5). The build adds
// PSRAM to the capability allocator only (CONFIG_SPIRAM_USE_CAPS_ALLOC): malloc,
// task stacks, the Wi-Fi driver and the CSI ring stay in internal SRAM, so only
// what is allocated here moves out. Allocate once at init, before the tasks that
// use it start; a board without PSRAM stops here with a log line.
#include <new>
#include <stddef.h>
#include <stdlib.h>
#include "esp_heap_caps.h"
#include "esp_log.h"

template <class T>
T* psram_new_array(size_t n, const char* what) {
    void* p = heap_caps_calloc(n, sizeof(T), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) {
        ESP_LOGE("psram", "no PSRAM for %s (%u B, free %u B)", what, (unsigned)(n * sizeof(T)),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        abort();
    }
    T* a = static_cast<T*>(p);
    for (size_t i = 0; i < n; i++) new (a + i) T();
    return a;
}

template <class T>
T* psram_new(const char* what) { return psram_new_array<T>(1, what); }
