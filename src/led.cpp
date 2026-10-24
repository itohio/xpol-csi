#include "led.h"
#include "config.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// XIAO ESP32-C5 USER LED is active-LOW: drive 0 to light it.
static inline void led_write(bool on) { gpio_set_level((gpio_num_t)PIN_USER_LED, on ? 0 : 1); }

static volatile LedMode s_mode = LedMode::OFF;
static volatile float   s_bpm = 15.0f;

static void led_task(void*) {
    const uint32_t step = 20;  // ms
    uint32_t t = 0;
    for (;;) {
        bool on = false;
        switch (s_mode) {
        case LedMode::OFF:          on = false; break;
        case LedMode::SOLID:        on = true;  break;
        case LedMode::BLINK_FAST:   on = (t % 240) < 120; break;
        case LedMode::BLINK_SLOW:   on = (t % 1000) < 500; break;
        case LedMode::DOUBLE_BLINK: {
            uint32_t p = t % 1200;
            on = (p < 100) || (p >= 200 && p < 300);
            break;
        }
        case LedMode::PULSE: {
            float bpm = s_bpm > 1.0f ? s_bpm : 15.0f;
            uint32_t period = (uint32_t)(60000.0f / bpm);
            if (period < 300) period = 300;
            on = (t % period) < 120;  // short flash per breath
            break;
        }
        }
        led_write(on);
        t += step;
        vTaskDelay(pdMS_TO_TICKS(step));
    }
}

void led_init() {
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << PIN_USER_LED;
    io.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io);
    led_write(false);
    xTaskCreate(led_task, "led", 2048, nullptr, 2, nullptr);
}

void led_set(LedMode m) { s_mode = m; }
void led_set_bpm(float bpm) { s_bpm = bpm; }
