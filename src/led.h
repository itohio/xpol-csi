#pragma once
// USER LED (GPIO27, single-color yellow, active-LOW on the XIAO ESP32-C5).
// No RGB on this board, so phases are distinguished by blink pattern:
//   BLINK_FAST   election
//   DOUBLE_BLINK CSI background survey / channel selection
//   BLINK_SLOW   calibration
//   SOLID        role selected / running, no breathing yet
//   PULSE        running: one flash per breath at the strongest breathing rate

enum class LedMode { OFF, BLINK_FAST, BLINK_SLOW, DOUBLE_BLINK, SOLID, PULSE };

void led_init();
void led_set(LedMode m);
void led_set_bpm(float bpm);   // pulse rate for PULSE mode
