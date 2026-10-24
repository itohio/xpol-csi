// STEP 1 — minimal bring-up: LED blink + UART link + USB telemetry only.
// No Wi-Fi, no NVS, no CSI, no web. Proves the board boots, the LED works, the
// USB console works, and the two boards talk over the D6/D7 UART cross-link.
//
// Build/flash: pio run -e step1 -t upload --upload-port <port>
// Each board prints over USB:
//   # STEP1 ... self=<mac>
//   ALIVE self=<mac> seq=N led=0/1          (every 500 ms)
//   RX self=<mac> <= PING <peermac> <seq>   (when the peer's UART frame arrives)
// Cross-wire D6(TX)->D7(RX) both ways + GND to see RX lines on both boards.

#include "config.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_random.h"
#include <stdio.h>
#include <string.h>

void step1_main() {
    setvbuf(stdout, nullptr, _IONBF, 0);

    // LED
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << PIN_USER_LED;
    io.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io);

    // UART link on D6/D7
    uart_config_t uc = {};
    uc.baud_rate = LINK_UART_BAUD;
    uc.data_bits = UART_DATA_8_BITS;
    uc.parity = UART_PARITY_DISABLE;
    uc.stop_bits = UART_STOP_BITS_1;
    uc.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uc.source_clk = UART_SCLK_DEFAULT;
    uart_driver_install((uart_port_t)LINK_UART_PORT, 1024, 1024, 0, nullptr, 0);
    uart_param_config((uart_port_t)LINK_UART_PORT, &uc);
    uart_set_pin((uart_port_t)LINK_UART_PORT, PIN_LINK_UART_TX, PIN_LINK_UART_RX,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);       // unique per chip (efuse default is not, on C5)
    uint8_t macdef[6] = {0};
    esp_efuse_mac_get_default(macdef);
    uint16_t nonce = (uint16_t)(esp_random() & 0xFFFF);  // unique per boot, to spot loopback
    char id[13];
    snprintf(id, sizeof(id), "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    printf("# STEP1 led+uart self=%s nonce=%04x macdef=%02x%02x%02x%02x%02x%02x tx=D6(gpio%d) rx=D7(gpio%d) baud=%d\n",
           id, nonce, macdef[0],macdef[1],macdef[2],macdef[3],macdef[4],macdef[5],
           PIN_LINK_UART_TX, PIN_LINK_UART_RX, LINK_UART_BAUD);
    uint32_t seq = 0;
    bool on = false;
    uint8_t rx[256];
    for (;;) {
        on = !on;
        // active-LOW LED; toggling is visible regardless of polarity
        gpio_set_level((gpio_num_t)PIN_USER_LED, on ? 0 : 1);

        char msg[64];
        int n = snprintf(msg, sizeof(msg), "PING %s n=%04x %lu\n", id, nonce, (unsigned long)seq);
        uart_write_bytes((uart_port_t)LINK_UART_PORT, msg, n);

        int r = uart_read_bytes((uart_port_t)LINK_UART_PORT, rx, sizeof(rx) - 1, pdMS_TO_TICKS(50));
        if (r > 0) {
            rx[r] = 0;
            printf("RX self=%s <= %s", id, (char*)rx);
            if (rx[r - 1] != '\n') printf("\n");
        }

        printf("ALIVE self=%s seq=%lu led=%d\n", id, (unsigned long)seq, on ? 1 : 0);
        seq++;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
