#include "link.h"
#include "config.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static SemaphoreHandle_t s_tx_mtx;

static LinkParser* s_parser = nullptr;

static void link_rx_task(void*) {
    uint8_t buf[256];
    for (;;) {
        int n = uart_read_bytes((uart_port_t)LINK_UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(20));
        if (n > 0 && s_parser) s_parser->feed(buf, n);
    }
}

void link_init(LinkParser::Handler handler) {
    s_tx_mtx = xSemaphoreCreateMutex();
    static LinkParser parser(handler);
    s_parser = &parser;

    uart_config_t cfg = {};
    cfg.baud_rate = LINK_UART_BAUD;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    uart_driver_install((uart_port_t)LINK_UART_PORT, 2048, 2048, 0, nullptr, 0);
    uart_param_config((uart_port_t)LINK_UART_PORT, &cfg);
    uart_set_pin((uart_port_t)LINK_UART_PORT, PIN_LINK_UART_TX, PIN_LINK_UART_RX,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    xTaskCreate(link_rx_task, "link_rx", 4096, nullptr, 9, nullptr);
}

void link_send(MsgType type, const void* payload, uint16_t len) {
    uint8_t frame[600];
    if (len + 6u > sizeof(frame)) return;
    size_t total = link_frame(frame, type, payload, len);
    xSemaphoreTake(s_tx_mtx, portMAX_DELAY);
    uart_write_bytes((uart_port_t)LINK_UART_PORT, (const char*)frame, total);
    xSemaphoreGive(s_tx_mtx);
}
