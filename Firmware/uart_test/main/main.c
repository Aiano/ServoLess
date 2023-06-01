#include <stdio.h>
#include <string.h>
#include "driver/uart.h"

void app_main(void)
{
    const uart_port_t uart_num = UART_NUM_2;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 17, 18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size,
                                        uart_buffer_size, 10, &uart_queue, 0));

    while (1)
    {
        // Write data to UART.
        char *test_str = "This is a test string.\n";
        uart_write_bytes(uart_num, (const char *)test_str, strlen(test_str));

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
