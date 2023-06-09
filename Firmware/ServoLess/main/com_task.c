#include "com_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ESP_LOG.h"
#include "current_sense.h"
#include "FOC.h"
#include "FOC_task.h"
#include "driver/twai.h"
#include "driver/uart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

const static char *TAG = "communication";

#define CAN_TX_GPIO GPIO_NUM_36
#define CAN_RX_GPIO GPIO_NUM_37

static void init_can()
{
    // Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
    {
        printf("Driver installed\n");
    }
    else
    {
        printf("Failed to install driver\n");
        return;
    }

    // Start TWAI driver
    if (twai_start() == ESP_OK)
    {
        printf("Driver started\n");
    }
    else
    {
        printf("Failed to start driver\n");
        return;
    }
}

static void can_transmit()
{
    // Configure message to transmit
    twai_message_t message;
    message.identifier = 0x1FF;
    message.extd = 0;
    message.data_length_code = 8;
    message.ss = 1;
    message.rtr = 0;
    message.dlc_non_comp = 0;
    for (int i = 0; i < 8; i++)
    {
        message.data[i] = 0;
    }

    // Queue message for transmission
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK)
    {
        printf("Message queued for transmission\n");
    }
    else
    {
        printf("Failed to queue message for transmission\n");
    }
}

static void can_receive()
{
}

static void print_curr()
{
    printf("%.2f,%.3f,%.2f,%.3f\n", target_Iq, output_Iq, 0.0, output_Id);
}

static void print_pos(){
    printf("%.3f,%.3f\n", target_angle, output_mechanical_angle);
}

static void print_loop_period()
{
    printf("%lld\n", loop_timer_count);
}

static void print_all_info()
{
    printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", output_mechanical_angle, output_electrical_angle, output_target_velocity, output_velocity, output_target_Iq, output_Iq, 0.0, output_Id);
}

static QueueHandle_t uart_queue;
static const uart_port_t uart_num = UART_NUM_1;
// Setup UART buffered IO with event queue
const int uart_buffer_size = (1024 * 2);
char rx_buf[1024];
char tx_buf[1024];
float rx_num;

static void uartEventTask(void *pvParameter)
{
    uart_event_t event;
    while(1){
        if(xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)){
            switch (event.type)
            {
            case UART_DATA:
                uart_read_bytes(uart_num, rx_buf, event.size, portMAX_DELAY);
                rx_num = atof(rx_buf);
                target_angle = rx_num;
                break;
            
            default:
                break;
            }
        }
    }
}

static void uart_init()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, 17, 18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, uart_buffer_size,
                                        uart_buffer_size, 10, &uart_queue, 0));

    xTaskCreatePinnedToCore(uartEventTask, "uart_event_task", 2048, NULL, 2, NULL, 1);
}

// char test_buf[] = "Hello, world!\n";
char tx_buffer[512];
static void uart_print_all_info(){
    sprintf(tx_buffer, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", output_mechanical_angle, output_electrical_angle, output_target_velocity, output_velocity, output_target_Iq, output_Iq, 0.0, output_Id);
    uart_write_bytes(UART_NUM_1, tx_buffer, strlen(tx_buffer));
}

void comTask(void *pvParameter)
{
    uart_init();
    while (1)
    {
        print_all_info();
        uart_print_all_info();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}