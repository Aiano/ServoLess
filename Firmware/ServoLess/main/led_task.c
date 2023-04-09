#include "led_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "esp_log.h"

static const char *TAG = "led task";

static void init_gpio();

void ledTask(void *pvParameter){
    ESP_LOGI(TAG, "Init GPIO.");
    init_gpio();

    uint32_t gpio_level = 0;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));

        gpio_level = !gpio_level;
        gpio_set_level(BLINK_GPIO, gpio_level);
    }
}

static void init_gpio()
{
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}