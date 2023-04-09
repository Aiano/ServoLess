#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "FOC_task.h"
#include "led_task.h"
#include "com_task.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "Creating FreeRTOS tasks.");

    xTaskCreatePinnedToCore(FOCTask, "FOC", 4096, NULL, 0, NULL, 0);
    xTaskCreatePinnedToCore(ledTask, "led", 4096, NULL, 0, NULL, 1);
    xTaskCreatePinnedToCore(comTask, "com", 4096, NULL, 1, NULL, 1);

    ESP_LOGI(TAG, "Tasks created.");
}
