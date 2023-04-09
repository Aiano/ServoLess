#include "FOC_task.h"
#include "FOC.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp32_mcu.h"
#include "AS5048A.h"


static const char *TAG = "FOC task";

float target_Iq = 0.15;
float target_velocity = 5;
float target_angle = _PI;

void FOCTask(void *pvParameter)
{
    FOC_init();

    while (1)
    {
        // FOC_open_loop_voltage_control_loop(2);
        // FOC_current_control_loop(target_Iq); // 电流环不能加延时，循环频率最高, 测试值：7.2kHz
        // FOC_velocity_control_loop(target_velocity); // 其他环需要加1ms延时
        FOC_position_control_loop(target_angle);
        // 
    }
}