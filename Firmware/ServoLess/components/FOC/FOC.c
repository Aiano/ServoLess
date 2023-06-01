/**
 * @file FOC.c
 * @author Aiano_czm
 */

#include <math.h>
#include <stdio.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "FOC.h"
#include "FOC_conf.h"
#include "FOC_utils.h"
#include "FOC_PID.h"
#include "FOC_LPF.h"
#include "hardware_api.h"
#include "AS5048A.h"
#include "current_sense.h"
// #include "usart.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"

static const char *TAG = "FOC";

float output_target_Iq;
float output_Id, output_Iq;

float output_target_velocity;
float output_velocity;

float output_mechanical_angle, output_electrical_angle;

uint64_t loop_timer_count;

void FOC_init()
{
    cs_init();       // Init current sense
    _init3PWM();     // Init MCPWM
    _enableDriver(); // Enable DRV8313
    AS5048A_init();  // Init AS5048A related SPI

    FOC_electrical_angle_calibration();

    // FOC_test_pole_pairs();

    pid_set_parameters();
    FOC_lpf_set_parameters();
    
    FOC_init_vel_timer();
    FOC_init_loop_timer();
}

void FOC_electrical_angle_calibration()
{
    // FOC calibration
    FOC_SVPWM(0, 8, 0); // Uq and Ud are orthogonal, and normally we control Uq, so set Ud in calibration
    vTaskDelay(pdMS_TO_TICKS(500));

    // zero_electrical_angle = FOC_electrical_angle();
    AS5048A_set_zero_position();

    vTaskDelay(pdMS_TO_TICKS(100));
    FOC_SVPWM(0, 0, 0);
}

void FOC_SVPWM(float Uq, float Ud, float angle)
{

    if (Uq > VOLTAGE_LIMIT_2)
        Uq = VOLTAGE_LIMIT_2;
    if (Uq < -VOLTAGE_LIMIT_2)
        Uq = -VOLTAGE_LIMIT_2;
    if (Ud > VOLTAGE_LIMIT_2)
        Ud = VOLTAGE_LIMIT_2;
    if (Ud < -VOLTAGE_LIMIT_2)
        Ud = -VOLTAGE_LIMIT_2;

    int sector;

    // Nice video explaining the SpaceVectorModulation (FOC_SVPWM) algorithm
    // https://www.youtube.com/watch?v=QMSWUMEAejg

    // the algorithm goes
    // 1) Ualpha, Ubeta
    // 2) Uout = sqrt(Ualpha^2 + Ubeta^2)
    // 3) angle_el = atan2(Ubeta, Ualpha)
    //
    // equivalent to 2)  because the magnitude does not change is:
    // Uout = sqrt(Ud^2 + Uq^2)
    // equivalent to 3) is
    // angle_el = angle_el + atan2(Uq,Ud)

    float Uout = _sqrt(Ud * Ud + Uq * Uq) / VOLTAGE_LIMIT; // Actually, Uout is a ratio
    angle = _normalizeAngle(angle + atan2(Uq, Ud));

    // find the sector we are in currently
    sector = floor(angle / _PI_3) + 1;
    // calculate the duty cycles
    float T1 = _SQRT3 * _sin(sector * _PI_3 - angle) * Uout;
    float T2 = _SQRT3 * _sin(angle - (sector - 1.0f) * _PI_3) * Uout;
    // two versions possible
    //    float T0 = 0; // pulled to 0 - better for low power supply voltage
    float T0 = 1 - T1 - T2; // modulation_centered around driver->voltage_limit/2

    // calculate the duty cycles(times)
    float Ta, Tb, Tc;
    switch (sector)
    {
    case 1:
        Ta = T1 + T2 + T0 / 2;
        Tb = T2 + T0 / 2;
        Tc = T0 / 2;
        break;
    case 2:
        Ta = T1 + T0 / 2;
        Tb = T1 + T2 + T0 / 2;
        Tc = T0 / 2;
        break;
    case 3:
        Ta = T0 / 2;
        Tb = T1 + T2 + T0 / 2;
        Tc = T2 + T0 / 2;
        break;
    case 4:
        Ta = T0 / 2;
        Tb = T1 + T0 / 2;
        Tc = T1 + T2 + T0 / 2;
        break;
    case 5:
        Ta = T2 + T0 / 2;
        Tb = T0 / 2;
        Tc = T1 + T2 + T0 / 2;
        break;
    case 6:
        Ta = T1 + T2 + T0 / 2;
        Tb = T0 / 2;
        Tc = T1 + T0 / 2;
        break;
    default:
        // possible error state
        Ta = 0;
        Tb = 0;
        Tc = 0;
    }

    // printf("Uout:%.2f, sector:%d, Uq:%.2f, Ud:%.2f\n", Uout, sector, Uq, Ud);
    // printf("T1:%.2f, T2:%.2f, T0%.2f\n", T1, T2, T0);
    // // Ta, Tb, Tc range [0,1]
    // printf("Ta:%.2f, Tb:%.2f, Tc:%.2f\n", Ta, Tb, Tc);
    _writeDutyCycle3PWM(Ta, Tb, Tc);
}

void FOC_Clarke_Park(float Ia, float Ib, float Ic, float angle, float *Id, float *Iq)
{
    // Clarke transform
    float mid = (1.f / 3) * (Ia + Ib + Ic);
    float a = Ia - mid;
    float b = Ib - mid;
    float i_alpha = a;
    float i_beta = _1_SQRT3 * a + _2_SQRT3 * b;

    // Park transform
    float ct = _cos(angle);
    float st = _sin(angle);
    *Id = i_alpha * ct + i_beta * st;
    *Iq = i_beta * ct - i_alpha * st;

    return;
}

float FOC_get_mechanical_angle()
{
    output_mechanical_angle = (float)AS5048A_get_position() / SENSOR_VALUE_RANGE * _2PI;
    return output_mechanical_angle;
}

float FOC_electrical_angle()
{
    return _normalizeAngle(SENSOR_DIRECTION * POLE_PAIR * FOC_get_mechanical_angle());
}

uint8_t FOC_test_pole_pairs()
{
    FOC_electrical_angle_calibration(); // Calibrate first

    FOC_SVPWM(0, 2, 0); // Back to original position
    vTaskDelay(pdMS_TO_TICKS(300));
    for (float i = 0; i < 6.28; i += 0.01)
    {
        FOC_SVPWM(0, 2, i);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    AS5048A_get_position();
    float rotated_angle = FOC_get_mechanical_angle();
    float pole_pairs = _2PI / rotated_angle;
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Rotated angle: %.2f, Pole pairs: %.2f", rotated_angle, pole_pairs);

    return 0;
}

static gptimer_handle_t loop_timer = NULL;
void FOC_init_loop_timer()
{    
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &loop_timer));

    ESP_ERROR_CHECK(gptimer_enable(loop_timer));
    ESP_ERROR_CHECK(gptimer_start(loop_timer));

    ESP_LOGI(TAG, "Created and enabled loop timer.");
}

void FOC_loop_timer_spin()
{
    ESP_ERROR_CHECK(gptimer_get_raw_count(loop_timer, &loop_timer_count));
    ESP_ERROR_CHECK(gptimer_set_raw_count(loop_timer, 0)); // Clear counts
}

static gptimer_handle_t vel_timer = NULL;
void FOC_init_vel_timer()
{    
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &vel_timer));

    ESP_ERROR_CHECK(gptimer_enable(vel_timer));
    ESP_ERROR_CHECK(gptimer_start(vel_timer));

    ESP_LOGI(TAG, "Created and enabled velocity timer.");
}

uint64_t FOC_vel_timer_get_dt_us()
{
    static uint64_t count;
    ESP_ERROR_CHECK(gptimer_get_raw_count(vel_timer, &count));
    ESP_ERROR_CHECK(gptimer_set_raw_count(vel_timer, 0)); // Clear counts

    return count;
}

// rad/s
void FOC_get_velocity(float *velocity, float* electrical_angle, float *mechanical_angle)
{
    static float last_mechanical_angle = 0;

    // float ts = (float) (clock() - pre_tick) / CLOCKS_PER_SEC;
    // if (ts < 1e-3) ts = 1e-3;
    // pre_tick = clock();
    float dt = FOC_vel_timer_get_dt_us() / 1000000.0; // dt: s

    float now_mechanical_angle = FOC_get_mechanical_angle();
    float delta_angle = now_mechanical_angle - last_mechanical_angle;
    last_mechanical_angle = now_mechanical_angle;
    if (fabs(delta_angle) > _PI)
    {
        if (delta_angle > 0)
        {
            delta_angle -= _2PI;
        }
        else
        {
            delta_angle += _2PI;
        }
    }

    *velocity = delta_angle / dt;
    *mechanical_angle = now_mechanical_angle;
    *electrical_angle = _normalizeAngle(SENSOR_DIRECTION * POLE_PAIR * now_mechanical_angle);
}

void FOC_open_loop_voltage_control_loop(float Uq)
{
    float electrical_angle = FOC_electrical_angle();
    cs_read();
    FOC_SVPWM(Uq, 0, electrical_angle);

    vTaskDelay(pdMS_TO_TICKS(1));
}

// 7.2kHz
void FOC_current_control_loop(float target_Iq)
{
    static float electrical_angle;
    
    // ESP_ERROR_CHECK(gptimer_get_raw_count(loop_timer, &loop_timer_count));
    electrical_angle = FOC_electrical_angle(); // 106us
    // ESP_ERROR_CHECK(gptimer_set_raw_count(loop_timer, 0)); // Clear counts

    // Current sense
    static float Id, Iq;
    
    cs_read(); // 60us
    
    FOC_Clarke_Park(cs_current[0], cs_current[1], cs_current[2], electrical_angle, &Id, &Iq);
    Id = FOC_low_pass_filter(&lpf_current_d, Id);
    Iq = FOC_low_pass_filter(&lpf_current_q, Iq);

    static float Uq, Ud;
    // back feed
    
    Uq = pid_get_u(&pid_current_q, target_Iq, Iq);
    Ud = pid_get_u(&pid_current_d, 0, Id);

    FOC_SVPWM(Uq, Ud, electrical_angle);

    // Output some values
    output_Iq = Iq;
    output_Id = Id;
    
    // printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", cs_current[0], cs_current[1], cs_current[2], Iq, target_Iq, Id, 0.0, Uq, Ud);
    
}

void FOC_vel_curr_control_loop(float target_velocity) {
    static float electrical_angle;
    static float mechanical_angle;
    static float velocity;
    static float Id, Iq;
    static float target_Iq = 0;
    static float Uq, Ud;


    gptimer_get_raw_count(loop_timer, &loop_timer_count);
    if(loop_timer_count > 2000){ // Vel loop, 2ms 500Hz
        gptimer_set_raw_count(loop_timer, 0);

        FOC_get_velocity(&velocity, &electrical_angle, &mechanical_angle);
        velocity = FOC_low_pass_filter(&lpf_velocity, velocity);

        target_Iq = pid_get_u(&pid_curr_vel, target_velocity, velocity);
    }else{ // Current loop
        electrical_angle = FOC_electrical_angle();
        cs_read(); // 60us
        FOC_Clarke_Park(cs_current[0], cs_current[1], cs_current[2], electrical_angle, &Id, &Iq);
        Id = FOC_low_pass_filter(&lpf_current_d, Id);
        Iq = FOC_low_pass_filter(&lpf_current_q, Iq);
        // back feed
        Uq = pid_get_u(&pid_current_q, target_Iq, Iq);
        Ud = pid_get_u(&pid_current_d, 0, Id);
        
        FOC_SVPWM(Uq, Ud, electrical_angle);
    }

    // Output some infomation
    output_mechanical_angle = mechanical_angle;
    output_electrical_angle = electrical_angle;
    output_target_velocity = target_velocity;
    output_velocity = velocity;
    output_target_Iq = target_Iq;
    output_Iq = Iq;
    output_Id = Id;
    // printf("%.2f, %.2f, %.2f, %.3f, %.3f\n", velocity, electrical_angle, mechanical_angle, Iq, Id);
}

void FOC_pos_vel_curr_control_loop(float target_angle){
    static float electrical_angle;
    static float mechanical_angle;
    static float angle_error;
    static float target_velocity;
    static float velocity;
    static float Id, Iq;
    static float target_Iq = 0;
    static float Uq, Ud;


    gptimer_get_raw_count(loop_timer, &loop_timer_count);
    if(loop_timer_count > 1000){ // Vel loop, 2ms 500Hz
        gptimer_set_raw_count(loop_timer, 0);

        FOC_get_velocity(&velocity, &electrical_angle, &mechanical_angle);
        mechanical_angle = _normalizeAngle(mechanical_angle);

        angle_error = target_angle - mechanical_angle;
        if (angle_error < -_PI) target_angle += _2PI;
        else if (angle_error > _PI) target_angle -= _2PI;

        target_velocity = pid_get_u(&pid_curr_vel_pos, target_angle, mechanical_angle);

        velocity = FOC_low_pass_filter(&lpf_velocity, velocity);

        target_Iq = pid_get_u(&pid_curr_vel, target_velocity, velocity);
    }else{ // Current loop
        electrical_angle = FOC_electrical_angle();
        cs_read(); // 60us
        FOC_Clarke_Park(cs_current[0], cs_current[1], cs_current[2], electrical_angle, &Id, &Iq);
        Id = FOC_low_pass_filter(&lpf_current_d, Id);
        Iq = FOC_low_pass_filter(&lpf_current_q, Iq);
        // back feed
        Uq = pid_get_u(&pid_current_q, target_Iq, Iq);
        Ud = pid_get_u(&pid_current_d, 0, Id);
        
        FOC_SVPWM(Uq, Ud, electrical_angle);
    }

    // Output some infomation
    output_mechanical_angle = mechanical_angle;
    output_electrical_angle = electrical_angle;
    output_target_velocity = target_velocity;
    output_velocity = velocity;
    output_target_Iq = target_Iq;
    output_Iq = Iq;
    output_Id = Id;
    // printf("%.2f, %.2f, %.2f, %.3f, %.3f\n", velocity, electrical_angle, mechanical_angle, Iq, Id);
}

/**
 *
 * @param target_velocity unit: rad/s
 */
void FOC_velocity_control_loop(float target_velocity) {
    static float now_velocity;
    static float electrical_angle, mechanical_angle;
    FOC_get_velocity(&now_velocity, &electrical_angle, &mechanical_angle);

    now_velocity = FOC_low_pass_filter(&lpf_velocity, now_velocity);
    float Uq = pid_get_u(&pid_velocity, target_velocity, now_velocity);
    FOC_SVPWM(Uq, 0, electrical_angle);

    // Output some values
    output_velocity = now_velocity;
    // printf("%.2f, %.2f, %.2f\n", target_velocity, now_velocity, Uq);
    vTaskDelay(pdMS_TO_TICKS(1));
}



void FOC_position_control_loop(float target_angle) {

    target_angle = _normalizeAngle(target_angle);

    static float now_angle, electrical_angle, now_velocity;
    FOC_get_velocity(&now_velocity, &electrical_angle, &now_angle);
    now_angle = _normalizeAngle(now_angle);

    float angle_error = target_angle - now_angle;
    if (angle_error < -_PI) target_angle += _2PI;
    else if (angle_error > _PI) target_angle -= _2PI;

    float target_velocity = pid_get_u(&pid_position, target_angle, now_angle);
    now_velocity = FOC_low_pass_filter(&lpf_velocity, now_velocity);
    float Uq = pid_get_u(&pid_velocity, target_velocity, now_velocity);

    FOC_SVPWM(Uq, 0, electrical_angle);

    // printf("%.2f,%.2f,%.2f,%.2f\n", target_angle, now_angle, target_velocity, now_velocity);
    vTaskDelay(pdMS_TO_TICKS(1));
}

// void FOC_spring_loop(float target_angle, PID_Datatype *pid) {
//     target_angle = _normalizeAngle(target_angle);
//     float now_angle = _normalizeAngle(FOC_get_mechanical_angle() - zero_mechanical_angle);

//     float angle_error = target_angle - now_angle;
//     if (angle_error < -_PI) target_angle += _2PI;
//     else if (angle_error > _PI) target_angle -= _2PI;

//     float Uq = FOC_low_pass_filter(&lpf_spring, pid_get_u(pid, target_angle, now_angle));
//     float electrical_angle = FOC_electrical_angle();
//     FOC_SVPWM(Uq, 0, electrical_angle);
// }

// void FOC_knob_loop(uint8_t sector_num) {
//     float now_angle = _normalizeAngle(FOC_get_mechanical_angle() - zero_mechanical_angle);
//     uint8_t now_sector = (uint8_t) floor(now_angle * sector_num / _2PI);
//     float target_angle = now_sector * _2PI / sector_num + _PI / sector_num;

//     float angle_error = target_angle - now_angle;
//     if (angle_error < -_PI) target_angle += _2PI;
//     else if (angle_error > _PI) target_angle -= _2PI;

//     float Uq = pid_get_u(&pid_knob, target_angle, now_angle);

//     float electrical_angle = FOC_electrical_angle();
//     FOC_SVPWM(Uq, 0, electrical_angle);

//     char data = now_sector + '0';

//     HAL_UART_Transmit(&huart1, &data, 1, 0xff);
// }
