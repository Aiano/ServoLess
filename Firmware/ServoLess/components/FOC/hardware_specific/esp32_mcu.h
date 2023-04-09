#pragma once

#include "hardware_api.h"

// default pwm parameters
// #define _PWM_RESOLUTION 12 // 12bit
// #define _PWM_RANGE 4095.0 // 2^12 -1 = 4095
// #define _PWM_FREQUENCY 25000 // 25khz
// #define _PWM_FREQUENCY_MAX 50000 // 50khz

#define TIMER_RESOLUTION_HZ 80000000  // 80MHz
#define TIMER_PERIOD 250     // 5000 ticks, 62us, 16kHz
#define MOTOR_IN1_GPIO 1
#define MOTOR_IN2_GPIO 2
#define MOTOR_IN3_GPIO 3
#define MOTOR_EN_GPIO 4