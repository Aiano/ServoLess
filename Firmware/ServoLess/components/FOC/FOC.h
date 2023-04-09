/**
 * @file FOC.h
 * @author Aiano_czm
 * @brief
 */

#ifndef FOC_DRIVER_FOC_H
#define FOC_DRIVER_FOC_H

#include "FOC_PID.h"
#include <stdint.h>

typedef enum
{
    OPEN_LOOP_POSITION_CONTROL = 0,
    OPEN_LOOP_SPEED_CONTROL,
    TORQUE_CONTROL,
    SPEED_CONTROL,
    POSITION_CONTROL,
    SPRING,
    SPRING_WITH_DAMP,
    DAMP,
    KNOB,
    ZERO_RESISTANCE
} FOC_CONTROL_MODE;

#define FOC_CONTROL_MODE_NUM 10

extern float output_Id, output_Iq;
extern float output_mechanical_angle, output_velocity;
extern uint64_t loop_timer_count;

void FOC_init();
void FOC_electrical_angle_calibration();
void FOC_SVPWM(float Uq, float Ud, float angle);
void FOC_Clarke_Park(float Ia, float Ib, float Ic, float angle, float *Id, float *Iq);
float FOC_get_mechanical_angle();
float FOC_electrical_angle();
uint8_t FOC_test_pole_pairs();

void FOC_init_vel_timer();
uint64_t FOC_vel_timer_get_dt_us();
void FOC_get_velocity(float *velocity, float *electrical_angle, float *mechanical_angle);

void FOC_init_loop_timer();
void FOC_loop_timer_spin();

// Some modes to choose
void FOC_open_loop_voltage_control_loop(float Uq);
void FOC_current_control_loop(float target_Iq);
void FOC_velocity_control_loop(float target_velocity);
void FOC_position_control_loop(float target_angle);
// void FOC_spring_loop(float target_angle, PID_Datatype *pid);
// void FOC_knob_loop(uint8_t sector_num);
#endif // FOC_DRIVER_FOC_H
