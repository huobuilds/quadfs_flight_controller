/**
 ******************************************************************************
 *  QuadFS - quadcopter flight controller built from scratch on STM32
 *
 *  Copyright (C) 2026  Henry Odoemelem <huobuilds@gmail.com>
 *
 *  SPDX-License-Identifier: GPL-3.0-or-later
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "pid_controller.h"
#include "math.h"
#include "stdlib.h"
#include <stdbool.h>
#include "radio.h"

#define THRUST_TABLE_SIZE 8

//920kv, 8045 propellers
static const float thrust_table_N[THRUST_TABLE_SIZE] = {
    0.000f,
    0.245f,
    0.441f,
    0.540f,
    0.834f,
    1.619f,
    2.109f,
    3.875f
};

static const uint16_t pwm_table[THRUST_TABLE_SIZE] = {
    2000,
    2504,
    2636,
    2752,
    2928,
    3354,
    3518,
    4000
};

typedef struct pid
{
    float pos_z_integral;
    float roll_integral;
    float pitch_integral;
    float yaw_integral;

    float torque_control_limit;     //  attitude can only command +-0.60N of thrust
    float torque_control_limit_yaw; //  yaw can only command +-0.30N of thrust
    float pos_z_thrust_limit;

    bool is_pos_z_saturated;
    bool is_roll_saturated;
    bool is_pitch_saturated;
    bool is_yaw_saturated;

    bool is_pos_z_same_sign;
    bool is_roll_same_sign;
    bool is_pitch_same_sign;
    bool is_yaw_same_sign;

    // correction ie thrust or torque input required to correct errors
    float pos_z_correction;
    float roll_correction;
    float pitch_correction;
    float yaw_correction;
    float pos_z_torque_temp;
    float roll_torque_temp;
    float pitch_torque_temp;
    float yaw_torque_temp;

    float pos_z_derivative;
    float roll_derivative;
    float pitch_derivative;
    float yaw_derivative;

    float pos_z_error;
    float pos_z_prev_error;
    float roll_error;
    float pitch_error;
    float yaw_error;

    float dt;

    float min_thrust;
    float max_thrust;
    float pwm_max;
    float pwm_min;

    float F1;
    float F2;
    float F3;
    float F4;

    bool first_entry;
    float last_excution_time;

    setpoint_t pid_setpoint;

} pid_param_t;
pid_param_t pid_param;

static float moment_limit(float moment, float limit);
static bool check_sign(float moment, float error);
static bool check_saturation(float moment, float moment_temp);
static float calc_integral(bool is_saturated, bool is_same_sign, float error, float integral, float dt);
static void motor_mixing();
static void saturate_force();
static pwm_t force_to_pwm();
static float limit_force(float F);
static float calc_pwm(float F);
static void limit_correction();
static void monitor_saturation();
static void monitor_sign_change();
static void calc_control_input();
static void calc_derivative_part(const imu_t imu);
static void calc_integral_part();
static void calc_proportional_part(const setpoint_t setpoint, const iekf_t actual,const imu_t imu);
static float wrap_180(float angle);
static uint16_t thrust_to_pwm(float F_N);
static uint16_t interp_pwm_table(float F_N, uint8_t i);

// Wrap any angle to ±180°
static float wrap_180(float angle)
{
    angle = fmodf(angle + 180.0f, 360.0f);
    if (angle < 0.0f) angle += 360.0f;
    return angle - 180.0f;
}

void init_pid_attitude_heading_controller(const float dt)
{
    pid_param.pos_z_integral = 0.0f;
    pid_param.roll_integral = 0.0f;
    pid_param.pitch_integral = 0.0f;
    pid_param.yaw_integral = 0.0f;

    pid_param.is_pos_z_saturated = false;
    pid_param.is_roll_saturated = false;
    pid_param.is_pitch_saturated = false;
    pid_param.is_yaw_saturated = false;

    pid_param.is_pos_z_same_sign = false;
    pid_param.is_roll_same_sign = false;
    pid_param.is_pitch_same_sign = false;
    pid_param.is_yaw_same_sign = false;

    pid_param.pos_z_prev_error = 0.0f;

    pid_param.torque_control_limit = ATTITUDE_MOMENT_LIMIT;
    pid_param.torque_control_limit_yaw = YAW_MOMENT_LIMIT;

    pid_param.pos_z_thrust_limit = MAX_THROTTLE;

    pid_param.dt = dt;

    pid_param.min_thrust = MIN_THRUST;
    pid_param.max_thrust = MAX_THRUST;
    pid_param.pwm_max = PWM_MAX;
    pid_param.pwm_min = PWM_MIN;

    pid_param.first_entry = true;
    pid_param.last_excution_time = (float)HAL_GetTick();
}
pwm_t pid_attitude_heading_controller(const imu_t imu, const setpoint_t setpoint, const filter_t actual)
{
    // /*    //// PID controller  steps
    //      calc_proportional_part
    //      calc_integral_part
    //      calc_derivative_part
    //      calc_control_input
    //      limit_correction
    //      monitor_saturation
    //      monitor_sign_change
    //      motor_mixing
    //      saturate_force();
    //      return force_to_pwm();
    //  */


    //put in a state machine
    calc_proportional_part(setpoint, actual.imu_f_data,imu);
    calc_integral_part();
    calc_derivative_part(imu);
    calc_control_input();
    limit_correction();
    monitor_saturation();
    monitor_sign_change();
    motor_mixing(setpoint);
    saturate_force();

    return force_to_pwm();
}


static void calc_proportional_part(const setpoint_t setpoint, const iekf_t actual,const imu_t imu)
{
    static float to_rad = TO_RAD;
    
    // P
    pid_param.roll_error = (setpoint.roll_ref - actual.roll) * to_rad;
    pid_param.pitch_error = (setpoint.pitch_ref - actual.pitch) * to_rad;

    #if defined(USE_MAGNETOMETER)
        // Compute heading error — always take the shortest angular path
        float yaw_error = wrap_180(setpoint.yaw_ref - actual.yaw); // angle hold control
        pid_param.yaw_error = yaw_error * to_rad;
    #else
        float setpoint_yaw_rate =  setpoint.yaw_ref; // in this case the input(setpoint.yaw_ref) is the yaw rate (not angle yaw)
        float yaw_rate_error = setpoint_yaw_rate - imu.gyroZnf; //yaw rate control
        pid_param.yaw_error = yaw_rate_error * to_rad;
    #endif
}
static void calc_integral_part()
{
    pid_param.roll_integral = calc_integral(pid_param.is_roll_saturated, pid_param.is_roll_same_sign, pid_param.roll_error, pid_param.roll_integral, pid_param.dt);
    pid_param.pitch_integral = calc_integral(pid_param.is_pitch_saturated, pid_param.is_pitch_same_sign, pid_param.pitch_error, pid_param.pitch_integral, pid_param.dt);
    pid_param.yaw_integral = calc_integral(pid_param.is_yaw_saturated, pid_param.is_yaw_same_sign, pid_param.yaw_error, pid_param.yaw_integral, pid_param.dt);
}
static void calc_derivative_part(const imu_t imu)
{
    static float to_rad = TO_RAD;
    // D
    //  -gyro rate has less noise and no derivative kick and has same waveform as a derivative.
    pid_param.roll_derivative = (0.0f - imu.gyroXnf) * to_rad;
    pid_param.pitch_derivative = (0.0f - imu.gyroYnf) * to_rad;
    pid_param.yaw_derivative = (0.0f - imu.gyroZnf) * to_rad;
}

static void calc_control_input()
{
    // calc. Input torque to the system
    pid_param.roll_correction = p_roll_gain * pid_param.roll_error + i_roll_gain * pid_param.roll_integral + d_roll_gain * pid_param.roll_derivative;
    pid_param.pitch_correction = p_pitch_gain * pid_param.pitch_error + i_pitch_gain * pid_param.pitch_integral + d_pitch_gain * pid_param.pitch_derivative;
    pid_param.yaw_correction = p_yaw_gain * pid_param.yaw_error + i_yaw_gain * pid_param.yaw_integral + d_yaw_gain * pid_param.yaw_derivative;

    pid_param.roll_torque_temp = pid_param.roll_correction;
    pid_param.pitch_torque_temp = pid_param.pitch_correction;
    pid_param.yaw_torque_temp = pid_param.yaw_correction;
}
static void monitor_sign_change()
{
    // check signs
    pid_param.is_roll_same_sign = check_sign(pid_param.roll_correction, pid_param.roll_error);
    pid_param.is_pitch_same_sign = check_sign(pid_param.pitch_correction, pid_param.pitch_error);
    pid_param.is_yaw_same_sign = check_sign(pid_param.yaw_correction, pid_param.yaw_error);
    // end check signs
}
static void monitor_saturation()
{
    // check saturation
    pid_param.is_roll_saturated = check_saturation(pid_param.roll_correction, pid_param.roll_torque_temp);
    pid_param.is_pitch_saturated = check_saturation(pid_param.pitch_correction, pid_param.pitch_torque_temp);
    pid_param.is_yaw_saturated = check_saturation(pid_param.yaw_correction, pid_param.yaw_torque_temp);
    // end check saturation
}
static void limit_correction()
{
    // Do saturation
    pid_param.roll_correction = moment_limit(pid_param.roll_correction, pid_param.torque_control_limit);
    pid_param.pitch_correction = moment_limit(pid_param.pitch_correction, pid_param.torque_control_limit);
    pid_param.yaw_correction = moment_limit(pid_param.yaw_correction, pid_param.torque_control_limit_yaw);
    // end do saturation
}
static float calc_integral(bool is_saturated, bool is_same_sign, float error, float integral, float dt)
{
    //integral anti-windup clamping
    return (is_saturated && is_same_sign) ? integral : (integral + (error * dt));  
    //return  (integral + (error * dt)); // no clamping
}
static bool check_saturation(float moment, float moment_temp)
{
    return !(moment == moment_temp) ? true : false;
}
static bool check_sign(float moment, float error)
{
    return (moment * error) > 0 ? true : false;
}
static float moment_limit(float moment, float limit)
{
    limit = fabsf(limit);
    if (fabsf(moment) > limit)
        moment = moment < 0.0f ? -1.0f*limit : limit;
    return moment;
}

static void motor_mixing(const setpoint_t setpoint)
{

    static float throttle_force = 0;

    throttle_force = setpoint.throttle_z;
    
    float pid_max = PID_LIMIT; // N allowed for roll,pitch and yaw pid
    float K_F_M = K_F / K_M;   

    if (throttle_force > (pid_param.max_thrust - pid_max)) 
        throttle_force = (pid_param.max_thrust - pid_max);


    // calc. control force for each motor
    pid_param.F1 = throttle_force - 0.25f * pid_param.roll_correction / arm_len_C45 + 0.25f * pid_param.pitch_correction / arm_len_C45 + 0.25f * pid_param.yaw_correction * K_F_M;
    pid_param.F2 = throttle_force - 0.25f * pid_param.roll_correction / arm_len_C45 - 0.25f * pid_param.pitch_correction / arm_len_C45 - 0.25f * pid_param.yaw_correction * K_F_M;
    pid_param.F3 = throttle_force + 0.25f * pid_param.roll_correction / arm_len_C45 - 0.25f * pid_param.pitch_correction / arm_len_C45 + 0.25f * pid_param.yaw_correction * K_F_M;
    pid_param.F4 = throttle_force + 0.25f * pid_param.roll_correction / arm_len_C45 + 0.25f * pid_param.pitch_correction / arm_len_C45 - 0.25f * pid_param.yaw_correction * K_F_M;
}
static void saturate_force()
{
    // Limit thrust
    pid_param.F1 = limit_force(pid_param.F1);
    pid_param.F2 = limit_force(pid_param.F2);
    pid_param.F3 = limit_force(pid_param.F3);
    pid_param.F4 = limit_force(pid_param.F4);
}
static pwm_t force_to_pwm()
{
    pwm_t pwm_data;
    // Map forces to PWM values
    pwm_data.pwm1 = calc_pwm(pid_param.F1); // M1
    pwm_data.pwm2 = calc_pwm(pid_param.F2); // M2
    pwm_data.pwm3 = calc_pwm(pid_param.F3); // M3
    pwm_data.pwm4 = calc_pwm(pid_param.F4); // M4

    return pwm_data;
}
static float limit_force(float F)
{
    F = (F < pid_param.min_thrust) ? pid_param.min_thrust : F;
    F = (F > pid_param.max_thrust) ? pid_param.max_thrust : F;
    return F;
}

static float calc_pwm(float F)
{
    return (int)((pid_param.pwm_min + ((F - pid_param.min_thrust) * (pid_param.pwm_max - pid_param.pwm_min)) / (pid_param.max_thrust - pid_param.min_thrust)));

    //return  (float)thrust_to_pwm(F);
}

static uint16_t thrust_to_pwm(float F_N)
{
    if (F_N <= thrust_table_N[0]) {
        return pwm_table[0];
    }

    if (F_N >= thrust_table_N[THRUST_TABLE_SIZE - 1]) {
        return pwm_table[THRUST_TABLE_SIZE - 1];
    }

    if (F_N <= thrust_table_N[1]) {
        return interp_pwm_table(F_N, 0);
    }
    else if (F_N <= thrust_table_N[2]) {
        return interp_pwm_table(F_N, 1);
    }
    else if (F_N <= thrust_table_N[3]) {
        return interp_pwm_table(F_N, 2);
    }
    else if (F_N <= thrust_table_N[4]) {
        return interp_pwm_table(F_N, 3);
    }
    else if (F_N <= thrust_table_N[5]) {
        return interp_pwm_table(F_N, 4);
    }
    else if (F_N <= thrust_table_N[6]) {
        return interp_pwm_table(F_N, 5);
    }
    else {
        return interp_pwm_table(F_N, 6);
    }
}

static uint16_t interp_pwm_table(float F_N, uint8_t i)
{
    float F1 = thrust_table_N[i];
    float F2 = thrust_table_N[i + 1];

    float PWM1 = (float)pwm_table[i];
    float PWM2 = (float)pwm_table[i + 1];

    float pwm = PWM1 + ((F_N - F1) * (PWM2 - PWM1)) / (F2 - F1);

    return (uint16_t)(pwm + 0.5f);
}