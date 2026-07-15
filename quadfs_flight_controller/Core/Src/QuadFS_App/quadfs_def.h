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
#ifndef QUADFS_DEF_H
#define QUADFS_DEF_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stm32f4xx_hal.h"


#define CONTROLLER_PRIORITY 3
#define FILTER_PRIORITY 3
#define SENSOR_PRIORITY 3
#define TELEMETRY_PRIORITY 2
#define MONITOR_PRIORITY 1

#define STACK_SIZE 128 * 8
#define QUEUE_LENGTH 5

//#define USE_MAGNETOMETER

//ticks 1 tick -> 1ms, 3ticks -> 3ms -> 333Hz  loop freq
//------------------------------------------------------------
// control loop freq(set to 333Hz) < PWM freq (set to 400Hz)
// to allow for pwm updating without interruption
//------------------------------------------------------------
#define TASK_INTERVAL_MS 3

#define M_PI 3.14159265358979323846f /* pi */

#define  FIRMWARE_VERSION_STRING "QuadFS v1.0"

#define TO_RAD (M_PI / 180.0f)
#define TO_DEG (180.0f / M_PI)

#define p_pitch_gain 0.45f
#define i_pitch_gain 0.01f
#define d_pitch_gain 0.1f

#define p_roll_gain 0.45f
#define i_roll_gain 0.01f
#define d_roll_gain 0.1f

#define p_yaw_gain 0.05f
#define i_yaw_gain 0.001f
#define d_yaw_gain 0.0f


#define RADIO_MIN   1000U
#define RADIO_MAX   2000U

#define MAX_THROTTLE 5.0f
#define MIN_THROTTLE 1.8f

#define MAX_THRUST 6.25f // 6.25N the max. rated thrust the motor can technically produce.
#define MIN_THRUST 1.5f

#define PWM_MAX 4000 
#define PWM_MIN 2000

#define PID_LIMIT 1.25f    // max N allowed for roll,pitch and yaw pid
#define DECLINATION +3.53f // Geographic North +3.53 deg declination

/*
 * for roll and pitch
 * F = 0.25*M/l
 * F = input force
 * M = pid torque correction/control input
 * arm_len_C45, l = arm length *cos45 = 0.1167m
 */
#define ATTITUDE_MOMENT_LIMIT 0.4668f //  attitude can only command +-1.0N of thrust

/*
 * for roll and pitch
 * F = 0.25*Myaw*(K_F / K_M)
 * F = input force
 * Myaw = yaw pid torque correction/control input
 * kf/km
 */
#define YAW_MOMENT_LIMIT 0.0531f //  yaw can only command +-1N of thrust

extern QueueHandle_t sensor_telemetry_q;
extern QueueHandle_t filter_telemetry_q;
extern QueueHandle_t controller_telemetry_q;

extern QueueHandle_t sensor_filter_q;
extern QueueHandle_t filter_controller_q;

extern TaskHandle_t Status_indicator_Handle;
extern TaskHandle_t controller_run_Handle;
extern TaskHandle_t telemetry_run_Handle;

enum control_states
{
    safe_start,
    start_stage,
    second_start_stage,
    controller_enabled,
    controller_disabled,
    fault,
    end_control
};
#endif // QUADFS_DEF_H