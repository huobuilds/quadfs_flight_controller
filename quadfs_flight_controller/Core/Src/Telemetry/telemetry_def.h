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
#ifndef TELEMETRY_DEF_H
#define TELEMETRY_DEF_H

#include "telemetry.h"

typedef enum data_type
{
  CONTROLLER = 0,
  FILTER,
  SENSOR
} data_type_t;
typedef struct telemetry
{
  // inputs
  sensor_t sensor;
  filter_t filter;
  controller_t ctrl;
} telemetry_t;
telemetry_t tel;

typedef struct tx
{
  const float *const data;
  const char *id;
} tx_t;

const tx_t output_sensor[] = {
    // Sensor
    {&tel.sensor.imu_data.accXn, "s_accXn"},
    {&tel.sensor.imu_data.accYn, "s_accYn"},
    {&tel.sensor.imu_data.accZn, "s_accZn"},
    {&tel.sensor.imu_data.gyroXn, "s_gyroXn"},
    {&tel.sensor.imu_data.gyroYn, "s_gyroYn"},
    {&tel.sensor.imu_data.gyroZn, "s_gyroZn"},
    {&tel.sensor.imu_data.magXn, "s_magXn"},
    {&tel.sensor.imu_data.magYn, "s_magYn"},
    {&tel.sensor.imu_data.magZn, "s_magZn"},
    {&tel.sensor.pos_data.pos_z, "s_pos_z"},
    {&tel.sensor.vbatt, "s_vbatt"},
};

const size_t output_sensor_n = sizeof(output_sensor) / sizeof(output_sensor[0]);

const tx_t output_filter[] = {
    // Filter
    {&tel.filter.imu_f_data.roll, "f_roll"},
     {&tel.filter.compf_data.roll, "comf_roll"},
    {&tel.filter.imu_f_data.pitch, "f_pitch"},
    {&tel.filter.imu_f_data.yaw, "f_yaw"},
};
const size_t output_filter_n = sizeof(output_filter) / sizeof(output_filter[0]);

const tx_t output_ctrl[] = {
    // Controller
    {&tel.ctrl.pwm_data.pwm1, "c_pwm1"},
    {&tel.ctrl.pwm_data.pwm2, "c_pwm2"},
    {&tel.ctrl.pwm_data.pwm3, "c_pwm3"},
    {&tel.ctrl.pwm_data.pwm4, "c_pwm4"},
    {&tel.ctrl.setpoint_data.roll_ref, "c_roll_ref"},
    {&tel.ctrl.setpoint_data.pitch_ref, "c_pitch_ref"},
    {&tel.ctrl.setpoint_data.yaw_ref, "c_yaw_ref"},
    {&tel.ctrl.setpoint_data.throttle_z, "c_throttle_z"},
};

const size_t output_ctrl_n = sizeof(output_ctrl) / sizeof(output_ctrl[0]);
#endif // TELEMETRY_DEF_H
