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
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "imu.h"
#include "setpoint.h"
#include "filter.h"
#include "motor_pwm.h"

#define arm_len_C45 0.1167f // perpendicular len, arm_len_C45 = arm length*cos(45) [m], 330 mm  diameter frame, arm length = 0.33/2 m
#define mass   0.8f // platform weight [kg] 
#define G 9.80665f // acceleration due to gravity(meters per second squared)

//see in doc: Lift_Drag_Constants_8045_Concise.pdf
#define K_F 6.35E-8f  // thrust constant c -> F=c*w^2; thrust
#define K_M 8.43E-10f //  moment constant b -> M=b*w^2; moment
//kf/km = 75.32 

void init_pid_attitude_heading_controller(const float dt);
pwm_t pid_attitude_heading_controller(const imu_t imu,const setpoint_t setpoint, const filter_t actual);

#endif /* PID_CONTROLLER_H_ */