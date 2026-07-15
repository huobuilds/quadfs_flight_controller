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
#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "filter.h"
#include "quadfs_def.h"
#include "motor_pwm.h"
#include "setpoint.h"


typedef struct controller
{
    pwm_t pwm_data;
    setpoint_t setpoint_data;
    filter_t filter_data;
} controller_t;


controller_t *const controller_create();
void controller_destory(controller_t *self);
controller_t ctrl_output_data();
BaseType_t controller_launch_thread(controller_t *const self);

#endif // CONTROLLER_H