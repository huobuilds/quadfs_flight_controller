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
#ifndef SENSOR_H
#define SENSOR_H

#include "imu.h"

typedef struct pos
{
    float pos_x;
    float pos_y;
    float pos_z;
} pos_t;

typedef struct sensor_parameter
{
	float alpha;
} sensor_param_t;

typedef struct sensor
{
	imu_t imu_data;
	pos_t pos_data;
	float vbatt;
	sensor_param_t sensor_param;
} sensor_t;


sensor_t * const sensor_create(sensor_param_t sensor_param);
void sensor_destory(sensor_t *self);
sensor_t sensor_process_data_ex(void);
sensor_t sensor_output_data();

#endif // SENSOR_H
