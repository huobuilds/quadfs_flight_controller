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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <stdbool.h>
#include "sensor.h"
#include "sensor_def.h"
#include "stm32f4xx_hal.h"
#include "imu.h"

extern UART_HandleTypeDef huart2;

static bool create_once = true;
static void sensor_param_init(sensor_param_t sensor_param);
static sensor_t *self = NULL;

// constructor and destructor ============================================
sensor_t *const sensor_create(sensor_param_t sensor_param)
{
	if (create_once)
	{
		self = (sensor_t *)malloc(sizeof(sensor_t));
		sensor_param_init(sensor_param);
		create_once = false;
	}
	return self;
}
void sensor_destory(sensor_t *self)
{
	free(self);
}

// Private members =======================================================
static void sensor_param_init(sensor_param_t sensor_param)
{
	self->sensor_param.alpha = sensor_param.alpha;
}

sensor_t sensor_process_data_ex()
{
	self->imu_data = update_mpu_imu();
	return (*self);
}

sensor_t sensor_output_data()
{
	return (*self);
}
