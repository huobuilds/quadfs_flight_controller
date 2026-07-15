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
#ifndef FILTER_H
#define FILTER_H
#include "sensor.h"
#include "quadfs_def.h"
#include "iekf.h"
#include "complementary_filter.h"


typedef struct filter_parameter
{
	float gain;
} filter_param_t;


typedef struct filter
{
	iekf_t imu_f_data;
	imu_t imu_data;
	compf_t compf_data;
	filter_param_t filter_param;
} filter_t;

filter_t *const filter_create(filter_param_t filter_param);
void filter_destory(filter_t *self);
filter_t filter_output_data();
filter_t filter_process_data_ex();


#endif // FILTER_H