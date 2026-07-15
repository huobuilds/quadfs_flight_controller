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
#ifndef IEKF_H_
#define IEKF_H_


#include  "stm32f411xe.h"                  // Device header
#define ARM_MATH_CM4
#include "arm_math.h"                   // ARM::CMSIS:DSP
#include <stdlib.h>
 #include "imu.h"


typedef struct iekf
{
    float roll;
    float pitch;
    float yaw;
} iekf_t;


iekf_t iekf_orientation_estimation(const float sample_time_s, const imu_t imu_data);


#endif /* IEKF_H_ */