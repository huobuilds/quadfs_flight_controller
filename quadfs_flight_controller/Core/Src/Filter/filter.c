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
#include <stdbool.h>
#include "filter.h"
#include "filter_def.h"
#include "stm32f4xx_hal.h"
#include "queue.h"
#include "main.h"


#define M_PI 3.14159265358979323846f /* pi */
#define IMU_ROLL_TILT_OFFSET 0.3175f
#define IMU_PITCH_TILT_OFFSET 0.4663f;
static int tick_time = TASK_INTERVAL_MS;


static void filter_param_init(filter_param_t filter_param);
static bool create_once = true;
static filter_t *self = NULL;
float dt = 0.0f;
uint32_t last_cycle_count;

// constructor and destructor ============================================
filter_t *const filter_create(filter_param_t filter_param)
{
    last_cycle_count = DWT->CYCCNT;
  
    if (create_once)
    {
        self = (filter_t *)malloc(sizeof(filter_t));
        filter_param_init(filter_param);
        create_once = false;
    }
    return self;
}

void filter_destory(filter_t *self)
{
    free(self);
}

// Private members =======================================================

static void filter_param_init(filter_param_t filter_param)
{
    self->filter_param.gain = filter_param.gain;
}

filter_t filter_process_data_ex()
{
    // //for loop measurement
    // uint32_t current_cycle_count = DWT->CYCCNT;
    // uint32_t elapsed_cycles = current_cycle_count - last_cycle_count;
    // last_cycle_count = current_cycle_count;
    // dt = (float)elapsed_cycles / (float)SystemCoreClock;

    float to_deg = TO_DEG;
    float to_rad = TO_RAD;
    float declination = DECLINATION;
    float new_angle[2];

    sensor_process_data_ex();
    self->imu_data = sensor_output_data().imu_data;
    // comp_filter_angle(dt,self->imu_data, new_angle,2);
    // self->compf_data.roll = new_angle[0];
    // self->compf_data.pitch = new_angle[1];

    iekf_t iekf_angles = iekf_orientation_estimation(tick_time, self->imu_data);
    self->imu_f_data = iekf_angles;
    self->imu_f_data.roll =  self->imu_f_data.roll + IMU_ROLL_TILT_OFFSET;//apply tilt offset
    self->imu_f_data.pitch =  -1.0f*self->imu_f_data.pitch + IMU_PITCH_TILT_OFFSET;//apply tilt offset
    
    return (*self);
}

filter_t filter_output_data()
{
    return (*self);
}
