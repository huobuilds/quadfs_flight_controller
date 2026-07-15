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
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "complementary_filter.h"
#include <math.h>
#include "stm32f4xx_hal.h"

#define M_PI 3.14159265358979323846 /* pi */

comp_filter_angle(const float sample_time_ms,imu_t imu_data, float *new_angle, uint8_t len)
{
 
    static float angle_pitch_acc = 0;
    static float angle_roll_acc = 0;
    static float angle_pitch = 0;
    static float angle_roll = 0;
    static float alphar = 0.999;
    static float alphap = 0.999;
    static bool first_entry = true;
    static double to_deg = ((double)180.0 / M_PI);
    double resultant = 0;
    static float last = 0;
    float now;
    float angle_yaw_mag;
    float angle_yaw;

    angle_roll_acc  = atan2f(imu_data.accYn, imu_data.accZn) * to_deg;
    angle_pitch_acc = atan2f(-imu_data.accXn, sqrtf(imu_data.accYn * imu_data.accYn + imu_data.accZn * imu_data.accZn)) * to_deg;
    
    float dt = sample_time_ms;

    if (first_entry)
    {
        angle_pitch = angle_pitch_acc;
        angle_roll  = angle_roll_acc;
        first_entry = false;
    }
    else
    {
        angle_roll  = (angle_roll  + imu_data.gyroXn * dt) * alphar + angle_roll_acc  * (1.0f - alphar);
        angle_pitch = (angle_pitch + imu_data.gyroYn * dt) * alphap + angle_pitch_acc * (1.0f - alphap);

        // resultant = sqrt((imu_data.accXn * imu_data.accXn) + (imu_data.accYn * imu_data.accYn) + (imu_data.accZn * imu_data.accZn)); // should be approx. 1.0

        // if ( resultant > 0.85f &&  resultant < 1.15f){
        //     angle_roll  = (angle_roll  + imu_data.gyroXn * dt) * alphar + angle_roll_acc  * (1.0f - alphar);
        //     angle_pitch = (angle_pitch + imu_data.gyroYn * dt) * alphap + angle_pitch_acc * (1.0f - alphap);
        // }else{//skip acc vibrations
        //     angle_roll  = (angle_roll  + imu_data.gyroXn * dt);
        //     angle_pitch = (angle_pitch + imu_data.gyroYn * dt);
        // }
    }
    
    new_angle[0] = angle_roll;
    new_angle[1] = angle_pitch;
}
