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
#ifndef SETPOINT_H
#define SETPOINT_H

#define CH_ROL (0)
#define CH_PIT (1)
#define CH_THR (2)
#define CH_YAW (3)
#define CH_SWA (4)
#define CH_SWB (5)
#define CH_SWC (6)
#define CH_SWD (7)
#define CH_VRA (8)
#define CH_VRB (9) 

typedef struct setpoint
{
    float roll_ref;
    float pitch_ref;
    float yaw_ref;
    float throttle_z;
} setpoint_t;

void init_setpoints(double yaw_z);
setpoint_t update_setpoints(double gyro_z,double yaw_z,uint8_t tick_time);
#endif