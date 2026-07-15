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
#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

typedef struct pwm
{
	float pwm1;
	float pwm2;
    float pwm3;
    float pwm4;
} pwm_t;
void motor_init(void);
void set_motors_pwm(pwm_t pwm);
pwm_t turn_motors_off(void);
uint16_t radio_to_pwm(uint16_t radio_val);
void esc_calibrate_all(void);
#endif