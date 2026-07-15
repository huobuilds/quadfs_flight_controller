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
#include "motor_pwm.h"
#include <stdio.h>
#include <stdlib.h>
#include "quadfs_def.h"
#include "main.h"

static uint16_t clamp_pwm_values(uint16_t value);
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

void motor_init(void)
{
    /*
     * Motor outputs:
     * PA0 -> TIM2_CH1
     * PA1 -> TIM2_CH2
     * PA8 -> TIM1_CH1
     * PA9 -> TIM1_CH2
     */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    /*
     * TIM1 is an advanced timer.
     * This ensures the main output is enabled.
     */
    __HAL_TIM_MOE_ENABLE(&htim1);

    turn_motors_off();
}

static uint16_t clamp_pwm_values(uint16_t value)
{
    if (value < PWM_MIN)
    {
        return PWM_MIN;
    }

    if (value > PWM_MAX)
    {
        return PWM_MAX;
    }

    return value;
}

void set_motors_pwm(pwm_t pwm)
{
    uint16_t m1 = clamp_pwm_values((uint16_t)pwm.pwm1);
    uint16_t m2 = clamp_pwm_values((uint16_t)pwm.pwm2);
    uint16_t m3 = clamp_pwm_values((uint16_t)pwm.pwm3);
    uint16_t m4 = clamp_pwm_values((uint16_t)pwm.pwm4);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, m1); /* PA1 Motor 1 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, m2); /* PA0 Motor 2 */

   
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, m3); /* PA9 Motor 3 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, m4); /* PA8 Motor 4 */
}

pwm_t turn_motors_off(void)
{
    pwm_t pwm;

    pwm.pwm1 = 2000; // M1
    pwm.pwm2 = 2000; // M2
    pwm.pwm3 = 2000; // M3
    pwm.pwm4 = 2000; // M4

    
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint16_t)pwm.pwm1); /* PA1 Motor 1 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint16_t)pwm.pwm2); /* PA0 Motor 2 */

   
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)pwm.pwm3); /* PA9 Motor 3 */
     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)pwm.pwm4); /* PA8 Motor 4 */
    return pwm;
}


uint16_t radio_to_pwm(uint16_t radio_val)
{
    if (radio_val < RADIO_MIN) radio_val = RADIO_MIN;
    if (radio_val > RADIO_MAX) radio_val = RADIO_MAX;

    return PWM_MIN +
           ((uint32_t)(radio_val - RADIO_MIN) * (PWM_MAX - PWM_MIN)) /
           (RADIO_MAX - RADIO_MIN);
}

void esc_calibrate_all(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    /*
        * TIM1 is an advanced timer.
        * This ensures the main output is enabled.
        */
    __HAL_TIM_MOE_ENABLE(&htim1);


    pwm_t pwm_data;

    // 1. Send maximum throttle before powering ESCs

    pwm_data.pwm1 =  4000;
    pwm_data.pwm2 =  pwm_data.pwm1;
    pwm_data.pwm3 =  pwm_data.pwm1;
    pwm_data.pwm4 =  pwm_data.pwm1;
    set_motors_pwm(pwm_data);

    HAL_Delay(5000);

    // LED ON
    HAL_GPIO_WritePin(status_led_GPIO_Port, GPIO_PIN_5, GPIO_PIN_SET);



    // Now connect/power the ESC battery here
    // Wait for ESC calibration beep
    HAL_Delay(5000);

    // 2. Send minimum throttle
    pwm_data.pwm1 =  2000;
    pwm_data.pwm2 =  pwm_data.pwm1;
    pwm_data.pwm3 =  pwm_data.pwm1;
    pwm_data.pwm4 =  pwm_data.pwm1;
    set_motors_pwm(pwm_data);

    // Wait for confirmation beeps
    HAL_Delay(5000);
    // LED OFF
    HAL_GPIO_WritePin(status_led_GPIO_Port, GPIO_PIN_5, GPIO_PIN_RESET);

    // 3. Keep ESCs at minimum
    set_motors_pwm(pwm_data);
}
