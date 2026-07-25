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

#include "status_monitor.h"
#include "quadfs_def.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "filter.h"
#include "radio.h"
#include "setpoint.h"
#include "motor_pwm.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
TaskHandle_t Status_indicator_Handle;
BaseType_t Status_indicator_return_Handle;

TickType_t xLastWakeTime;
static int tick_time = 20;// ticks 1 tick -> 1ms, 20ticks -> 20ms -> 50Hz  update freq.
static bool launch_once = true;
uint8_t quadfs_state = 0;

extern uint16_t channels[channel_on]; // 0 roll,1-pitch,2-lift/throttle,3-yaw,4-SWA,7-SWD

static void status_monitor(void *pvParameters);


void status_monitor_launch()
{
	if (launch_once)
	{
		Status_indicator_return_Handle = xTaskCreate(status_monitor, "Status indicator Activity Task",
													 configMINIMAL_STACK_SIZE*4,
													 NULL, MONITOR_PRIORITY, &Status_indicator_Handle);
		launch_once = false;
	}
}

static void status_monitor(void *pvParameters)
{
    uint32_t last_toggle_time = HAL_GetTick();
    bool first_entry = true;

	// Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	while (1)
	{
        uint32_t now = HAL_GetTick();

        if (quadfs_state == controller_enabled)
        {
            //armed
            if(first_entry){
                //armed
                HAL_GPIO_WritePin(status_led_GPIO_Port, status_led_Pin, GPIO_PIN_SET);// LED ON
                first_entry = false;
            }
        }
        else
        {   
            //unarmed
            first_entry = true;

            if ((now - last_toggle_time) >= 100U)
            {//one cycle: on 100ms, off 100ms, cycle 200ms, ie 5Hz blink
                HAL_GPIO_TogglePin(status_led_GPIO_Port, status_led_Pin);
                last_toggle_time += 100U;
            }
            
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(tick_time));

    }
}

void quadfs_status(enum control_states ctrl_state)
{
    quadfs_state = ctrl_state;
}