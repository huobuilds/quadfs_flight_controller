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
#include "pid_controller.h"
#include "controller.h"
#include "controller_def.h"
#include "queue.h"
#include "radio.h"
#include "status_monitor.h"
#include "imu.h"
#include "stm32f4xx_hal.h"
#include "main.h"

static enum control_states ctrl_state = controller_disabled;

static void controller_run(void *const self_);
static void controller_process_data();
static void quadfs_status_indicator();
static controller_t ouptut_c;

QueueHandle_t controller_telemetry_q;
TaskHandle_t controller_run_Handle;
BaseType_t controller_run_return_Handle;
static bool create_once = true;
static bool launch_once = true;

//ticks 1 tick -> 1ms, 3ticks -> 3ms -> 333Hz  loop freq
//------------------------------------------------------------
// control loop freq(set to 333Hz) < PWM freq (set to 400Hz)
// to allow for pwm/motor updating without interruption
//------------------------------------------------------------
static uint8_t tick_time = TASK_INTERVAL_MS;
static controller_t *self = NULL;

// constructor and destructor ============================================
controller_t *const controller_create()
{
    if (create_once)
    {
        self = (controller_t *)malloc(sizeof(controller_t));
    }
    return self;
}

void controller_destory(controller_t *self)
{
    free(self);
}

// Public members ========================================================
BaseType_t controller_launch_thread(controller_t *const self)
{
    if (launch_once)
    {
        controller_telemetry_q = xQueueCreate(QUEUE_LENGTH, sizeof(controller_t));
        controller_run_return_Handle = xTaskCreate(controller_run, "Controller Activity Task",
                                                  configMINIMAL_STACK_SIZE*8,
                                                   (void *)self, CONTROLLER_PRIORITY, &controller_run_Handle);
        launch_once = false;
    }
    return controller_run_return_Handle;
}

// Private members =======================================================

static void controller_run(void *const self_)
{
    TickType_t xLastWakeTime;
    // Initialize the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        update_channels();
        self->filter_data = filter_process_data_ex();
        controller_process_data();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(tick_time));
    }
}

static void controller_process_data()
{
    //SWA 1000 is up(use for off or disarm), 2000 is down (for on or arm )
    uint16_t CH_MID = 1500;

    switch (ctrl_state)
    {
        case safe_start:
            self->pwm_data = turn_motors_off();
            if (channels[CH_SWA] < CH_MID)
            {
                ctrl_state = start_stage;
            }
        case start_stage:
            self->pwm_data = turn_motors_off();
            if (channels[CH_SWA] > CH_MID)
            {
                init_setpoints(self->filter_data.imu_f_data.yaw);
                init_pid_attitude_heading_controller((float)(tick_time*0.001f));
                ctrl_state = controller_enabled;
            }
            break;
        case controller_enabled:
            if (channels[CH_SWA] < CH_MID)
            {
                ctrl_state = controller_disabled;
                break;
            }

            self->setpoint_data = update_setpoints(self->filter_data.imu_data.gyroZnf,
                                                   self->filter_data.imu_f_data.yaw, 
                                                   tick_time);
            self->pwm_data = pid_attitude_heading_controller(self->filter_data.imu_data, 
                                                             self->setpoint_data,
                                                             self->filter_data);

            set_motors_pwm(self->pwm_data);
            break;
        case controller_disabled:
            self->pwm_data = turn_motors_off();
            init_setpoints(self->filter_data.imu_f_data.yaw);
            init_pid_attitude_heading_controller((float)(tick_time * 0.001f));
            ctrl_state = safe_start;
            break;
        default:
            break;
    }


    ouptut_c = (*self);  
    quadfs_status(ctrl_state);  
}

controller_t ctrl_output_data()
{
    return ouptut_c;
}
