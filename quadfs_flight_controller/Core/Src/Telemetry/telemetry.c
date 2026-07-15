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
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "telemetry.h"
#include "telemetry_def.h"
#include "queue.h"
#include "radio.h"
#include "quadfs_def.h"
#include "setpoint.h"

static bool launch_once = true;
static void telemetry_run(void *const tel_ptr);
static bool telemetry_update_input_data(telemetry_t *const tel, const data_type_t input_type);
static void telemetry_process_data(const tx_t output[], const size_t output_len);
static void send(telemetry_t *const tel, const data_type_t input_type, const tx_t output[], size_t output_len);

extern double gx_offset;
extern double gy_offset;
extern double gz_offset;

// constructor and destructor ============================================

TaskHandle_t telemetry_run_Handle;
BaseType_t telemetry_run_return_Handle;

// Public members ========================================================

BaseType_t telemetry_launch_thread()
{
    if (launch_once)
    {
        telemetry_run_return_Handle = xTaskCreate(telemetry_run, "Telemetry Activity Task",
                                                 configMINIMAL_STACK_SIZE*4,
                                                  (void *)&tel, TELEMETRY_PRIORITY, &telemetry_run_Handle);
        launch_once = false;
    }
    return telemetry_run_return_Handle;
}

// Private members =======================================================
static void telemetry_run(void *const tel_ptr)
{
    TickType_t xLastWakeTime;
    // Initialize the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    static int tick_time = 10;// 100Hz telemetry

    telemetry_t *const tel = (telemetry_t *)tel_ptr;
    while (true)
    {
        send(tel, SENSOR, output_sensor, output_sensor_n);
        send(tel, FILTER, output_filter, output_filter_n);
        send(tel, CONTROLLER, output_ctrl, output_ctrl_n);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(tick_time));
    }
    free(tel);
}

static void send(telemetry_t *const tel, const data_type_t input_type, const tx_t output[], const size_t output_len)
{
    if (telemetry_update_input_data(tel, input_type))
        telemetry_process_data(output, output_len);
}

static void telemetry_process_data(const tx_t output[], const size_t output_len)
{

    static char msg[500];


    int len = snprintf(msg,
            sizeof(msg),
            "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
            tel.filter.imu_f_data.roll,tel.filter.imu_f_data.pitch,
            tel.ctrl.pwm_data.pwm1,
            tel.ctrl.pwm_data.pwm2,
            tel.ctrl.pwm_data.pwm3,
            tel.ctrl.pwm_data.pwm4);

    CDC_Transmit_FS((uint8_t *)msg, (uint16_t)len);

}

static bool telemetry_update_input_data(telemetry_t *const tel, const data_type_t input_type)
{
    switch (input_type)
    {
        case CONTROLLER:
            tel->ctrl = ctrl_output_data();
            return true;
            break;
        case FILTER:
            tel->filter = filter_output_data();
            return true;
            break;
        case SENSOR:
            tel->sensor = sensor_output_data();
            return true;
            break;
        default:
            break;
    }
    return false;
}
