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
#include "arm_math.h" // ARM::CMSIS:DSP
#include "radio.h"
#include "setpoint.h"
#include "quadfs_def.h"
#include <stdlib.h>

#define CH_MAX 2000
#define CH_MID 1500
#define CH_MIN 1000

//roll stick does not have full range
#define CH_ROL_MIN 1043
#define CH_ROL_MAX 1970

#define YAW_DEADBAND         0.05f

#define YAW_DEADBAND_US      30u
#define YAW_RATE_DEG_PER_SEC   200.0f// max yaw rate when stick fully deflected

// RC channel thresholds (PWM microseconds)
#define CW_LIMIT                1525u    // yaw stick CW threshold
#define CCW_LIMIT               1450u    // yaw stick CCW threshold
#define THR_LOW                 1100u    // minimum throttle to enable yaw (after hover)

static float RC_delta_yaw  = 0.0f;
static float prev_yaw_z    = 0.0f;

static setpoint_t setpoint_data;
static float calc_heading_setpoint(double yaw_z,uint8_t tick_time);
static float yaw_rate_setpoint_no_mag(float gyro_z_dps);
static float calc_attitude_setpoint(uint16_t ch);
static float map_ch_ref(float ref_max, float ref_min, float ch_max, float ch_min, float ch_val);
static float wrap_180(float angle);
static float constrain_float(float x, float min_val, float max_val);
static float get_rc_yaw_normalized(uint16_t yaw_ch);

void init_setpoints(double yaw_z)
{
    RC_delta_yaw = 0;
    setpoint_data.yaw_ref = yaw_z;
    prev_yaw_z  = yaw_z;
}
static float map_ch_ref(float ref_max, float ref_min, float ch_max, float ch_min, float ch_val)
{
   return (ref_min + ((ch_val - ch_min) * (ref_max - ref_min)) / (ch_max - ch_min));
}

static float calc_attitude_setpoint(uint16_t ch)
{
    uint8_t margin = 30;
    float ch_val = (float)channels[ch];
    float  UP_LIMIT = (float )(CH_MID + margin);
    float  DOWN_LIMIT =(float)( CH_MID - margin);
    float ref_max = 15.0f;
    float  ref_min = 0;
  

    if (ch_val >= UP_LIMIT)
    { // ccw +ve rotation
         // () ? pitch: roll
        return (ch == CH_PIT ? -1.0f * map_ch_ref(ref_max, ref_min, CH_MAX, UP_LIMIT, ch_val) : map_ch_ref(ref_max, ref_min, CH_ROL_MAX, UP_LIMIT, ch_val));
    }
    else if (ch_val <= DOWN_LIMIT)
    { // cw -ve rotation
        // () ? pitch: roll
        return (ch == CH_PIT ? -1.0f * map_ch_ref(ref_min, -ref_max, DOWN_LIMIT, CH_MIN, ch_val) : map_ch_ref(ref_min, -ref_max, DOWN_LIMIT, CH_ROL_MIN, ch_val));
    }
    else if (ch_val > DOWN_LIMIT && ch_val < UP_LIMIT)
    {
        return 0; // No attitude change
    }

    return 0; // No attitude change, unknown region
}


// Wrap any angle to ±180°
static float wrap_180(float angle)
{
    angle = fmodf(angle + 180.0f, 360.0f);
    if (angle < 0.0f) angle += 360.0f;
    return angle - 180.0f;
}



static float constrain_float(float x, float min_val, float max_val)
{
    if (x > max_val)
        return max_val;

    if (x < min_val)
        return min_val;

    return x;
}

/*
 * Converts yaw PWM channel to normalized yaw command.
 *
 * Return:
 *   -1.0f = full CCW / left
 *    0.0f = center
 *   +1.0f = full CW / right
 */
static float get_rc_yaw_normalized(uint16_t yaw_ch)
{
    int32_t yaw_error_us = (int32_t)yaw_ch - (int32_t)CH_MID;

    
    // Small deadband around center to avoid noise causing yaw motion.
    if (abs(yaw_error_us) <= YAW_DEADBAND_US)
    {
        return 0.0f;
    }

    float rc_yaw;

    if (yaw_error_us > 0)
    {
        // 1500 -> 2000 maps to 0.0 -> +1.0
        rc_yaw = (float)yaw_error_us / (float)(CH_MAX - CH_MID);
    }
    else
    {
        // 1500 -> 1000 maps to 0.0 -> -1.0
        rc_yaw = (float)yaw_error_us / (float)(CH_MID - CH_MIN);
    }

    return constrain_float(rc_yaw, -1.0f, 1.0f);
}

// -----------------------------------------------------------------------------
// Heading setpoint calculation (angle hold)
// -----------------------------------------------------------------------------

// Returns the desired yaw angle in degrees (±180°).
// Call once per control loop tick.
//
// Behavior:
//   - Stick centered : hold current heading
//   - Stick right :        rotate CW  at YAW_RATE_DEG_PER_SEC
//   - Stick left  :        rotate CCW at YAW_RATE_DEG_PER_SEC
//   - Throttle below THR_LOW:     no yaw (quad on ground)

static float calc_heading_setpoint(double yaw_z,uint8_t tick_time)
{
    uint16_t throttle_ch = channels[CH_THR];
    uint16_t yaw_ch      = channels[CH_YAW];

    // Per-tick yaw increment scaled to actual loop rate
    float32_t dt = (float)(tick_time*0.001);
    float yaw_rate = YAW_RATE_DEG_PER_SEC * dt;

    // 1. Accumulate pilot yaw intent (only when airborne)
    if (throttle_ch > THR_LOW)
    {
        if (yaw_ch > CW_LIMIT)  RC_delta_yaw += yaw_rate;
        if (yaw_ch < CCW_LIMIT) RC_delta_yaw -= yaw_rate;
    }

    // 2. Correct delta for actual yaw motion BEFORE computing setpoint
    //    This prevents RC_delta_yaw from double-counting real rotation,
    //    and removes the one-frame lag present in the original code.
    RC_delta_yaw -= (yaw_z - prev_yaw_z);
    prev_yaw_z    = yaw_z;

    // 3. Compute desired heading
    float yaw_ref = RC_delta_yaw + yaw_z;

    // 4. Wrap to ±180° — handles any magnitude, not just ±360°
    yaw_ref = wrap_180(yaw_ref);

    return yaw_ref;
}

static float yaw_setpoint(float gyro_z_dps, float yaw_now, uint8_t tick_time)
{
    #if defined(USE_MAGNETOMETER)
        // Magnetometer available:
        // you can use heading hold.
        return calc_heading_setpoint(yaw_now,tick_time);
    #else
        // No magnetometer:
        // use yaw rate control only.
        return yaw_rate_setpoint_no_mag(gyro_z_dps);
    #endif
}

static float yaw_rate_setpoint_no_mag(float gyro_z_dps)
{
    float yaw_rate_setpoint = 0.0f;
    uint16_t throttle_ch = channels[CH_THR];
    uint16_t yaw_ch      = channels[CH_YAW];

    //If throttle is low, disable yaw control.
    if (throttle_ch <= THR_LOW)
    {
        yaw_rate_setpoint = 0.0f;
        return yaw_rate_setpoint;
    }

    float rc_yaw = get_rc_yaw_normalized(yaw_ch);

    if (fabsf(rc_yaw) > YAW_DEADBAND)
    {
        // Pilot commands yaw rate.
        yaw_rate_setpoint = rc_yaw * YAW_RATE_DEG_PER_SEC;
    }
    else
    {
        // Stick centered:
        // command zero yaw rate.
        yaw_rate_setpoint = 0.0f;
    }

    return yaw_rate_setpoint;
}


setpoint_t update_setpoints(double gyro_z,double yaw_z,uint8_t tick_time)
{

    float max_thurst = MAX_THROTTLE;
    float min_thrust = MIN_THROTTLE;


    setpoint_data.throttle_z = map_ch_ref(max_thurst, min_thrust, CH_MAX, CH_MIN, channels[CH_THR]); // thrust setpoint
    setpoint_data.roll_ref = calc_attitude_setpoint(CH_ROL);                                         // for desired roll
    setpoint_data.pitch_ref = calc_attitude_setpoint(CH_PIT);                                        // for desired pitch
    setpoint_data.yaw_ref =  yaw_setpoint(gyro_z, yaw_z,tick_time); // yaw ssetpoint

    return setpoint_data;
}