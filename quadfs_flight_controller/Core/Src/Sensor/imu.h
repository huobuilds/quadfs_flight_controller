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
#ifndef IMU_H
#define IMU_H
#include <stdbool.h>
#define PWR_MGMT_1 0x6B		  // Power Management. Typical values:0x00(run mode)
#define ADDRESS_AD0_LOW (0x68<<1)  // address pin AD0 to  low (GND), default for InvenSense evaluation board
#define ADDRESS_AD0_HIGH (0x69<<1) // address pin AD0 to  high (VCC)
#define DEFAULT_ADDRESS GYRO_ADDRESS
//  identity of MPU9255 is 0x73 , MPU9250 is 0x71, 0x70 for MPU6500, 0x68 for mpu6050
#define WHO_AM_I_VAL_6050 0x68	 
#define WHO_AM_I_VAL_6500 0x70	
#define WHO_AM_I_VAL_9250 0x71	
#define WHO_AM_I_VAL_MAG 0x48 //  MPU9255 or MPU9250
#define WHO_AM_I 0x75		  // identity of the device
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define SMPLRT_DIV 0x19	  // Sample Rate Divider. Typical values:0x07for (125Hz) sample rate , ie (1KHz/(1 + 0x07)) internal sample rate
#define CONFIG 0x1A		  // Low Pass Filter.Typical values:0x06(5Hz)
#define GYRO_CONFIG 0x1B  // Gyro Full Scale Select. Typical values:0x10(1000dps)
#define ACCEL_CONFIG 0x1C // Accel Full Scale Select. Typical values:0x01(2g)
#define ACCEL_CONFIG2 0x1D // set cutoff freq for low pass filter for acc.

// MAG:AK8963
#define AK8963_ADDRESS  (0x0C << 1) // Address of magnetometer
#define WHO_AM_I_MAG 0x00	// (AKA WIA) should return 0x48
#define INFO 0x01
#define MAG_ST1 0x02 // data ready status bit 0
#define MAG_XOUT_L 0x03
#define MAG_XOUT_H 0x04
#define MAG_YOUT_L 0x05
#define MAG_YOUT_H 0x06
#define MAG_ZOUT_L 0x07
#define MAG_ZOUT_H 0x08

#define MAG_ST2 0x09					// Data overflow bit 3 and data read error status bit 2
#define MAG_CNTL 0x0A					// Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define MAG_ASTC 0x0C					// Self test control
#define MAG_I2CDIS 0x0F					// I2C disable
#define MAG_ASAX 0x10					// Fuse ROM x-axis sensitivity adjustment value
#define MAG_ASAY 0x11					// Fuse ROM y-axis sensitivity adjustment value
#define MAG_ASAZ 0x12					// Fuse ROM z-axis sensitivity adjustment value
#define mRes 10.0f * 4912.0f / 32760.0f // 16-bit magnetometer resolution |  16-bit (0.15 µT/LSB)


typedef struct imu
{
    float accXn;
    float accYn;
    float accZn;
    float gyroXn;
    float gyroYn;
    float gyroZn;
    float  gyroXnf;
    float  gyroYnf;
    float  gyroZnf;
    float magXn;
    float magYn;
    float magZn;
} imu_t;

//scale = 2/(max-min)
//offset = (max+min)/2
//for mpu9250
#define ACC_Z_OFFSET_G  -0.0988f
#define ACC_Z_SCALE_G   0.9781f
#define ACC_Y_OFFSET_G  0.0739f
#define ACC_Y_SCALE_G   0.9975f
#define ACC_X_OFFSET_G  0.11675f
#define ACC_X_SCALE_G   0.9966f
// Note: calc. offset and scale for any new accelerometer sensor

void init_mpu(void);
imu_t update_mpu_imu();
#endif 