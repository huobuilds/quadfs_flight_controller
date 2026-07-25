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
#include "imu.h"
#include <stdio.h>
#include "stm32f411xe.h"
#include <stdlib.h>
#include "arm_math.h" // ARM::CMSIS:DSP
#include "i2c.h"
#include "quadfs_def.h"
#include <math.h>
#include "stm32f4xx_hal.h"
#include "median_filter.h"



#define M_PI 3.14159265358979323846 /* pi */

#define ACCEL_4g 0x08        // +-4g
#define ACCEL_8g 0x10        // +-8g
#define GYRO_500dps 0x08     //+-500dps
#define GYRO_1000dps  0x10   //(1000dps)
#define RUN_MODE 0x00        // run mode
#define CUTOFF_HZ 0x06       // DLPF cutoff freq of %5Hz cutoff
#define UPDATE_HZ 0x07       // update sensor reg. at 125Hz
#define MAG_PWR_DOWN 0x00    //// Power down magnetometer
#define ROM_ACCESS_MODE 0x0F ////Enter Fuse ROM access mode
#define MAG_RES 0x16         // Set magnetometer data resolution to 16bits and mode to continuous mode 2
#define BYPASS_REG 0x37
#define BYPASS_EN 0x02


float ASAX, ASAY, ASAZ;
static imu_t imu_data;

static char msg[100];
static void gyro_calib(void);
static void init();
static uint8_t read_accelerometer();
static uint8_t read_accelerometer_calib();
static uint8_t read_gyro();
static void read_magnetometer();
static float convert_MAG_ASA_(int16_t ASA_);
static void init_acc_gyro();
static void init_magnetometer();
int len;
int8_t acc_gyro_id = 0;
int8_t mag_id = 0;

extern I2C_HandleTypeDef hi2c1;


float gyroXnf;
float gyroYnf;
float gyroZnf;
static float roll_rate_buffer[5]  = {0.0f};
static float pitch_rate_buffer[5] = {0.0f};
static float yaw_rate_buffer[5]   = {0.0f};
static uint8_t rate_buf_idx = 0U;


float gx_offset = 0;
float gy_offset = 0;
float gz_offset = 0;
float ASAX, ASAY, ASAZ;
static imu_t imu_data;

static char msg[100];
static void gyro_calib(void);
static void init();
static uint8_t read_accelerometer();
static uint8_t read_gyro();
static void read_magnetometer();
static float convert_MAG_ASA_(int16_t ASA_);
static void init_acc_gyro();
static void init_magnetometer();
uint16_t who_i_am_acc_gyro = 0;

static void init_acc_gyro()
{
     while ((who_i_am_acc_gyro != WHO_AM_I_VAL_9250)&&
            (who_i_am_acc_gyro != WHO_AM_I_VAL_6050)&&
            (who_i_am_acc_gyro != WHO_AM_I_VAL_6500))
    {
        uint8_t read_buffer[1];
        if (i2c1_read(ADDRESS_AD0_LOW, WHO_AM_I, 1, read_buffer))
        {
           who_i_am_acc_gyro = read_buffer[0];
        }

        len = snprintf(msg,
					sizeof(msg),
					"MPU ID %d, Found\r\n",who_i_am_acc_gyro);

        CDC_Transmit_FS((uint8_t *)msg, (uint16_t)len);
    }

    //config works good
    config_12c1_node(ADDRESS_AD0_LOW, ACCEL_CONFIG, ACCEL_4g, 1);   
    config_12c1_node(ADDRESS_AD0_LOW, GYRO_CONFIG, GYRO_500dps, 1); 
    config_12c1_node(ADDRESS_AD0_LOW, CONFIG,6, 1);     //   6 for DLPF cutoff freq of %5Hz cutoff gyro
    config_12c1_node(ADDRESS_AD0_LOW, PWR_MGMT_1, RUN_MODE, 1);     // run mode
    config_12c1_node(ADDRESS_AD0_LOW, SMPLRT_DIV, 0, 1);    // update sensor reg. 0 for 1kHz
}

static void init_magnetometer()
{

    #if defined(USE_MAGNETOMETER)

        if(who_i_am_acc_gyro == WHO_AM_I_VAL_6050) return;
        if(who_i_am_acc_gyro == WHO_AM_I_VAL_6500) return;

        int16_t ASAXD, ASAYD, ASAZD;
        

        // Start MAG config
        // pg 29 mpu9255 datasheet, pg 23 mpu9255 spec
        config_12c1_node(ADDRESS_AD0_LOW, BYPASS_REG, BYPASS_EN, 1); // BYPASS_EN to setup MAG

        int who_i_am = 0;
        do
        {
            uint8_t read_buffer[1];
            if (i2c1_read(AK8963_ADDRESS, WHO_AM_I_MAG, 1, read_buffer))
            {
                who_i_am = read_buffer[0];
            }
            
            len = snprintf(msg,
                        sizeof(msg),
                        "MPU mag ID %d, Found\r\n", who_i_am);

            CDC_Transmit_FS((uint8_t *)msg, (uint16_t)len);
        } while (who_i_am != WHO_AM_I_VAL_MAG);


        config_12c1_node(AK8963_ADDRESS, MAG_CNTL, MAG_PWR_DOWN, 1);    //// Power down magnetometer
        config_12c1_node(AK8963_ADDRESS, MAG_CNTL, ROM_ACCESS_MODE, 1); ////Enter Fuse ROM access mode

        uint8_t read_buffer[3];
        i2c1_read(AK8963_ADDRESS , MAG_ASAX, 3,read_buffer);
        ASAXD = read_buffer[0];
        ASAYD = read_buffer[1];
        ASAZD = read_buffer[2];

        // mpu datasheet pg 53
        ASAX = convert_MAG_ASA_(ASAXD);
        ASAY = convert_MAG_ASA_(ASAYD);
        ASAZ = convert_MAG_ASA_(ASAZD);

        config_12c1_node(AK8963_ADDRESS, MAG_CNTL, MAG_PWR_DOWN, 1); // Power down magnetometer
        config_12c1_node(AK8963_ADDRESS, MAG_CNTL, MAG_RES, 1);      // Set magnetometer data resolution to 16bits and mode to continuous mode 2

    #else
    //do nothing
    #endif

}
static void init()
{
    init_acc_gyro();
    init_magnetometer();
    gyro_calib();
}
static float convert_MAG_ASA_(int16_t ASA_)
{
    return ((((ASA_ - 128) * 0.5f) / 128.0f) + 1.0f);
}

void init_mpu(void)
{
    init();
}

static void gyro_calib(void)
{
    // Gyro Offset calibration at every power cycle,
    // make sure drone is stationary during gyro calibration.
    int counter = 0;
    float counter_limit = 2000.0f;
    int16_t gyroXD, gyroYD, gyroZD;
    uint8_t read_buffer[6];
    
    gx_offset = 0;
    gy_offset = 0;
    gz_offset = 0;

    while (counter < counter_limit)
    {
        if (i2c1_read(ADDRESS_AD0_LOW, GYRO_XOUT_H, 6, read_buffer))
        {
            gyroXD = read_buffer[0] << 8 | read_buffer[1];
            gyroYD = read_buffer[2] << 8 | read_buffer[3];
            gyroZD = read_buffer[4] << 8 | read_buffer[5];

            gx_offset += gyroXD;
            gy_offset += gyroYD;
            gz_offset += gyroZD;
            counter++;
        }
        HAL_Delay(2); // wait before next read.
    }
    gx_offset = gx_offset / counter_limit;
    gy_offset = gy_offset / counter_limit;
    gz_offset = gz_offset / counter_limit;
}

static uint8_t read_accelerometer()
{
    int16_t accXD, accYD, accZD;
    float sensitivity_4g = 8192.0f;
    float sensitivity_8g = 4096.0f;
    uint8_t read_buffer[6];

    if (i2c1_read(ADDRESS_AD0_LOW, ACCEL_XOUT_H, 6, read_buffer))
    {
        accXD = read_buffer[0] << 8 | read_buffer[1];
        accYD = read_buffer[2] << 8 | read_buffer[3];
        accZD = read_buffer[4] << 8 | read_buffer[5];

        float accXn = (float)(((accXD) / sensitivity_4g));
        float accYn = (float)(((accYD) / sensitivity_4g));
        float accZn = (float)(((accZD) / sensitivity_4g));

        /* Board convention: x fwd, y left, z up.
        drone: Roll + = right side down, Pitch + = nose up. At rest: accZn = +1. */

        imu_data.accXn =  (accXn - ACC_X_OFFSET_G)*ACC_X_SCALE_G;
        imu_data.accYn =  (accYn - ACC_Y_OFFSET_G)*ACC_Y_SCALE_G;
        imu_data.accZn =  (accZn - ACC_Z_OFFSET_G)*ACC_Z_SCALE_G;

        return 1;
    }
    return 0;
}

static uint8_t read_gyro()
{

    int16_t gyroXD, gyroYD, gyroZD;
    float sensitivity_500dps = 65.5f;
    float sensitivity_1000dps = 32.8f;
    uint8_t read_buffer[6];
    if (i2c1_read(ADDRESS_AD0_LOW, GYRO_XOUT_H, 6, read_buffer))
    {
        gyroXD = read_buffer[0] << 8 | read_buffer[1];
        gyroYD = read_buffer[2] << 8 | read_buffer[3];
        gyroZD = read_buffer[4] << 8 | read_buffer[5];

        /* Board convention: x fwd, y left, z up.
        drone: Roll + = right side down, Pitch + = nose up. At rest: accZn = +1. */

        // convert of deg/s,
        imu_data.gyroXn = ((float)(gyroXD - gx_offset) / sensitivity_500dps);
        imu_data.gyroYn = ((float)(gyroYD - gy_offset) / sensitivity_500dps);
        imu_data.gyroZn = ((float)(gyroZD - gz_offset) / sensitivity_500dps);

        roll_rate_buffer[rate_buf_idx]  = imu_data.gyroXn;
        pitch_rate_buffer[rate_buf_idx] = imu_data.gyroYn;
        yaw_rate_buffer[rate_buf_idx]   = imu_data.gyroZn;
        rate_buf_idx = (rate_buf_idx + 1U) % 5U;

        imu_data.gyroXnf =  median_filter(roll_rate_buffer);
        imu_data.gyroYnf = -median_filter(pitch_rate_buffer); // sign flipped for pid use
        imu_data.gyroZnf = -median_filter(yaw_rate_buffer); // sign flipped for pid use

        return 1;
    }
    return 0;
}

static void read_magnetometer()
{
    #if defined(USE_MAGNETOMETER)

         if((who_i_am_acc_gyro == WHO_AM_I_VAL_6050)||(who_i_am_acc_gyro == WHO_AM_I_VAL_6500)) 
        {
            //no Mag, simulate Mag to avoid math error in ekf
            imu_data.magXn = 18.5f;
            imu_data.magYn = 2.0f;
            imu_data.magZn = 44.0f;
            return;
        }
        
        int16_t magXD, magYD, magZD;
        float magxr, magyr, magzr;
        uint8_t read_buffer[7];

        i2c1_read((AK8963_ADDRESS ), MAG_ST1, 1,read_buffer);
        uint8_t st1 = read_buffer[0];

        if (st1 & 0x01)
        { // check for magnetometer data ready bit to be set
            i2c1_read((AK8963_ADDRESS), MAG_XOUT_L, 7,read_buffer);
            magXD = read_buffer[0] | read_buffer[1] << 8;
            magYD = read_buffer[2] | read_buffer[3] << 8;
            magZD = read_buffer[4] | read_buffer[5] << 8;
            uint8_t st2 = read_buffer[6];

            // validate data
            if (!(st2 & 0x08))
            { // Check HOFL flag in ST2[3]
                // raw
                magxr = magXD * 0.15f * ASAX;
                magyr = magYD * 0.15f * ASAY;
                magzr = magZD * 0.15f * ASAZ;

                // calibrated
                imu_data.magXn = magxr - 2.9423f;
                imu_data.magYn = -1.0f*(magyr - 33.2415f);
                imu_data.magZn = -1.0f*(magzr + 59.7366f);
                                
                /* Board convention: x fwd, y left, z up.
                drone: Roll + = right side down, Pitch + = nose up. At rest: accZn = +1. */

            }
        }
    #else
        //no Mag, simulate Mag to avoid math error in ekf
        imu_data.magXn = 18.5f;
        imu_data.magYn = 2.0f;
        imu_data.magZn = 44.0f;
    #endif


}
imu_t update_mpu_imu()
{
    read_accelerometer();
    read_gyro();
    read_magnetometer();
    return imu_data;
}