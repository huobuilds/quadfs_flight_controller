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

#include "i2c.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

void config_12c1_node(uint8_t saddr, uint8_t mem_addr, uint8_t data, uint8_t data_len)
{

    HAL_I2C_Mem_Write(&hi2c1,
                             saddr,
                             mem_addr,
                             I2C_MEMADD_SIZE_8BIT,
                             &data,
                             data_len,
                             1);//short timeout under our 3ms loop, else might hold up the control loop
}


uint8_t i2c1_read(uint8_t saddr, uint8_t maddr, uint8_t data_len, uint8_t *buffer)
{

    uint8_t status = HAL_I2C_Mem_Read(&hi2c1,
                        saddr,
                        maddr,
                        I2C_MEMADD_SIZE_8BIT,
                        buffer,
                        data_len,
                        1);//short timeout under our 3ms loop, else might hold up the control loop

    return (HAL_OK == status);
}
