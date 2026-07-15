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
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "setpoint.h"

extern UART_HandleTypeDef huart1;
uint8_t ibus[radio_buffer] = {0};
uint8_t ibus_src[radio_buffer] = {0};
uint16_t channels[channel_on]; 
static DMA_HandleTypeDef *radio_hdma_rx = NULL;

static uint16_t calc_checksum(const uint8_t ch_data[]);
static void process_ibus(void);
static void usart_radio_dma_restart(void);

const uint16_t header = 32;

void process_ibus(void)
{
    uint8_t empty_buffer[radio_buffer] = {0};
    // 32 Max iBus packet size (2 byte header, 14 channels x 2 bytes, 2 byte checksum)
    //  TX only has 10 channels
    uint8_t ch_data[radio_buffer] = {0};

    if ((ibus[0] == header) && (ibus[1] == header * 2)) // check the header
    {
        memcpy(ch_data, ibus, sizeof(ch_data));
        if (calc_checksum(ch_data) == (ch_data[30] | (ch_data[31] << 8)))
        {
            // process iBus buffer into channels
            for (uint8_t i = 2, ch = 0; ch < channel_on; i = i + 2, ch++)
            {
                channels[ch] = ch_data[i] | (ch_data[i + 1] << 8);
            }

        }
        // reset buffer values
        memcpy(ibus, empty_buffer, sizeof(ibus));
        memcpy(ch_data, empty_buffer, sizeof(ch_data));
    }
}
static uint16_t calc_checksum(const uint8_t ch_data[])
{
    uint16_t checksum = 65535;
    for (uint8_t i = 0; i <= 29; i++)
    {
        checksum -= ch_data[i];
    }
    return checksum;
}

uint8_t update_channels(void)
{
    if (radio_hdma_rx == NULL)
    {
        return 0;
    }

    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))
    {
        HAL_UART_DMAStop(&huart1);              /* abort any stuck transfer   */
        __HAL_UART_CLEAR_OREFLAG(&huart1);      /* release the ORE/DMA lock   */
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        HAL_UART_Receive_DMA(&huart1, ibus_src, radio_buffer);
           /* Disable callbacks because we are polling manually */
        __HAL_DMA_DISABLE_IT(radio_hdma_rx, DMA_IT_HT);
        __HAL_DMA_DISABLE_IT(radio_hdma_rx, DMA_IT_TC);
       return 0;
    }

    /* Poll DMA2 Stream2 transfer-complete flag */
    if (__HAL_DMA_GET_FLAG(radio_hdma_rx, DMA_FLAG_TCIF2_6) != RESET)
    {
        /* Process copied frame */
        process_ibus();
        
        /* Restart DMA quickly for the next frame */
        usart_radio_dma_restart();
        return 1;
    }

    return 0;
}

void usart_radio_dma_start(void)
{
    radio_hdma_rx = huart1.hdmarx;

    HAL_UART_Receive_DMA(&huart1, ibus_src, radio_buffer);

    /* Disable callbacks because we are polling manually */
    __HAL_DMA_DISABLE_IT(radio_hdma_rx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(radio_hdma_rx, DMA_IT_TC);
}

static void usart_radio_dma_restart(void)
{
    if (radio_hdma_rx == NULL)
    {
        return;
    }

    /* Disable DMA stream */
    __HAL_DMA_DISABLE(radio_hdma_rx);

    while ((radio_hdma_rx->Instance->CR & DMA_SxCR_EN) != 0U)
    {
        /* wait until DMA is disabled */
    }

    /* Reload transfer size */
    radio_hdma_rx->Instance->NDTR = radio_buffer;

    /* Reload memory address */
    radio_hdma_rx->Instance->M0AR = (uint32_t)ibus;

    /* Clear DMA2 Stream2 flags */
    __HAL_DMA_CLEAR_FLAG(radio_hdma_rx,
                         DMA_FLAG_TCIF2_6  |
                         DMA_FLAG_HTIF2_6  |
                         DMA_FLAG_TEIF2_6  |
                         DMA_FLAG_DMEIF2_6 |
                         DMA_FLAG_FEIF2_6);

    /* Re-enable DMA stream */
    __HAL_DMA_ENABLE(radio_hdma_rx);

    /* Ensure USART RX DMA request is enabled */
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
}