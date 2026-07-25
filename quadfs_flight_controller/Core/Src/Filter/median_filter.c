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
#include "median_filter.h"

//compare and swap
#define CS(a, b) if ((a) > (b)) { float _t = (a); (a) = (b); (b) = _t; }

float median_filter(float vals[5])
{
    //5 window median filter, remove spikes
    float a=vals[0], b=vals[1], c=vals[2], d=vals[3], e=vals[4];

    CS(a,b); CS(c,d);
    CS(a,c); CS(b,d);
    CS(b,c); CS(a,e);
    CS(c,e); CS(b,e);
    CS(b,c);
    
    return c;
}