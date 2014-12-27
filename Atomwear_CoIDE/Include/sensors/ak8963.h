/* Copyright (c) 2012 Giayee Technology Co. Ltd. All Rights Reserved.
 *
 * Author: LL
 * Web: www.giayee.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
 
#ifndef _AK8963_H_
#define _AK8963_H_

#include "stdint.h"

#define AK8963_IIC_ADDR		0x0C

#define AK8963_WIA		0x00
#define AK8963_INFO		0x01
#define AK8963_ST1		0x02
#define AK8963_HXL		0x03
#define AK8963_HXH		0x04
#define AK8963_HYL		0x05
#define AK8963_HYH		0x06
#define AK8963_HZL		0x07
#define AK8963_HZH		0x08
#define AK8963_ST2		0x09

#define AK8963_CNTL1	0x0A
#define AK8963_CNTL2	0x0B
#define AK8963_ASTC		0x0C
#define AK8963_TS1		0x0D
#define AK8963_TS2		0x0E
#define AK8963_I2CDIS	0x0F

#define AK8963_ASAX		0x10
#define AK8963_ASAY		0x11
#define AK8963_ASAZ		0x12
#define AK8963_RSV		0x13

typedef struct
{
    int16_t X;
    int16_t Y;
    int16_t Z;
    int16_t Angle;
}ak8963_cmps;

typedef struct ak8963_adj_value
{
	uint8_t	x;
	uint8_t y;
	uint8_t z;
}ak8963_adj_value;


void ak8963_init(void);
void ak8963_pwr_down(void);
void ak8963_cs1_mode(void);
void self_test(void);
void ak8963_test(void);
ak8963_cmps* ak8963_get_value(void);

#endif

