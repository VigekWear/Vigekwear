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
 
#include "stdint.h"
#include "twi_master.h"
#include "nrf_delay.h"
#include "xprintf.h"
#include <math.h>
#include "bmi055.h"
#include "mma8452q.h"


void acce_init()
{
	if(IIC_RegRead(ACCE_IIC_ADDRESS, 0x00) == 0xFA)
		xprintf("reading ACCE IC succeseful...\r\n");
	else
	{
		xprintf("Not found the ACCE IC...\r\n");
		return;
	}
	IIC_RegWrite(ACCE_IIC_ADDRESS, 0x20, 0x00);
}	

void gyro_init()
{
	if(IIC_RegRead(GYRO_IIC_ADDRESS, 0x00) == 0x0F)
		xprintf("reading GYRO IC succeseful...\r\n");
	else
	{
		xprintf("Not found the GYRO IC...\r\n");
		return;
	}
	IIC_RegWrite(GYRO_IIC_ADDRESS, 0x16, 0x00);
}

void bmi055_init()
{
	acce_init();
	gyro_init();
}

