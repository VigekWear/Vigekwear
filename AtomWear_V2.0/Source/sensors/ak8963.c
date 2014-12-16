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
#include "ak8963.h"
#include "mma8452q.h"

void ak8963_write(uint8_t device_addr, uint8_t register_addr, uint8_t *register_data, uint8_t write_length)
{
    uint8_t send_rec[10], i;
    if( (write_length > 9) || (write_length < 1) )
    {
        xprintf("write length error.\r\n");
        return;
    }
    else
    {
        device_addr = (device_addr << 1) & 0xFE;
        send_rec[0] = register_addr;
        
        for(i = 1; i <= write_length; i++)
        {
            send_rec[i] = register_data[i-1];
        }
        
        twi_master_transfer(device_addr, send_rec, write_length + 1, TWI_ISSUE_STOP); 
    }
}

uint8_t ak8963_read(uint8_t device_addr, uint8_t register_addr, uint8_t *register_data, uint8_t read_length)
{
    if(read_length < 1)
    {
        xprintf("read length error.\r\n");
        return false;
    }
    else
    {
        device_addr = (device_addr << 1) & 0xFE;
        twi_master_transfer(device_addr, &register_addr, 1, TWI_DONT_ISSUE_STOP); // write
        
        device_addr = device_addr | 0x01;
        twi_master_transfer(device_addr, register_data, read_length, TWI_ISSUE_STOP); // read
        return true;
    }
}

uint8_t ak8963_Read(uint8_t device_addr, uint8_t register_addr)
{
	uint8_t temp;
	
	device_addr = (device_addr << 1) & 0xFE;
	twi_master_transfer(device_addr, &register_addr, 1, TWI_DONT_ISSUE_STOP); // write
	
	device_addr = device_addr | 0x01;
    twi_master_transfer(device_addr, &temp, 1, TWI_ISSUE_STOP); // read
	
	return temp;
}

int16_t number_convert(uint8_t lsb, uint8_t msb, uint8_t bit)
{
	int16_t value;
	
	if(bit == 14)
	{
		value = msb;
		value <<= 8;
		value += lsb;
		
		if(value & 0x2000)
		{
			value = ~value;
			value &= ~0xc000;    
			value++;
			value = -value;
			return value;	   
		}
		else
		{
			return value;
		}
	}	
	else if(bit == 16)
	{
		value = msb;
		value <<= 8;
		value += lsb;
		
		if(value & 0x8000)
		{
			value = ~value;
			value++;
			value = -value;
			return value;	  
		}
		else
		{
			return value;
		}
	}
	else return 0;
}

void ak8963_init(void)
{
    uint8_t buffer[2];
	
    ak8963_read(AK8963_ADDR, AK8963_WIA, buffer, 1);
    if(buffer[0] != 0x48)
    {
        xprintf("AK8963_WIA address error:%X\r\n", buffer[0]);
        return;
    }
	else
		xprintf("AK8963_WIA read successful!\r\n");
    
	buffer[0] = 0x10;
    ak8963_write(AK8963_ADDR, AK8963_CNTL1, &buffer[0], 1);//AK8963_CNTL1 = 0x10;			//16bit / power down mode
	    
    nrf_delay_ms(67);   
}

void self_test(void)
{
	uint8_t temp;
	uint8_t value[6];
	int16_t x_value, y_value, z_value;
	
	ak8963_init();
	
	temp = 0x40;
	ak8963_write(AK8963_ADDR, AK8963_ASTC, &temp, 1);//AK8963_ASTC = 0x40;				//self test
	
	temp = 0x18;
	ak8963_write(AK8963_ADDR, AK8963_CNTL1, &temp, 1);//AK8963_CNTL1 = 0x18;		//self-test mode;
	
	nrf_delay_ms(10);
	while(!(ak8963_Read(AK8963_ADDR, AK8963_ST1) & 0x01));
	
	ak8963_read(AK8963_ADDR, AK8963_HXL, value, 6);
	
	temp = 0x00;
	ak8963_write(AK8963_ADDR, AK8963_ASTC, &temp, 1);//AK8963_ASTC = 0x00;				//close self test mode
	
	temp = 0x10;
	ak8963_write(AK8963_ADDR, AK8963_CNTL1, &temp, 1);//AK8963_CNTL1 = 0x10;		//power down;	
	
	x_value = number_convert(value[0], value[1], 16);
	y_value = number_convert(value[2], value[3], 16);
	z_value = number_convert(value[4], value[5], 16);
	
	xprintf("%d,%d,%d\r\n", x_value, y_value, z_value);
	
}

void ak8963_test(void)
{
	uint8_t temp;
	uint8_t value[6];
	int16_t x_value, y_value, z_value;
	
	self_test();
	
	temp = 0x12;
	ak8963_write(AK8963_ADDR, AK8963_CNTL1, &temp, 1);//AK8963_CNTL1 = 0x12; Continuous measurement mode 1; 8hz
	
	nrf_delay_ms(10);
	while(1)
	{
		do
		{
			ak8963_read(AK8963_ADDR, AK8963_ST1, &temp, 1);
		}while(!(temp & 0x01));
		
		ak8963_read(AK8963_ADDR, AK8963_HXL, value, 6);
		
		ak8963_read(AK8963_ADDR, AK8963_ST2, &temp, 1);
		
		if(!(temp & 0x08))
		{
			x_value = number_convert(value[0], value[1], 16);
			y_value = number_convert(value[2], value[3], 16);
			z_value = number_convert(value[4], value[5], 16);
		}
		
		xprintf("%d,%d,%d\r\n", x_value, y_value, z_value);
		
		nrf_delay_ms(10);
	}
}

