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
#include "mpu6050.h"
#include <math.h>
#include "hmc5883l.h"



char hmc5883l_write(uint8_t device_addr, uint8_t register_addr, uint8_t *register_data, uint8_t write_length)
{
    uint8_t send_rec[10], i;
    if( (write_length > 9) || (write_length < 1) )
    {
        xprintf("write length error.\r\n");
        return false;
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
        return true;
    }
}


char hmc5883l_read(uint8_t device_addr, uint8_t register_addr, uint8_t *register_data, uint8_t read_length)
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


void hmc5883l_init()
{
    uint8_t send_rec[2];
    hmc5883l_read(HMC5883L_ADDR, HMC5883L_IDEN_A, send_rec, 1);
    if(send_rec[0] != 0x48)
    {
        xprintf("hmc5883l address error:%X %X %X !\r\n", HMC5883L_ADDR, HMC5883L_IDEN_A, send_rec[0]);
        while(1);
    }
    xprintf("hmc5883l IDEN_A:%X \r\n", send_rec[0]);
    
    send_rec[0] = 0x70;
    hmc5883l_write(HMC5883L_ADDR, HMC5883L_CRA, send_rec, 1);  // 8-average, 15 Hz default, normal measurement
    
    send_rec[0] = 0x00;
    hmc5883l_write(HMC5883L_ADDR, HMC5883L_CRB, send_rec, 1);  // Gain = 0, or any other desired gain
    
    send_rec[0] = 0x00;
    hmc5883l_write(HMC5883L_ADDR, HMC5883L_MODE, send_rec, 1);  // Continuous-measurement mode
    
    nrf_delay_ms(67);   
}

hmc5883l_cmps hmc5883l_get_cmps()
{
    uint8_t send_rec[6];
    hmc5883l_cmps cmps;
    hmc5883l_read(HMC5883L_ADDR, HMC5883L_XMSB, send_rec, 6);
    cmps.X = (uint16_t)((send_rec[0] << 8) | send_rec[1]);
    cmps.Z = (uint16_t)((send_rec[2] << 8) | send_rec[3]);
    cmps.Y = (uint16_t)((send_rec[4] << 8) | send_rec[5]);
    
    if((cmps.X == 0) && (cmps.Y == 0))
    {
        cmps.Angle = 0xFFFF;        
    }
    else
    {
        //cmps.Angle = atan(1.1);
        cmps.Angle = (180*atan2( cmps.Y, cmps.X)) / 3.14 ;
    }
    
    return cmps;
}

void hmc5883l_test()
{
    hmc5883l_cmps cmps;
    hmc5883l_init();
    while(1)
    {
        cmps = hmc5883l_get_cmps();
        xprintf("cmps_X:%d cmp_Y:%d cmps_Z:%d cmps_Angle:%d \r\n", cmps.X, cmps.Y, cmps.Z, cmps.Angle);
        nrf_delay_ms(200); 
    }
}

