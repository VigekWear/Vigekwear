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
#include "math.h"



char mpu6050_write(uint8_t device_addr, uint8_t register_addr, uint8_t *register_data, uint8_t write_length)
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


char mpu6050_read(uint8_t device_addr, uint8_t register_addr, uint8_t *register_data, uint8_t read_length)
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


void mpu6050_init()
{
    uint8_t send_rec[2];
    mpu6050_read(MPU6050_ADDR0, WHO_AM_I, send_rec, 1);
    if(send_rec[0] != 0x68)
    {
        xprintf("mpu6050 address error!\r\n");
    }
    xprintf("mpu6050 ID:%X \r\n", send_rec[0]);
    
    send_rec[0] = 0x00;
    mpu6050_write(MPU6050_ADDR0, PWR_MGMT_1, send_rec, 1);  
                                 // 0x6B
    send_rec[0] = 0x07;
    mpu6050_write(MPU6050_ADDR0, SMPRT_DIV, send_rec, 1);  
                                 // 0x19  
    send_rec[0] = 0x06;
    mpu6050_write(MPU6050_ADDR0, CONFIG, send_rec, 1);  //  
                                 // 0x1A
    send_rec[0] = 0x18;
    mpu6050_write(MPU6050_ADDR0, GYRO_CFG, send_rec, 1);  // 000 11 000
                                 // 0x1B
    send_rec[0] = 0x08;
    mpu6050_write(MPU6050_ADDR0, ACCEL_CFG, send_rec, 1);  // 000 11 000  4g
                                 // 0x1C
}


mpu6050_temp mpu6050_get_temp()
{
    uint8_t send_rec[2];
    short temp;
    mpu6050_read(MPU6050_ADDR0, TEMP_OUT_H, send_rec, 2);
    temp = (send_rec[0] << 8) | send_rec[0];
    
    return ((temp*1.0)/340 + 36.53);
}


mpu6050_accel mpu6050_get_accel()
{
    uint8_t send_rec[6];
    mpu6050_accel accel;
    mpu6050_read(MPU6050_ADDR0, ACCEL_XOUT_H, send_rec, 6);
    accel.X = (uint16_t)((send_rec[0] << 8) | send_rec[1]);
    accel.Y = (uint16_t)((send_rec[2] << 8) | send_rec[3]);
    accel.Z = (uint16_t)((send_rec[4] << 8) | send_rec[5]);
    return accel;
}

mpu6050_gyro mpu6050_get_gyro()
{
    uint8_t send_rec[6];
    mpu6050_gyro gyro;
    mpu6050_read(MPU6050_ADDR0, GYRO_XOUT_H, send_rec, 6);
    gyro.X = (uint16_t)((send_rec[0] << 8) | send_rec[1]);
    gyro.Y = (uint16_t)((send_rec[2] << 8) | send_rec[3]);
    gyro.Z = (uint16_t)((send_rec[4] << 8) | send_rec[5]);
    return gyro;
}


void mpu6050_test()
{
    uint16_t temp;
    mpu6050_gyro gyro;
    mpu6050_accel accel;
    
    mpu6050_init();    
    while(1)
    {
        temp  = (uint16_t)mpu6050_get_temp();
        accel = mpu6050_get_accel();
        gyro  = mpu6050_get_gyro();
        
        //xprintf("the temperature is:%d\r\n", temp);
        //xprintf("accle_X:%d accel_Y:%d  accel_Z:%d\r\n", accel.X, accel.Y, accel.Z);
        xprintf("gyro_X:%d gyro_Y:%d  gyro_Z:%d\r\n", gyro.X, gyro.Y, gyro.Z);
        nrf_delay_ms(200);
    }
}

    
    
    
    
   

