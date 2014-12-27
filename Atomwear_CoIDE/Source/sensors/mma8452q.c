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
#include "mma8452q.h"
#include "math.h"


byte SlaveAddressIIC = MMA8452_IIC_ADDRESS;

/*********************************************************\
* IIC Write Register
\*********************************************************/
void IIC_RegWrite(byte address, byte reg, byte val)
{
	byte send[2];
	
	address = (address<<1) & 0xFE;
	send[0] = reg; 
	send[1] = val;
	
	twi_master_transfer(address, send, 2, TWI_ISSUE_STOP);
}

/*********************************************************\
* IIC Read Register
\*********************************************************/
byte IIC_RegRead(byte address, byte reg)
{
	byte b;
	
	address = (address<<1) & 0xFE;
	twi_master_transfer(address, &reg, 1, TWI_DONT_ISSUE_STOP); // write
	
	address = address | 0x01;
	twi_master_transfer(address, &b, 1, TWI_ISSUE_STOP); // read
	
	return b;
}

/*********************************************************\
* IIC Read Register
\*********************************************************/
void IIC_RegReadN(byte address, byte reg, uint8_t len, uint8_t *value)
{
	
	address = (address<<1) & 0xFE;
	twi_master_transfer(address, &reg, 1, TWI_DONT_ISSUE_STOP); // write
	
	address = address | 0x01;
	twi_master_transfer(address, value, len, TWI_ISSUE_STOP); // read
}

/*********************************************************\
* Put MMA845xQ into Active Mode
\*********************************************************/
void MMA845x_Active ()
{
  IIC_RegWrite(SlaveAddressIIC, CTRL_REG1, (IIC_RegRead(SlaveAddressIIC, CTRL_REG1) | ACTIVE_MASK));
}

/*********************************************************\
* Put MMA845xQ into Standby Mode
\*********************************************************/
void MMA845x_Standby (void)
{
  byte n;
  /*
  **  Read current value of System Control 1 Register.
  **  Put sensor into Standby Mode.
  **  Return with previous value of System Control 1 Register.
  */
  n = IIC_RegRead(SlaveAddressIIC, CTRL_REG1);
  IIC_RegWrite(SlaveAddressIIC, CTRL_REG1, n & ~ACTIVE_MASK);
}

/*********************************************************\
* Initialize MMA845xQ
\*********************************************************/
void MMA845x_Init (void)
{
	if(IIC_RegRead(SlaveAddressIIC, WHO_AM_I_REG) == 0x2A)
	{
		xprintf("reading MMA8452Q IC successful...\r\n");
	}
	else
	{
		xprintf("Not found the MMA8452Q IC...\r\n");
		xprintf("%d\r\n", IIC_RegRead(SlaveAddressIIC, WHO_AM_I_REG));
		return;
	}	
	
  MMA845x_Standby();
  /*
  **  Configure sensor for:
  **    - Sleep Mode Poll Rate of 50Hz (20ms)
  **    - System Output Data Rate of 200Hz (5ms)
  **    - Full Scale of +/-2g
  */
//   IIC_RegWrite(SlaveAddressIIC, CTRL_REG1, ASLP_RATE_20MS+DATA_RATE_5MS);
//   IIC_RegWrite(SlaveAddressIIC, XYZ_DATA_CFG_REG, FULL_SCALE_2G);
	IIC_RegWrite(SlaveAddressIIC, CTRL_REG1, 0x14);		//System Output Data Rate:200HZ
	IIC_RegWrite(SlaveAddressIIC, CTRL_REG2, 0x12);		//
	IIC_RegWrite(SlaveAddressIIC, CTRL_REG3, 0x00);
	IIC_RegWrite(SlaveAddressIIC, CTRL_REG4, 0x00);		//close interrupt
	IIC_RegWrite(SlaveAddressIIC, CTRL_REG5, 0xBD);
	IIC_RegWrite(SlaveAddressIIC, XYZ_DATA_CFG_REG, FULL_SCALE_2G);
}

int16_t numberConvert(byte msb, byte lsb)
{
	int16_t acce;
	
	acce = msb;
	acce <<= 4;
	acce += (lsb>>4);
	
	if(acce & 0x0800)
    {
        acce = ~acce;
        acce &= ~0xf000;    
        acce++;
        acce = -acce;
        return acce;	  
    }
    else
    {
        return acce;
    }	
}



uint16_t mma8452q_get_mo_value(void)
{
    int16_t x_value, y_value, z_value;
	byte value[6];
    
    uint32_t m_acce = 0;

    if(IIC_RegRead(SlaveAddressIIC, STATUS_00_REG)&0x08)
	{
		IIC_RegReadN(SlaveAddressIIC, OUT_X_MSB_REG, 6, value);

		x_value = numberConvert(value[0], value[1]);
		y_value = numberConvert(value[2], value[3]);
		z_value = numberConvert(value[4], value[5]);
    
        m_acce = 0;
        m_acce += x_value*x_value;
        m_acce += y_value*y_value;
        m_acce += z_value*z_value;
        
        return (uint16_t)sqrt(m_acce);
    }
    
    else
        return 0;
}


void mma8452q_test(void)
{
	int16_t x_value, y_value, z_value;
	byte value[6];
	
	MMA845x_Init();
	MMA845x_Active();	
    
	while(1)
	{
		if(IIC_RegRead(SlaveAddressIIC, STATUS_00_REG)&0x08)
		{
			IIC_RegReadN(SlaveAddressIIC, OUT_X_MSB_REG, 6, value);

			x_value = numberConvert(value[0], value[1]);
			y_value = numberConvert(value[2], value[3]);
			z_value = numberConvert(value[4], value[5]);
			
	// 		xprintf("%d,%d,%d,%d,%d,%d\r\n", value[0], value[1], value[2], value[3], value[4], value[5]);
			xprintf("%d,%d,%d\r\n", x_value, y_value, z_value);
			nrf_delay_ms(1000);
		}
	}
}
