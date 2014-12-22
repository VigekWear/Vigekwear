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
#include "bmp180.h"
#include "math.h"
struct bmp180_t *p_bmp180;          /**< pointer to BMP180 device area */

struct bmp180_t bmp180;

bmp180_env_value *bmp180_env_result;
bmp180_env_value bmp180_result;
   


/** initialize BMP180

  This function initializes the BMP180 pressure sensor.
  The function automatically detects the sensor type and stores this for all
  future communication and calculation steps
  \param *bmp180_t pointer to bmp180 device data structure
  \return result of communication routines */

int bmp180_init(struct bmp180_t *bmp180)
{
   char comres = 0;
   unsigned char data;

   p_bmp180 = bmp180;                   /* assign BMP180 ptr */
   p_bmp180->sensortype = E_SENSOR_NOT_DETECTED;
   p_bmp180->dev_addr = BMP180_I2C_ADDR;   /* preset BMP180 I2C_addr */
  
   p_bmp180->bus_write = IIC_write;
   p_bmp180->bus_read = IIC_read;
   p_bmp180->delay_msec = nrf_delay_ms;
    
   comres += p_bmp180->BMP180_BUS_READ_FUNC(p_bmp180->dev_addr,\
		BMP180_CHIP_ID__REG, &data, 1);  /* read Chip Id */

   p_bmp180->chip_id = BMP180_GET_BITSLICE(data, BMP180_CHIP_ID);
   p_bmp180->number_of_samples = 1;
   p_bmp180->oversampling_setting = 0;
   p_bmp180->sw_oss = 0;
	if (p_bmp180->chip_id == BMP180_CHIP_ID) {
		/* get bitslice */
		p_bmp180->sensortype = BOSCH_PRESSURE_BMP180;
		/* read Version reg */
		comres += p_bmp180->BMP180_BUS_READ_FUNC(p_bmp180->dev_addr,\
		BMP180_VERSION_REG, &data, 1);

		/* get ML Version */
		p_bmp180->ml_version = BMP180_GET_BITSLICE(data,\
		BMP180_ML_VERSION);
		/* get AL Version */
		p_bmp180->al_version = BMP180_GET_BITSLICE(data,\
		BMP180_AL_VERSION);
        xprintf("reading BMP180 IC successful...\r\n");
		bmp180_get_cal_param(); /*readout bmp180 calibparam structure*/
	}
    else
    {
        xprintf("Not found the BMP180 IC...\r\n");
    }
	bmp180_env_result = &bmp180_result;//(bmp180_env_value *)malloc(sizeof(bmp180_env_value));
   return comres;
}
/** read out parameters cal_param from BMP180 memory
   \return result of communication routines*/

int bmp180_get_cal_param(void)
{
   int comres;
   unsigned char data[22];
   comres = p_bmp180->BMP180_BUS_READ_FUNC(p_bmp180->dev_addr,\
	BMP180_PROM_START__ADDR, data, BMP180_PROM_DATA__LEN);

   /*parameters AC1-AC6*/
   p_bmp180->cal_param.ac1 =  (data[0] << 8) | data[1];
   p_bmp180->cal_param.ac2 =  (data[2] << 8) | data[3];
   p_bmp180->cal_param.ac3 =  (data[4] << 8) | data[5];
   p_bmp180->cal_param.ac4 =  (data[6] << 8) | data[7];
   p_bmp180->cal_param.ac5 =  (data[8] << 8) | data[9];
   p_bmp180->cal_param.ac6 = (data[10] << 8) | data[11];

   /*parameters B1,B2*/
   p_bmp180->cal_param.b1 =  (data[12] << 8) | data[13];
   p_bmp180->cal_param.b2 =  (data[14] << 8) | data[15];

   /*parameters MB,MC,MD*/
   p_bmp180->cal_param.mb =  (data[16] << 8) | data[17];
   p_bmp180->cal_param.mc =  (data[18] << 8) | data[19];
   p_bmp180->cal_param.md =  (data[20] << 8) | data[21];
   return comres;
}
/** calculate temperature from ut
  ut was read from the device via I2C and fed into the
  right calc path for BMP180
  \param ut parameter ut read from device
  \return temperature in steps of 0.1 deg celsius
  \see bmp180_read_ut()*/

short bmp180_get_temperature(unsigned long ut)
{
   short temperature;
   long x1, x2;
	if (p_bmp180->sensortype == BOSCH_PRESSURE_BMP180) {
		x1 = (((long) ut - (long) p_bmp180->cal_param.ac6) * \
		(long) p_bmp180->cal_param.ac5) >> 15;
		x2 = ((long) p_bmp180->cal_param.mc << 11) / \
		(x1 + p_bmp180->cal_param.md);
		p_bmp180->param_b5 = x1 + x2;
	}
   temperature = ((p_bmp180->param_b5 + 8) >> 4);  /* temperature in 0.1 deg C*/
   return temperature;
}
/** calculate pressure from up
  up was read from the device via I2C and fed into the
  right calc path for BMP180
  In case of BMP180 averaging is done through oversampling by the sensor IC

  \param ut parameter ut read from device
  \return temperature in steps of 1.0 Pa
  \see bmp180_read_up()*/

long bmp180_get_pressure(unsigned long up)
{
   long pressure, x1, x2, x3, b3, b6;
   unsigned long b4, b7;

   b6 = p_bmp180->param_b5 - 4000;
   /*****calculate B3************/
   x1 = (b6*b6) >> 12;
   x1 *= p_bmp180->cal_param.b2;
   x1 >>= 11;

   x2 = (p_bmp180->cal_param.ac2*b6);
   x2 >>= 11;

   x3 = x1 + x2;

   b3 = (((((long)p_bmp180->cal_param.ac1)*4 + x3) << \
		p_bmp180->oversampling_setting)+2) >> 2;

   /*****calculate B4************/
   x1 = (p_bmp180->cal_param.ac3 * b6) >> 13;
   x2 = (p_bmp180->cal_param.b1 * ((b6*b6) >> 12)) >> 16;
   x3 = ((x1 + x2) + 2) >> 2;
   b4 = (p_bmp180->cal_param.ac4 * (unsigned long) (x3 + 32768)) >> 15;

   b7 = ((unsigned long)(up - b3) * (50000>>p_bmp180->oversampling_setting));
	if (b7 < 0x80000000)
		pressure = (b7 << 1) / b4;
	else
		pressure = (b7 / b4) << 1;

   x1 = pressure >> 8;
   x1 *= x1;
   x1 = (x1 * PARAM_MG) >> 16;
   x2 = (pressure * PARAM_MH) >> 16;
   pressure += (x1 + x2 + PARAM_MI) >> 4;/* pressure in Pa*/
   return pressure;
}
/** read out ut for temperature conversion
   \return ut parameter that represents the uncompensated
    temperature sensors conversion value*/

unsigned short bmp180_get_ut()
{
   unsigned short ut;
   unsigned char data[2];
   unsigned char ctrl_reg_data;
   int wait_time;
   int comres;
	if (p_bmp180->chip_id == BMP180_CHIP_ID  /* get bitslice */) {
		ctrl_reg_data = BMP180_T_MEASURE;
		wait_time = BMP180_TEMP_CONVERSION_TIME;
	}
   comres = p_bmp180->BMP180_BUS_WRITE_FUNC(p_bmp180->dev_addr,\
		BMP180_CTRL_MEAS_REG, &ctrl_reg_data, 1);

   p_bmp180->delay_msec(wait_time);
   comres += p_bmp180->BMP180_BUS_READ_FUNC(p_bmp180->dev_addr,\
		BMP180_ADC_OUT_MSB_REG, data, 2);
   ut = ( data[0] << 8) | data[1];
   return ut;
}
/** read out up for pressure conversion
  depending on the oversampling ratio setting up can be 16 to 19 bit
   \return up parameter that represents the uncompensated pressure value*/

unsigned long bmp180_get_up()
{
   int j;         /* j included for loop*/
   unsigned long up = 0;
   unsigned long sum = 0; /* get the calculated pressure data*/
   unsigned char data[3];
   unsigned char ctrl_reg_data;
   int comres = 0;
   
	if (p_bmp180->chip_id == BMP180_CHIP_ID && p_bmp180->sw_oss == 1 && \
		p_bmp180->oversampling_setting == 3) {
		for (j = 0 ; j < 3; j++) 
         {
            /* 3 times getting pressure data*/				
            ctrl_reg_data = BMP180_P_MEASURE + (p_bmp180->oversampling_setting << 6);
			comres = p_bmp180->BMP180_BUS_WRITE_FUNC(p_bmp180->dev_addr, BMP180_CTRL_MEAS_REG, &ctrl_reg_data, 1);
			p_bmp180->delay_msec(2 + (3 << (p_bmp180->oversampling_setting)));
			comres += p_bmp180->BMP180_BUS_READ_FUNC(p_bmp180->dev_addr, BMP180_ADC_OUT_MSB_REG, data, 3);
			sum = (((unsigned long) data[0] << 16) |((unsigned long) data[1] << 8) |(unsigned long) data[2]) >> (8-p_bmp180->oversampling_setting);
			p_bmp180->number_of_samples = 1;
			up = up + sum;  /*add up with dummy var*/
        }
		up = up / 3;   /* averaging*/
	}
  else   
   {
	if (p_bmp180->chip_id == BMP180_CHIP_ID && p_bmp180->sw_oss == 0) 
     {
		ctrl_reg_data = BMP180_P_MEASURE + (p_bmp180->oversampling_setting << 6);
		comres = p_bmp180->BMP180_BUS_WRITE_FUNC(p_bmp180->dev_addr,BMP180_CTRL_MEAS_REG, &ctrl_reg_data, 1);
		p_bmp180->delay_msec(2 + (3 << (p_bmp180->oversampling_setting)));
		comres += p_bmp180->BMP180_BUS_READ_FUNC(p_bmp180->dev_addr,BMP180_ADC_OUT_MSB_REG, data, 3);
		up = (((unsigned long) data[0] << 16) |((unsigned long) data[1] << 8) |(unsigned long) data[2]) >> (8-p_bmp180->oversampling_setting);
		p_bmp180->number_of_samples = 1;
	}
  }
return up;
}

void bmp180_init_interface(void)
{
	bmp180_init(&bmp180); 
}

bmp180_env_value* get_environment_value()
{
	unsigned short  ut;
    unsigned long   up;
	
	ut = bmp180_get_ut();
    up = bmp180_get_up();
	
	ut = bmp180_get_temperature(ut);
	if(ut < 500)
		bmp180_env_result->temperature = ut;

	bmp180_env_result->press = bmp180_get_pressure(up);
	
	bmp180_env_result->altitude = ((float)bmp180_env_result->press) / 101325;
    bmp180_env_result->altitude = 44330.0*(1.0 - powf(bmp180_env_result->altitude,1.0/5.255));
	
	return bmp180_env_result;
// 	xprintf("ut:%d up:%d temperature:%d pressure:%d altitude:%d\r\n", ut, up, *temp, *press, (uint32_t)*alti);
}

short get_temperature()
{
	return bmp180_env_result->temperature;
}

long get_press()
{
	return bmp180_env_result->press;
}

float get_altitude()
{
	return bmp180_env_result->altitude;
}


void bmp180_test()
{
    unsigned short  ut;
    unsigned long   up;
    short temperature;
    long pressure;
    float altitude;
    
    bmp180_init(&bmp180);   
   
    ut = up = temperature = pressure = 0;
    

    while(1)
    {
        ut = bmp180_get_ut();
        up = bmp180_get_up();
      
        temperature = bmp180_get_temperature(ut);
        pressure = bmp180_get_pressure(up);
        
        altitude = ((float)pressure)/101325;
        altitude = 44330.0*(1.0 - powf(altitude,1.0/5.255));

        xprintf("ut:%d up:%d temperature:%d pressure:%d altitude:%d\r\n", ut, up, temperature, pressure, (uint32_t)altitude);
    }
}

