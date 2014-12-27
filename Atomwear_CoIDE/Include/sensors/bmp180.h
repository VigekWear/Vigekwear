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
 
#ifndef __BMP180_H__
#define __BMP180_H__

#define bmp180_calc_temperature(ut)\
		bmp180_get_temperature(ut)

#define bmp180_calc_pressure(up)\
		bmp180_get_pressure(up)

#define bmp180_read_ut()\
		bmp180_get_ut()

#define bmp180_read_up()\
		bmp180_get_up()

#define bmp180_read_reg(address, data, length)\
		bmp180_get_reg(address, data, length)

#define bmp180_write_reg(address, data, length)\
		bmp180_set_reg(address, data, length)

#define bmp180_read_cal_param()\
		bmp180_get_cal_param()

/* Define for used read and write macros
 * Define the calling convention of YOUR bus communication routine.
 * note This includes types of parameters. This example shows the
 * configuration for an SPI bus link.*/

/* defines the return parameter type of the BMP180_WR_FUNCTION */
#define BMP180_BUS_WR_RETURN_TYPE char

/* defines the calling parameter types of the BMP180_WR_FUNCTION */
#define BMP180_BUS_WR_PARAM_TYPES unsigned char, unsigned char,\
			unsigned char *, unsigned char

/* links the order of parameters defined in BMP180_BUS_WR_PARAM_TYPE
 * to function calls used inside the API */
#define BMP180_BUS_WR_PARAM_ORDER (device_addr, register_addr,\
			register_data, write_length)

/* never change this line  */
#define BMP180_BUS_WRITE_FUNC(device_addr, register_addr,\
			register_data, write_length)\
	bus_write(device_addr, register_addr, register_data, write_length)

/* defines the return parameter type of the BMP180_WR_FUNCTION */
#define BMP180_BUS_RD_RETURN_TYPE char

/* defines the calling parameter types of the BMP180_WR_FUNCTION */
#define BMP180_BUS_RD_PARAM_TYPES unsigned char, unsigned char,\
			unsigned char *, unsigned char

/* links the order of parameters defined in BMP180_BUS_WR_PARAM_TYPE
 * to function calls used inside the API*/
#define BMP180_BUS_RD_PARAM_ORDER (device_addr, register_addr,\
			register_data, read_length)

/* never change this line */
#define BMP180_BUS_READ_FUNC(device_addr, register_addr,\
			register_data, read_length)\
		bus_read(device_addr, register_addr, register_data, read_length)

/*        CHIP_TYPE CONSTANTS */
#define BMP180_CHIP_ID                      0x55
#define BOSCH_PRESSURE_BMP180   85

/*        BMP180 I2C Address */
#define BMP180_I2C_ADDR         (0xEE>>1)

/*        SMB380 API error codes */
#define E_BMP_NULL_PTR                      (char)  (-127)
#define E_BMP_COMM_RES                     (char) (-1)
#define E_BMP_OUT_OF_RANGE              (char) (-2)
#define E_SENSOR_NOT_DETECTED        (char) 0

/*     register definitions     */
#define BMP180_PROM_START__ADDR         0xaa
#define BMP180_PROM_DATA__LEN             22

#define BMP180_CHIP_ID_REG                      0xD0
#define BMP180_VERSION_REG                      0xD1

#define BMP180_CTRL_MEAS_REG            0xF4
#define BMP180_ADC_OUT_MSB_REG          0xF6
#define BMP180_ADC_OUT_LSB_REG          0xF7

#define BMP180_SOFT_RESET_REG           0xE0

#define BMP180_T_MEASURE        0x2E  /* temperature measurent */
#define BMP180_P_MEASURE        0x34  /* pressure measurement */

#define BMP180_TEMP_CONVERSION_TIME  5 /* TO be spec'd by GL or SB */

#define PARAM_MG      3038        /*calibration parameter */
#define PARAM_MH     -7357        /*calibration parameter */
#define PARAM_MI      3791        /*calibration parameter */

/* register write and read delays */

#define BMP180_MDELAY_DATA_TYPE unsigned int
#define BMP180_MDELAY_RETURN_TYPE  void
/* this structure holds all device specific calibration parameters */
struct bmp180_calibration_param_t{
   short ac1;
   short ac2;
   short ac3;
   unsigned short ac4;
   unsigned short ac5;
   unsigned short ac6;
   short b1;
   short b2;
   short mb;
   short mc;
   short md;
};
/* BMP180 image registers data structure */
struct bmp180_t {
   struct bmp180_calibration_param_t cal_param;
   unsigned char mode;
   unsigned char chip_id, ml_version, al_version;
   unsigned char dev_addr;
   unsigned char sensortype;

   long param_b5;
   int number_of_samples;
   short oversampling_setting;
   short sw_oss;
   BMP180_BUS_WR_RETURN_TYPE(*bus_write)(BMP180_BUS_WR_PARAM_TYPES);
   BMP180_BUS_RD_RETURN_TYPE(*bus_read)(BMP180_BUS_RD_PARAM_TYPES);
   BMP180_MDELAY_RETURN_TYPE(*delay_msec)(BMP180_MDELAY_DATA_TYPE);
};

typedef struct bmp180_env_value
{
	short 	temperature;
	long	press;
	float	altitude;
}bmp180_env_value;

/* bit slice positions in registers */

#define BMP180_CHIP_ID__POS             0
#define BMP180_CHIP_ID__MSK             0xFF
#define BMP180_CHIP_ID__LEN             8
#define BMP180_CHIP_ID__REG             BMP180_CHIP_ID_REG

#define BMP180_ML_VERSION__POS          0
#define BMP180_ML_VERSION__LEN          4
#define BMP180_ML_VERSION__MSK          0x0F
#define BMP180_ML_VERSION__REG          BMP180_VERSION_REG

#define BMP180_AL_VERSION__POS          4
#define BMP180_AL_VERSION__LEN          4
#define BMP180_AL_VERSION__MSK          0xF0
#define BMP180_AL_VERSION__REG          BMP180_VERSION_REG

/* DATA REGISTERS
 * LG/HG thresholds are in LSB and depend on RANGE setting
 * no range check on threshold calculation*/
#define BMP180_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

#define BMP180_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))
/* General Setup Functions */
/* BMP180_init */

/*   input :      Pointer to bmp180_t
 *   output:  -
 *   return:  result of communication function
 *   notes : */

int bmp180_init(struct bmp180_t *);

short bmp180_get_temperature(unsigned long ut);

long bmp180_get_pressure(unsigned long up);

unsigned short bmp180_get_ut(void);
unsigned long  bmp180_get_up(void);

/* MISC RAW functions */

/* read: address, data-pointer, length */
char bmp180_get_reg(unsigned char , unsigned char *, unsigned char);

/* write: address, data-pointer, length */
char bmp180_set_reg(unsigned char , unsigned char*, unsigned char);

/* API internal helper functions */
int bmp180_get_cal_param(void);

void bmp180_init_interface(void);
bmp180_env_value* get_environment_value(void);
void bmp180_test(void);
short get_temperature(void);
long	get_press(void);
float	get_altitude(void);

#endif   /* __BMP180_H__ */
