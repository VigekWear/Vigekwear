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
 
#ifndef  _OLED_H_
#define  _OLED_H_

#include "ak8963.h"

#define high 1
#define low  0 
#define W_Data 0x5C
//
//
//

#define STATE_MAX 0xFF
#define STATE_MIN 0x00
#define STATE_55 0x55
#define STATE_AA 0xAA
#define START_PAGE 0xB0
#define PAGE_TOTAL 4
#define START_HIGH_BIT 0x12
#define START_LOW_BIT 0x00
#define FRAME_HIGH_ROW 0x01
#define FRAME_LOW_ROW 0x80

#define OLED_MIN 0
#define COLUMN_MAX 64
#define ROW_MAX 32
//
//
#define	Slave_Address 0x78		 
#define	OP_Command 0x00
#define	OP_Data 0x40
//
//
#define Manual 1
#define Auto 0
//
//
#define PEDO_IMAGE_1 		0
#define PEDO_IMAGE_2 		1
#define TEMP_IMAGE			2
/**********************************************/
//
//
typedef unsigned int   uint;
typedef unsigned char  uchar;
typedef unsigned long  ulong;

void OLED_Clear(void);
void OLED_DrawPoint(uint8_t, uint8_t, uint8_t);
void OLED_ShowImg(uint8_t x, uint8_t y, uint8_t chr, uint8_t mode);
void OLED_ShowChar(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size);
void OLED_ShowString(uint8_t x,uint8_t y,const uint8_t *p);
void OLED_Refresh_Gram(void);
void OLED_Init(void);
void update_battery(uint8_t);
void oled_test(void);

void display_step_count(void);
void oled_default_mode(void);
void oled_pedo_mode(void);
void oled_temp_mode(void);
void oled_cmps_mode(void);
void oled_update_step_count(void);
void oled_update_temp(void);
void oled_drawline(int16_t );
void oled_update_cmps(ak8963_cmps *);
void oled_DFU_mode(void);

//
//

#endif
