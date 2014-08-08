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
 
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpio.h"

 
void GPIO_LoToHi_INT_config(uint32_t pin);

//void nRF_gpio_INT_config(uint32_t nRF_gpio_INT_pin);

void TIMER1_OPEN(void);

void TIMER1_INT_CLOSE(void);

void TIMER1_INT_OPEN(void);

void timer2_init(void);
