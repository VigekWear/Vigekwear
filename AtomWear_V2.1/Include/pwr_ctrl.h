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
 
#ifndef _PWR_CTRL_H_
#define _PWR_CTRL_H_

#include "stdbool.h"

bool battery_self_detect(void);

void system_on_sleep(void);

void battery_detect_start(void);

bool battery_is_available(void);

void battery_detect_stop(void);

uint8_t get_battery_value(void);

void system_low_battery(void);

#endif 
