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
 
#ifndef _FLASH_CTRL_H
#define _FLASH_CTRL_H

void flash_word_write(uint32_t *address, uint32_t value);

void flash_specword_write(uint32_t *address, uint8_t offset, uint32_t data);

#endif
