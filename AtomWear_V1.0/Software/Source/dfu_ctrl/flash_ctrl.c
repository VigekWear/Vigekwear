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
#include "nrf.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "flash_ctrl.h"
#include "ble_startup.h"
#include "xprintf.h"
#include "dfu_ctrl.h"

uint16_t crc16_compute(uint8_t * p_data, uint32_t size)
{
    uint32_t i;
    uint16_t crc = 0xffff;

    for (i = 0; i < size; i++)
    {
        crc = (unsigned char)(crc >> 8) | (crc << 8);
        crc ^= p_data[i];
        crc ^= (unsigned char)(crc & 0xff) >> 4;
        crc ^= (crc << 8) << 4;
        crc ^= ((crc & 0xff) << 4) << 1;
    }
    
    return crc;
}


/** @brief Function for erasing a page in flash.
 *
 * @param page_address Address of the first word in the page to be erased.
 */
void flash_page_erase(uint32_t *page_address)
{
  // Turn on flash erase enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
    
    // Erase page:
    NRF_NVMC->ERASEPAGE = (uint32_t)page_address;
    
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
    
    // Turn off flash erase enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
    
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
}


void system_app_pages_erase(uint32_t page_address, uint8_t pages_count)
{
    uint8_t i;
    // Turn on flash erase enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    
    for(i = 0; i < pages_count; i++)
    {
        // Erase page:
        NRF_NVMC->ERASEPAGE = page_address;     
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
        
        page_address += 1024;
    }
    
    // Turn off flash erase enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);  
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
}


/** @brief Function for filling a page in flash with a value.
 *
 * @param[in] address Address of the first word in the page to be filled.
 * @param[in] value Value to be written to flash.
 */
void flash_word_write(uint32_t *address, uint32_t value)
{
    // Turn on flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);
    
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
  
    *address = value;
  
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
  
    // Turn off flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
  
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
}


void flash_specword_write(uint32_t *address, uint8_t offset, uint32_t data)
{
    uint32_t backup[256];
    uint32_t i;
    
    for(i = 0; i < 256; i++ )
    {
        backup[i] = *(address + i);
    }
    
    flash_page_erase(address);
    
    backup[offset] = data;
    
    for(i = 0; i < 256; i++ )
    {
        flash_word_write((address + i), backup[i]);
    }
       
}



/** @brief Function for filling a page in flash with a value.
 *
 * @param[in] address Address of the first word in the page to be filled.
 * @param[in] value Value to be written to flash.
 */
uint8_t system_page_refresh(uint8_t cur_page, uint8_t page_data[], uint32_t data_length)
{
    uint32_t byte_count, tmp32, *address, word_value;
    uint16_t tmp16, CRC = 0;
    uint8_t j, cur_state = DFU_ERROR;
    
    for(j = 2; j > 0; j--)   // 5: 0 1 2 3 4
     {
        tmp16 = page_data[data_length - j]; 
        CRC |= (tmp16 << (2 - j)*8 );
    }
    data_length -=2;
    
    // Turn on flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);    
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
    
    
    byte_count = 0;
    word_value = 0;
    address = (uint32_t *)(CODE_REGION_1_START + cur_page*CODE_PAGE_SIZE);
   
    while(1)
    {
        for(j = 0; j < 4; j++)
        {
            if(byte_count <= data_length )
            {
                tmp32 = page_data[byte_count++];
                word_value |= tmp32 << j*8 ;
            }
        }
                
        *address = word_value;  
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
        
        word_value = 0;
        address += 1;
        
        if( (cur_page*CODE_PAGE_SIZE + byte_count) == get_dfu_amount())
        {
            cur_state = DFU_END;
            break;
        }
        else if( byte_count == data_length)
        {
           cur_state = DFU_SUCC;
           break; 
        }
        else
        {
            
        }
    }
  
    // Turn off flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy);

    address = (uint32_t *)(CODE_REGION_1_START + cur_page*CODE_PAGE_SIZE);
    
    for(byte_count = 0; byte_count <= data_length; )
    {
        tmp32 = *address;
       
        for(j = 0; j < 4; j++)
            page_data[byte_count++] = (tmp32 >> j*8 ) & 0xff;
        
        address +=1;                              
    }
  

    tmp16 = crc16_compute(page_data, data_length);
    xprintf("CRC is %d, %d \r\n", CRC, tmp16);
    if( CRC == tmp16)
        return cur_state;
    else
        return DFU_ERROR;    
}



/** @} */

