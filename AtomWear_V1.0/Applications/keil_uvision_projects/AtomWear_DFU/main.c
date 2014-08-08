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
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "simple_uart.h"
#include "xprintf.h"
#include "ble_startup.h"
#include "dfu_ctrl.h"
#include "flash_ctrl.h"
  



int main()
{
    simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, 0);
    xprintf("\n\n\n\n\n\r\n region2 start..\r\n");
       
    if(region1_is_valid())
    {
        xprintf("dump to region11111\r\n");
        bootloader_app_start();
    }
    
    system_app_pages_erase(CODE_REGION_1_START, CODE_REGION_1_SIZE);
    xprintf(" app area has eased..\r\n");

    ble_start(); 
  
    for( ; ;)
    {
        ble_rec_handle();       
        
        if(get_dfu_state() == SYSTEM_UPDATE_END)
        {            
            xprintf("system update end.\r\n");
            sd_softdevice_disable();
            
            flash_specword_write((uint32_t*)BOOTLOADER_SETTINGS_ADDRESS, SYSTEM_APP_VALID_OFFSET, SYSTEM_APP_VALID); 
            NVIC_SystemReset();  
        }
       
        if(get_dfu_state() == SYSTEM_UPDATE_FAILURE)
        {
            xprintf("system update failed.\r\n");
            NVIC_SystemReset();            
        }
    }

    
}


        

