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
#include "ble_startup.h"
#include "dfu_ctrl.h"

bool region1_is_valid()
{
    uint32_t state = *((uint32_t *)(BOOTLOADER_SETTINGS_ADDRESS + SYSTEM_APP_VALID_OFFSET));
    if(state == SYSTEM_APP_VALID)
        return true;
    else
        return false;
}


__asm void Start_Device_Firmware_Update(uint32_t start_addr)
{
    LDR   R2, [R0]               ; Get App MSP.
    MSR   MSP, R2                ; Set the main stack pointer  to the applications MSP.
    LDR   R3, [R0, #0x00000004]  ; Get application reset vector address.
    BX    R3                     ; No return - stack code is now activated only through SVC and plain interrupts.
    ALIGN
}
 


/**@brief Function for disabling all interrupts before jumping from bootloader to application.
 */
static void interrupts_disable(void)
{
    uint32_t interrupt_setting_mask;
    uint8_t  irq;

    // We start the loop from first interrupt, i.e. interrupt 0.
    irq                    = 0;
    // Fetch the current interrupt settings.
    interrupt_setting_mask = NVIC->ISER[0];
    
    for (; irq < 32; irq++)
    {
        if (interrupt_setting_mask & (0x01 << irq))
        {
            // The interrupt was enabled, and hence disable it.
            NVIC_DisableIRQ((IRQn_Type) irq);
        }
    }        
}

void bootloader_util_app_start(uint32_t start_addr)
{
    Start_Device_Firmware_Update(start_addr);
}

void bootloader_app_start()
{
    // start the application safely.
    interrupts_disable();
    uint32_t err_code = sd_softdevice_forward_to_application();
    APP_ERROR_CHECK(err_code);
    bootloader_util_app_start(CODE_REGION_1_START);
}
