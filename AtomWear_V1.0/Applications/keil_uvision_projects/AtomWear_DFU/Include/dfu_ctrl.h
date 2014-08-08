#ifndef _DFU_CTRL_H_
#define _DFU_CTRL_H

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "simple_uart.h"



#define CODE_PAGE_SIZE                     1024
#define BOOTLOADER_ADDRESS                 0x00028000
#define BLLTLOADER_ADDRESS_DEFAULT         0xFFFFFFFF
#define BOOTLOADER_SETTINGS_ADDRESS        0x0003FC00
#define NRF_UICR_BOOT_START_ADDRESS        (NRF_UICR_BASE + 0x14)

#define SYSTEM_APP_VALID_OFFSET            0
#define SYSTEM_APP_VALID                   0x00000001
#define SYSTEM_APP_DISVALID                0xFFFFFFFF

#define CODE_REGION_1_START                0x14000  
#define CODE_REGION_1_SIZE                 80






bool region1_is_valid(void);
void bootloader_app_start(void);

#endif

