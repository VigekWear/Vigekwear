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
#include "stdbool.h"
#include <string.h>
#include "system_info_storage.h"
#include "flash_ctrl.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "xprintf.h"

/*---------------------- storage region -------------------------*/

/*---------------------- system power ---------------------------*/
volatile uint8_t system_battery_level    = 0;
volatile bool    system_charging_state     = false;

/*---------------------- system local time ----------------------*/
volatile uint32_t SYSTEM_LOCAL_TIMING = 0;
volatile uint32_t SYSTEM_NO_SYNC_TIMING = 0;

/*---------------------- data in ram  ---------------------------*/
uint8_t pedo_data[2048];

/*---------------------- system version -------------------------*/
static uint16_t system_app_version = 0x0100;      // version 1.0

/*---------------------- step count storage struct --------------*/
struct pedo_struct
{
    uint32_t sto_pointer;           // pointer to the data that will be storaged
    bool slice_handle;              // whether a time slice is operating
    
    uint32_t diff_pointer[20];      // keep pedo_data relative time
    uint8_t  diff_count;            // relative times
} pedo_struct;
struct pedo_struct pedo_ctrl;

/*-----------------app setting area and bootloader address-------*/
uint32_t system_parameters_settings[256] __attribute__((at(SYSTEM_PARAMETERS_SETTINGS_ADDRESS))) __attribute__((used));      
uint32_t bootloader_start_address __attribute__((at(NRF_UICR_BOOT_START_ADDRESS)));



/*----------------------system work mode------------------------*/
volatile uint8_t SYSTEM_WORK_MODE  = 0x00;
volatile bool SYSTEM_WORK_MODE_SWITCH = false; 

/*------------------------system sleep countdown----------------*/
volatile uint32_t SYSTEM_SLEEP_COUNTDOWN = 0;

 


/*------------------------- system parameters init -------------*/
void pedo_data_ctrl_init(void);

void sys_parameters_init()
{
    system_battery_level  = 0;
    system_charging_state = false;

    SYSTEM_LOCAL_TIMING   = 0;
    SYSTEM_NO_SYNC_TIMING = 0;

    memset(pedo_data, 0, 2048);

    system_app_version    = 0x0100;      // version 1.0

    pedo_data_ctrl_init();

    SYSTEM_WORK_MODE      = SYSTEM_DEFAULT_MODE;

    SYSTEM_SLEEP_COUNTDOWN   = 0;
}



/*------------------- Judge whether it is a standard time ------*/
/* true:  relative time
 * false: absolute time
 * offset from year:2000/0/0/0:0
 */
bool standard_time_judge(void)
{
    if(SYSTEM_LOCAL_TIMING > 0xA39376E1598000)  // 2000/0/0/0
        return true;

    return false;
}

/*-------------------------- return relative time---------------*/
uint32_t get_no_sync_time(void)
{
    return SYSTEM_NO_SYNC_TIMING;
}

/*--------------------------storage relative time---------------*/
void sto_no_sync_time(void)
{
    SYSTEM_NO_SYNC_TIMING = SYSTEM_LOCAL_TIMING;
}

/*-------------------------- return standard time ----------------*/
uint32_t get_system_time(void)
{
    return SYSTEM_LOCAL_TIMING;
}

/*-------------------------- update standard time -----------------*/
void system_time_update(void)
{
    SYSTEM_LOCAL_TIMING++;
}

/*-------------------------- sync update the time ------------------*/
void set_system_time(uint32_t std_time)
{
    SYSTEM_LOCAL_TIMING = std_time;
}

/*-------------------------- return system countdown time -----------*/
uint32_t get_sleep_countdown(void)
{
    return SYSTEM_SLEEP_COUNTDOWN;
}

/*-------------------------- update system countdown time-------------*/
void system_countdown_update(void)
{
    SYSTEM_SLEEP_COUNTDOWN++;
}

/*-------------------------- clear system countdown time -------------*/
void clear_sleep_countdown(void)
{
    SYSTEM_SLEEP_COUNTDOWN = 0;
}

/*-------------------------- return system version --------------------*/
uint16_t get_system_app_version(void)
{
    return system_app_version;
}

/*--------------------------- set system power info---------------------*/
void set_battery_level(uint8_t level)
{
    system_battery_level = level;
}

/*--------------------------- return system power info------------------*/
uint8_t get_battery_level(void)
{
    return system_battery_level;
}

/*--------------------------- set sys charge state ---------------------*/
void set_charging_level(bool state)
{
    system_charging_state = state;
}

/*--------------------------- return sys charge state ------------------*/
bool get_charging_level(void)
{
    return system_charging_state;
}
   
/*--------------------------- switch sys work mode ----------------------*/
void work_mode_switch(void)
{
    switch(SYSTEM_WORK_MODE)
    {
        case SYSTEM_PEDOMETER_MODE :
            SYSTEM_WORK_MODE = SYSTEM_USER_SLEEP_MODE;
        break;
                
        case SYSTEM_USER_SLEEP_MODE:
             SYSTEM_WORK_MODE = SYSTEM_BLE_WORK_MODE;
        break;

        case SYSTEM_BLE_WORK_MODE:
             SYSTEM_WORK_MODE = SYSTEM_PEDOMETER_MODE;
        break;
     }
}

/*--------------------------- return sys work mode ------------------*/
uint8_t get_work_mode(void)
{
    return SYSTEM_WORK_MODE;
}

/*--------------------------- init sys work mode ---------------------*/
void work_mode_init(void)
{
    SYSTEM_WORK_MODE = SYSTEM_DEFAULT_MODE;
}

/*--------------------------- return sys work switch state -----------*/
bool get_work_switch_state(void)
{
    return SYSTEM_WORK_MODE_SWITCH;
}

/*--------------------------- set sys work switch state----------------*/
void work_switch_set(void)
{
    SYSTEM_WORK_MODE_SWITCH = true;
}

/*--------------------------- clear sys work witch state ---------------*/
void work_switch_clear(void)
{
    SYSTEM_WORK_MODE_SWITCH = false;
}

/*--------------------------- read back disable ------------------------*/
void read_back_disable(void)
{
     NRF_UICR->RBPCONF = UICR_RBPCONF_PALL_Disabled << UICR_RBPCONF_PALL_Pos;
}

/*--------------------------- check the bootloader of mcu ---------------*/
void mcu_bootloader_check(void)
{    
    if(bootloader_start_address == BOOTLOADER_ADDRESS_DEFAULT)
    {                                                
        read_back_disable();
        flash_word_write((uint32_t*)NRF_UICR_BOOT_START_ADDRESS, BOOTLOADER_ADDRESS);                               
        flash_specword_write((uint32_t*)SYSTEM_PARAMETERS_SETTINGS_ADDRESS, SYSTEM_APP_VALID_OFFSET, SYSTEM_APP_VALID);
        
        xprintf("switch bootloader to 0x28000, begin to reset.\r\n");
        NVIC_SystemReset();
    }
    
    xprintf("system app start work...\r\n");
}


/************************************************************************************/
/*--------------------------- step count data handle -------------------------------*/
/************************************************************************************/

/*--------------------------- step count data ctrl init ----------------------------*/
void pedo_data_ctrl_init(void)
{
    pedo_ctrl.sto_pointer = 0;   
    pedo_ctrl.slice_handle = false;
    
    pedo_ctrl.diff_count = 0;
    memset(pedo_ctrl.diff_pointer, 0, 20);
    
    memset(pedo_data, 0, 2048);
}

/*--------------------------- return current slice's state ---------------------*/
bool get_pedo_slice_handle_state(void)
{
    return pedo_ctrl.slice_handle;
}

/*--------------------------- the length of pedo data --------------------------*/
uint32_t get_pedo_data_length(void)
{
    return pedo_ctrl.sto_pointer;
}


/*--------------------------- erase the data ------------------------------------*/
void pedo_data_sto_ctrl_erase(void)
{
    pedo_data_ctrl_init();
}

/*--------------------------- sport data storage handle --------------------------*/
void pedo_data_sto_handle(uint16_t pedo_time, uint16_t pedo_peak, uint16_t pedo_valley)
{   
    uint32_t the_value;
    uint8_t  i;
    if(pedo_ctrl.slice_handle == false)
    {   // new sport slice

        pedo_ctrl.slice_handle = true;

        if(standard_time_judge() == true)
        {
            // standard time
            pedo_data[pedo_ctrl.sto_pointer++] = SYSTEM_STANDARD_TIME;
        }
        else
        {   
            pedo_data[pedo_ctrl.sto_pointer++] = SYSTEM_RELATIVE_TIME;
            
            // reserved space for relative time
            pedo_ctrl.diff_pointer[pedo_ctrl.diff_count++] = pedo_ctrl.sto_pointer;
            pedo_ctrl.sto_pointer += 4;
        }
        
        // sport start time
        pedo_data[pedo_ctrl.sto_pointer++] = SYSTEM_SLICE_TIME_START;
        the_value = get_system_time();
        for(i = 0; i < 4; i++)
        {
            pedo_data[pedo_ctrl.sto_pointer++] = (the_value >> i*8) & 0xff;
        }
        
    }
    
    // sport node storage
    pedo_data[pedo_ctrl.sto_pointer++] = SYSTEM_PEDO_START;
    the_value = pedo_time;
    for(i = 0; i < 2; i++)
    {
        pedo_data[pedo_ctrl.sto_pointer++] = (the_value >> i*8) & 0xff;
    }
    the_value = pedo_peak;
    for(i = 0; i < 2; i++)
    {
        pedo_data[pedo_ctrl.sto_pointer++] = (the_value >> i*8) & 0xff;
    }
    the_value = pedo_valley;
    for(i = 0; i < 2; i++)
    {
        pedo_data[pedo_ctrl.sto_pointer++] = (the_value >> i*8) & 0xff;
    }
   
}

/*--------------------------- sport slice finish handle ----------------------*/
void pedo_data_end_handle(void)
{
    uint32_t the_value;
    uint8_t  i;
    
    pedo_data[pedo_ctrl.sto_pointer++] = SYSTEM_SLICE_TIME_STOP;
    the_value = get_system_time();
    for(i = 0; i < 4; i++)
    {
        pedo_data[pedo_ctrl.sto_pointer++] = (the_value >> i*8) & 0xff;
    }    
    
    pedo_ctrl.slice_handle = false;     // slice finish handle
}

/*--------------------------- sport time calibration --------------------------*/
void pedo_time_diff_sto_handle(void)
{
    uint32_t time_diff;
    uint8_t i;
    // relative time slice
    if(pedo_ctrl.diff_count > 0)
    {
        time_diff = SYSTEM_LOCAL_TIMING - SYSTEM_NO_SYNC_TIMING;
        
        for(i = 0; i < pedo_ctrl.diff_count; i++)
        {
            pedo_data[pedo_ctrl.diff_pointer[i] + 0] = (time_diff >> 0 ) & 0xff;
            pedo_data[pedo_ctrl.diff_pointer[i] + 1] = (time_diff >> 8 ) & 0xff;
            pedo_data[pedo_ctrl.diff_pointer[i] + 2] = (time_diff >> 16) & 0xff;
            pedo_data[pedo_ctrl.diff_pointer[i] + 3] = (time_diff >> 24) & 0xff;
        }
    }
}

/*--------------------------- get sport data ------------------------------------*/
/*----- data: storage space£¬offset: data addr length:data's length -------------*/
void get_pedo_data_handle(uint8_t *data, uint32_t *offset, uint8_t *length)
{
    uint8_t i;
 
    if( (get_pedo_data_length() - (*offset)) >= 19)
    {
        for(i = 0; i < 20; i++)
        {
            data[i] = pedo_data[*offset];
            *offset += 1;
        }

        *length = 20;                
    }
    else
    {
        *length =  get_pedo_data_length() - *offset;
        
        for(i = 0; *offset < get_pedo_data_length(); *offset += 1, i++ )
        {
            data[i] = pedo_data[*offset];            
        }
    } 

   
}
