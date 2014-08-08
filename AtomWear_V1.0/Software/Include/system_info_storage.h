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
 
#ifndef _SYSTEM_INFO_STORAGE_H_
#define _SYSTEM_INFO_STORAGE_H_
#include "stdbool.h"

/*--------------------- system work mode -----------------------------*/
#define SYSTEM_DEFAULT_MODE                0x01
#define SYSTEM_PEDOMETER_MODE              0x01
#define SYSTEM_USER_SLEEP_MODE             0x02
#define SYSTEM_BLE_WORK_MODE               0x03
#define SYSTEM_MCU_SLEEP_MODE              0x04
#define SYSTEM_POWER_SHUTDOWN_MODE         0x05
            
/*-------------- system data collect and synchronous -----------------*/
#define SYSTEM_STANDARD_TIME               0x01
#define SYSTEM_SET_ALARM                   0x02
#define SYSTEM_BATTERY_SYNC                0x03
#define SYSTEM_PEDO_START                  0x04
#define SYSTEM_USER_SLEEP_SYNC             0x05
#define SYSTEM_SLICE_TIME_START            0x06
#define SYSTEM_SLICE_TIME_STOP             0x07
#define SYSTEM_RESPONSE                    0x08
#define SYSTEM_DATA_SYNC_END               0x09
#define SYSTEM_DATA_SYNC_AMOUNT            0x0A
#define SYSTEM_DATA_SYNC                   0x0B
#define SYSTEM_RELATIVE_TIME               0x0C
#define SYSTEM_APP_UPDATE                  0x0D
#define SYSTEM_APP_VERSION                 0x11

/*----------------- Addr/Sector/SYS State param -----------------------*/
#define CODE_PAGE_SIZE                     1024
#define BOOTLOADER_ADDRESS                 0x00028000
#define BOOTLOADER_ADDRESS_DEFAULT         0xFFFFFFFF
#define SYSTEM_PARAMETERS_SETTINGS_ADDRESS 0x0003FC00
#define NRF_UICR_BOOT_START_ADDRESS        (NRF_UICR_BASE + 0x14)

#define SYSTEM_APP_VALID_OFFSET            0
#define SYSTEM_APP_VALID                   0x00000001
#define SYSTEM_APP_INVALID                 0xFFFFFFFF

#define SYSTEM_SLEEP_DEADLINE              300 

/*------------------------- sys param init ----------------------------*/
void sys_parameters_init(void);

/*------------------------- judge system standard time ----------------*/
bool standard_time_judge(void);

/*-------------------------- get relative time ------------------------*/
uint32_t get_no_sync_time(void);

/*-------------------------- storage relative time --------------------*/
void sto_no_sync_time(void);

/*-------------------------- get standard time ------------------------*/
uint32_t get_system_time(void);

/*-------------------------- update standard time ---------------------*/
void system_time_update(void);

/*-------------------------- sync update standard time ----------------*/
void set_system_time(uint32_t std_time);

/*-------------------------- return sys countdown time ----------------*/
uint32_t get_sleep_countdown(void);

/*-------------------------- update sys countdown time ----------------*/
void system_countdown_update(void);

/*-------------------------- clear sys countdown time -----------------*/
void clear_sleep_countdown(void);

/*-------------------------- return system version --------------------*/
uint16_t get_system_app_version(void);

/*--------------------------- set sys power info ----------------------*/
void set_battery_level(uint8_t level);

/*--------------------------- return sys power info -------------------*/
uint8_t get_battery_level(void);

/*--------------------------- set sys charging state ------------------*/
void set_charging_level(bool state);

/*--------------------------- return sys charging state ---------------*/
bool get_charging_level(void);
   
/*--------------------------- switch sys work mode --------------------*/
void work_mode_switch(void);

/*--------------------------- get cur sys work mode -------------------*/
uint8_t get_work_mode(void);

/*--------------------------- init sys work mode ----------------------*/
void work_mode_init(void);  

/*--------------------------- return sys work switch mode -------------*/
bool get_work_switch_state(void);

/*--------------------------- set sys work switch mode ----------------*/
void work_switch_set(void);

/*--------------------------- clear sys work switch mode --------------*/
void work_switch_clear(void);
   
/*--------------------------- check sys's bootloader ------------------*/
void mcu_bootloader_check(void); 
   
   
/*--------------------------- sport data ctrl init ------------------------------*/
void pedo_data_ctrl_init(void);

/*--------------------------- return cur slice's state --------------------------*/
bool get_pedo_slice_handle_state(void);

/*--------------------------- the length of sport data --------------------------*/
uint32_t get_pedo_data_length(void);

/*--------------------------- erase sport data ----------------------------------*/
void pedo_data_sto_ctrl_erase(void);

/*--------------------------- sport slice storage handle ------------------------*/
void pedo_data_sto_handle(uint16_t pedo_time, uint16_t pedo_peak, uint16_t pedo_valley);

/*--------------------------- slice handle finish -------------------------------*/
void pedo_data_end_handle(void);

/*--------------------------- old sport time update -----------------------------*/
void pedo_time_diff_sto_handle(void);

/*--------------------------- get sport data ------------------------------------*/
/*----- data:storage region£¬offset:addr£¬length:data's length ------------------*/
void get_pedo_data_handle(uint8_t *data, uint32_t *offset, uint8_t *length);

#endif
