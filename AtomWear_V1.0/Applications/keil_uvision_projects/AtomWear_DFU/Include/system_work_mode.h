#ifndef SYSTEM_WORK_MODE_H
#define SYSTEM_WORK_MODE_H

#include "stdbool.h"
#include "stdint.h"
#include "step_count.h"

// system work mode
#define SYSTEM_IDLE_MODE                   0x00
#define SYSTEM_DEFAULT_MODE                0x01
#define SYSTEM_PEDOMETER_MODE              0x01
#define SYSTEM_USER_SLEEP_MODE             0x02
#define SYSTEM_BLE_WORK_MODE               0x03
#define SYSTEM_MCU_SLEEP_MODE              0x04
#define SYSTEM_POWER_SHUTDOWN_MODE         0x05
            
// system data collect and synchronous
#define SYSTEM_STANDARD_TIME               0x1
#define SYSTEM_SET_ALARM                   0x2
#define SYSTEM_BATTERY_SYNC                0x3
#define SYSTEM_PEDO_START                  0x4
#define SYSTEM_USER_SLEEP_SYNC             0x5
#define SYSTEM_SLICE_TIME_START            0x6
#define SYSTEM_SLICE_TIME_STOP             0x7
#define SYSTEM_RESPONSE                    0x8
#define SYSTEM_DATA_SYNC_END               0x9
#define SYSTEM_DATA_SYNC_AMOUNT            0xA
#define SYSTEM_DATA_SYNC                   0xB
#define SYSTEM_RELATIVE_TIME               0xC
#define SYSTEM_APP_UPDATE                  0xD

extern volatile uint8_t SYSTEM_WORK_MODE;

extern volatile uint8_t CURRENT_WORK_MODE;
 
extern volatile uint8_t system_battery_level;

extern volatile bool system_wait_high_g_INT;

extern volatile uint32_t SYSTEM_LOCAL_TIMING;

extern volatile uint8_t system_local_click;

extern volatile uint32_t SYSTEM_SLEEP_TIMING; 


/*--------------------------------------------------------------------------*/

uint32_t get_pedo_data_length(void);

bool get_pedo_slice_handle_state(void);

void pedo_data_sto_handle(uint16_t pedo_time, uint16_t pedo_peak, uint16_t pedo_valley);

void pedo_data_end_handle(void);

void sto_no_sync_time(void);
    
void pedo_time_diff_sto_handle(void);
    
void get_pedo_data_handle(uint8_t *data, uint32_t *offset, uint8_t *length);

void pedo_data_sto_ctrl_erase(void);

/*---------------------------------------------------------------------------*/

void system_parameter_init(void);

bool system_battery_self_test(void);

void system_gsensor_init(void);

void system_high_g_dect_open(void);

void system_high_g_dect_close(void);

void system_run(void);

//void system_default_work_mode_init(void);  

void system_user_sleep_mode_init(void);
            
void system_ble_work_mode_init(void);
           
void system_mcu_sleep_mode_init(void);
           
void system_power_shutdown_mode_init(void);



//void system_default_work_mode_operate(void);  

void system_user_sleep_mode_operate(void);
            
void system_ble_work_mode_operate(void);
           
void system_mcu_sleep_mode_operate(void);
           
void system_power_shutdown_mode_operate(void);
            
void system_default_operate(void);

void system_idle_operate(void);

void system_led_flicker_init(void);

void system_HFCLK_init(void);

void system_low_battery(void);

void system_module_init(void);

#endif
