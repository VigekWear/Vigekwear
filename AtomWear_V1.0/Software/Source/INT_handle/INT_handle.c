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
 
#include "INT_handle.h"
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_soc.h"
#include "xprintf.h"
#include "app_util.h"
#include "system_work_handle.h"
#include "boards.h"
#include "ble_startup.h"
#include "app_timer.h"
#include "bma180.h"

static uint8_t flicker_counter = 0;
static uint8_t flicker_cycle = 0;
volatile uint8_t system_local_click   = 0;

void set_led_flicker(uint8_t counter)
{
    flicker_counter = counter;
}


 
void GPIOTE_IRQHandler()
{
    if((NRF_GPIOTE->EVENTS_IN[0] == 1) && (NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN0_Msk) )  
    {
        NRF_GPIOTE->EVENTS_IN[0] = 0;
        //NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN0_Disabled << GPIOTE_INTENSET_IN0_Pos;
        
            switch(get_work_mode())
            {
                // tap cyclical
            case SYSTEM_PEDOMETER_MODE :
                if(get_pedo_slice_handle_state() == true)
                    pedo_data_end_handle();
                
                clear_sleep_countdown();
                work_mode_switch();
                work_switch_set();                
                break;
                //
            case SYSTEM_USER_SLEEP_MODE:
                work_mode_switch();
                work_switch_set();  
                break;

            case SYSTEM_BLE_WORK_MODE:
                if(BL.E.CONNECT_STATE == false)          // ble is not paired with phone
                {
                    ble_close();
                }
                break;
            }
   
    }

    // Event causing the interrupt must be cleared.
    if((NRF_GPIOTE->EVENTS_IN[1] == 1) && (NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN1_Msk)  )  
    {
        NRF_GPIOTE->EVENTS_IN[1] = 0;
        xprintf("GPIO INT.\r\n");
    }

}



void TIMER1_IRQHandler()
{
    if((NRF_TIMER1->EVENTS_COMPARE[0] == 1) && (NRF_TIMER1->INTENSET & TIMER_INTENSET_COMPARE0_Msk))
    {
        NRF_TIMER1->EVENTS_COMPARE[0] = 0;
        NRF_TIMER1->TASKS_CLEAR = 1;
        if(get_work_switch_state())  
        {
            work_switch_clear();
            
            switch(get_work_mode())
            {
            case SYSTEM_PEDOMETER_MODE:
                system_default_work_mode_init();
                set_led_flicker(2);
                break;

            case SYSTEM_USER_SLEEP_MODE :
                system_user_sleep_mode_init();
                set_led_flicker(3);
                break;

            case SYSTEM_BLE_WORK_MODE :
                system_ble_work_mode_init();
                set_led_flicker(4);
                break;

            }
        }
        else     
        {
            switch(get_work_mode())
            {
            case SYSTEM_PEDOMETER_MODE:
                system_default_work_mode_operate();

                break;

            case SYSTEM_USER_SLEEP_MODE :
                system_user_sleep_mode_operate();
                break;

            case SYSTEM_BLE_WORK_MODE :
                system_ble_work_mode_operate();
                break;

            case SYSTEM_MCU_SLEEP_MODE :
                system_mcu_sleep_mode_operate();
                break;

            }
            system_idle_operate();
        }

    }
}


void TIMER1_OPEN()
{
    NRF_TIMER1->MODE           = TIMER_MODE_MODE_Timer;        // Set the timer in Timer Mode.
    NRF_TIMER1->PRESCALER      = 4;                            // Prescaler 4, =>1 tick = 1 us.
    NRF_TIMER1->BITMODE        = TIMER_BITMODE_BITMODE_32Bit;  // 16 bit mode.
    /* Bit 16 : Enable interrupt on COMPARE[0] */
    NVIC_SetPriority(TIMER1_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_EnableIRQ(TIMER1_IRQn);

    NRF_TIMER1->TASKS_CLEAR    = 1;                            // clear the task first to be usable for later.
    NRF_TIMER1->CC[0]          =  10000;   // 10ms/1us =10000
    NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
    NRF_TIMER1->TASKS_START    = 1;

}

void TIMER1_INT_CLOSE()
{
    NVIC_DisableIRQ(TIMER1_IRQn);
    NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Disabled << TIMER_INTENSET_COMPARE0_Pos;
}

void TIMER1_INT_OPEN()
{
    NVIC_EnableIRQ(TIMER1_IRQn);
    NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
}


 
void GPIO_LoToHi_INT_config(uint32_t pin)
{
    nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_PULLDOWN);
    // Enable interrupt:
    NVIC_SetPriority(GPIOTE_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_EnableIRQ(GPIOTE_IRQn);
    //sd_nvic_EnableIRQ(GPIOTE_IRQn);
    
    NRF_GPIOTE->CONFIG[0] =  (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos)
                             | (pin << GPIOTE_CONFIG_PSEL_Pos)
                             | (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
    
    NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN0_Set << GPIOTE_INTENSET_IN0_Pos;
}




void led_flicker()
{
    if(flicker_counter != 0)
    {
        flicker_cycle++;
        if( flicker_cycle == 31)
        {
            nrf_gpio_pin_set(GREEN_LED);
        }
        if( flicker_cycle == 51)
        {
            nrf_gpio_pin_clear(GREEN_LED);
            flicker_cycle = 0;
            flicker_counter --;
        }

    }

}


void system_local_timing()
{
    if(system_local_click == 100)
    {
        xprintf("the time is :%d\r\n ", get_system_time());
        system_local_click = 0;
        //if(BL.E.OPEN_STATE == true)
        system_time_update();
    }
    else
        system_local_click++;
}



void system_idle_operate()
{
    led_flicker();
    system_local_timing();
}
