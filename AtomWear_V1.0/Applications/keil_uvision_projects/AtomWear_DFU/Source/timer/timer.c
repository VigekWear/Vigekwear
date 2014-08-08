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
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_soc.h"
#include "xprintf.h"
#include "app_util.h"
#include "boards.h"
#include "ble_startup.h"
#include "app_timer.h"
#include "timer.h"


void TIMER1_IRQHandler()
{
    if((NRF_TIMER1->EVENTS_COMPARE[0] == 1) && (NRF_TIMER1->INTENSET & TIMER_INTENSET_COMPARE0_Msk))    
    {
        NRF_TIMER1->EVENTS_COMPARE[0] = 0;
        NRF_TIMER1->TASKS_CLEAR = 1;
        NRF_TIMER1->TASKS_STOP = 1;
     
        set_ble_rec_state(DFU_ERROR);
    }
}




void time1_init()
{
    NRF_TIMER1->MODE           = TIMER_MODE_MODE_Timer;        // Set the timer in Timer Mode.
    NRF_TIMER1->PRESCALER      = 4;                            // Prescaler 4, =>1 tick = 1 us.
    NRF_TIMER1->BITMODE        = TIMER_BITMODE_BITMODE_32Bit;  // 16 bit mode.
    /* Bit 16 : Enable interrupt on COMPARE[0] */
    NVIC_SetPriority(TIMER1_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_EnableIRQ(TIMER1_IRQn);

    NRF_TIMER1->TASKS_CLEAR    = 1;                            // clear the task first to be usable for later.
    NRF_TIMER1->CC[0]          =  2000000;   // 2s/1us =2000 000
    NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
    //NRF_TIMER1->TASKS_START    = 1;

}

void timer1_start()
{
    NRF_TIMER1->TASKS_CLEAR    = 1; 
    NRF_TIMER1->TASKS_START    = 1;    
}

void timer1_stop()
{
    NRF_TIMER1->TASKS_CLEAR    = 1; 
    NRF_TIMER1->TASKS_STOP     = 1; 
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
