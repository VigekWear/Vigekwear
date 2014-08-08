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
 
#include "system_work_handle.h"
#include "stdbool.h"
#include "stdint.h"
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "INT_handle.h"
#include "ble_startup.h"
#include "app_timer.h"
#include "pwr_ctrl.h"
#include "ble_nus.h"
#include "flash_ctrl.h"
#include "step_count.h"
#include "mma8452q.h"




/*--------------------------- start timer, start working -------------------------*/
void system_startup(void)
{
    TIMER1_OPEN();
}

/*--------------------------- user sleep init -------------------------*/
void system_user_sleep_mode_init(void)
{
    xprintf("\r\nenter the user sleep mode...\r\n");
}

/*--------------------------- user sleep operate -------------------------*/
void system_user_sleep_mode_operate(void)
{ }

/*--------------------------- ble work init -------------------------*/
void system_ble_work_mode_init(void)
{
    ble_prepare();
    ble_start();
}

/*--------------------------- ble mode operate -------------------------*/
void system_ble_work_mode_operate(void)
{
    if(BL.E.TIME_OUT_STATE == true || BL.E.END_TRANS_STATE == true)  // time out
    {
        ble_close();
    }
    else
    {
        ble_rec_handle();
    }

    // ble_data_operate;
}

/*--------------------------- system sleep init -------------------------*/
void system_mcu_sleep_mode_init(void)
{
}

/*---------------------------   -------------------------*/
void system_mcu_sleep_mode_operate(void)
{
    TIMER1_INT_CLOSE();
    RTC1_COUNTER_START();
    
    //shake_dect_open();   
    system_on_sleep();   
    //shake_dect_close();
   
    RTC1_COUNTER_STOP();
    TIMER1_INT_OPEN();
}

/*---------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------*/










/*---------------------------   -------------------------*/
void system_sleep_countdown_detect()
{
    // 5min sleep 5*60 = 300 *1s
    if(( get_sleep_countdown() >= SYSTEM_SLEEP_DEADLINE) && (get_work_mode() == SYSTEM_DEFAULT_MODE))
    {
        clear_sleep_countdown();
        system_mcu_sleep_mode_operate();
    }
}


/*---------------------------  -------------------------*/
void system_low_battery()
{
    uint8_t i = 10;
    // Configure LED-pins as outputs
    nrf_gpio_cfg_output(RED_LED);

    // LED 0 and LED 1 blink alternately.
    while(i--)
    {
        nrf_gpio_pin_set(RED_LED);
        nrf_delay_ms(300);
        nrf_gpio_pin_clear(RED_LED);
        nrf_delay_ms(300);
    }
    // shutdown...
    xprintf("The battery is too low...\r\n");
    NRF_POWER->SYSTEMOFF = 0x01;
}



/*-------------------------------   ----------------------------*/
void system_battery_detect()
{
    static uint32_t detect_count = 0;
    uint8_t battery_level;
    
    // check every 2 minutes 
     if(detect_count == 1)
     {
         battery_detect_start();
     }
     
     if(battery_is_available() == true)
     {         
         battery_level = get_battery_value();
         battery_detect_stop();
         set_battery_level(battery_level);
         xprintf("             level = %d \r\n", battery_level);
         
         if(get_battery_level() < 20)
             system_low_battery();
                  
     }
         
    if(detect_count > 1500000*120)
    {
        xprintf(" time on..\r\n");
        detect_count = 0;
    }
    detect_count++;
}


/*---------------------------  ------------------------*/
void system_state_detect(void)
{
    system_sleep_countdown_detect();
    system_battery_detect();
}


/*---------------------------   -------------------------*/
void system_led_flicker_init()
{
    // gpio and timer init
    // timer init
    nrf_gpio_cfg_output(RED_LED);
    nrf_gpio_cfg_output(GREEN_LED);
    nrf_gpio_pin_clear(RED_LED);
    nrf_gpio_pin_clear(GREEN_LED);

}

/*---------------------------  -------------------------*/
void system_HFCLK_init()
{
    // Start 16 MHz crystal oscillator.
    NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;    // HFCLK oscillator state = 0.
    NRF_CLOCK->TASKS_HFCLKSTART     = 1;    // Start HFCLK clock source

    // Wait for the external oscillator to start up.
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}


/*---------------------------  -----------------------*/
void system_module_init()
{
    // uart init
    simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, 0); 
    xprintf("start up..\r\n");
    
    //  
    system_HFCLK_init();

    //  
    if(!system_battery_self_test())
    {
        system_low_battery();
    }
    //  
     system_bootloader_check();
    //   
    system_led_flicker_init();
    nrf_gpio_pin_set(RED_LED);
    //  
    system_parameter_init();
    //  
    system_I2C_init();
    //  
    system_gpio_LoToHi_INT_init(ACCE_TAP_PIN_NUMBER);
    
    nrf_delay_ms(2000);
    nrf_gpio_pin_clear(RED_LED);
    
    
}

 
/*--------------------------- in step count mode by default -------------------*/
void system_default_work_mode_init()
{
    
    MMA845x_Init();
	MMA845x_Active();	
    pedometer_init();
}

/*--------------------------- -----------------------*/
void system_default_work_mode_operate()
{
    pedometer_startup();
}
