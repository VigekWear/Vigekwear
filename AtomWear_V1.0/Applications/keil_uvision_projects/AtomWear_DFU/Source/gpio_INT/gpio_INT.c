#include "gpio_INT.h"
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_soc.h"
#include "xprintf.h"
#include "app_util.h"
#include "system_work_mode.h"
#include "boards.h"
#include "ble_startup.h"
#include "app_timer.h"

uint8_t flicker_counter = 0;
uint8_t flicker_cycle = 0;

void GPIOTE_IRQHandler()
{
    if((NRF_GPIOTE->EVENTS_IN[0] == 1) && (NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN0_Msk) )
    {
        NRF_GPIOTE->EVENTS_IN[0] = 0;
        //NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN0_Disabled << GPIOTE_INTENSET_IN0_Pos;
        
        if(system_wait_high_g_INT == false)
        { 
            NRF_GPIOTE->EVENTS_IN[0] = 0;
            //NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN0_Disabled << GPIOTE_INTENSET_IN0_Pos;
            switch(SYSTEM_WORK_MODE)
            {
                // tap cyclical
            case SYSTEM_PEDOMETER_MODE :
                if(get_pedo_slice_handle_state() == true)
                    pedo_data_end_handle();
                SYSTEM_SLEEP_TIMING = 0;
                SYSTEM_WORK_MODE = SYSTEM_USER_SLEEP_MODE;
                break;
                //
            case SYSTEM_USER_SLEEP_MODE:
                SYSTEM_WORK_MODE = SYSTEM_BLE_WORK_MODE;
                break;

            case SYSTEM_BLE_WORK_MODE:
                if(BL.E.CONNECT_STATE == false)          // ble is not paired with phone
                {
                    ble_close();
                }
                break;
            }
        }
        else
        {
            xprintf("system wake up in INT........\r\n");            
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
        if(SYSTEM_WORK_MODE != CURRENT_WORK_MODE) 
        {
            switch(SYSTEM_WORK_MODE)
            {
            case SYSTEM_PEDOMETER_MODE:
                CURRENT_WORK_MODE = SYSTEM_WORK_MODE;
                system_default_work_mode_init();
                flicker_counter = 2;
                break;

            case SYSTEM_USER_SLEEP_MODE :
                CURRENT_WORK_MODE = SYSTEM_WORK_MODE;
                system_user_sleep_mode_init();
                flicker_counter = 3;
                break;

            case SYSTEM_BLE_WORK_MODE :
                CURRENT_WORK_MODE = SYSTEM_WORK_MODE;
                system_ble_work_mode_init();
                flicker_counter = 4;
                break;

            case SYSTEM_MCU_SLEEP_MODE :
                CURRENT_WORK_MODE = SYSTEM_WORK_MODE;
                system_mcu_sleep_mode_init();
                break;

            case SYSTEM_POWER_SHUTDOWN_MODE :
                CURRENT_WORK_MODE = SYSTEM_WORK_MODE;
                system_power_shutdown_mode_init();
                break;
            }
        }
        else 
        {
            switch(CURRENT_WORK_MODE)
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

            case SYSTEM_POWER_SHUTDOWN_MODE :
                system_power_shutdown_mode_operate();
                break;

            }
            system_idle_operate();
        }

    }
}


void TIMER2_IRQHandler()
{
    if((NRF_TIMER2->EVENTS_COMPARE[0] == 1) && (NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE0_Msk)) 
    {
        NRF_TIMER2->EVENTS_COMPARE[0] = 0;
        NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Disabled << TIMER_INTENSET_COMPARE0_Pos; 

        NRF_TIMER2->TASKS_STOP = 1;
        xprintf("It is changed..\r\n");
    }

    if((NRF_TIMER2->EVENTS_COMPARE[1] == 1) && (NRF_TIMER2->INTENSET & TIMER_INTENSET_COMPARE1_Msk))
    {
        NRF_TIMER2->EVENTS_COMPARE[1] = 0;
        NRF_TIMER2->TASKS_CLEAR       = 1;
        nrf_gpio_pin_toggle(GREEN_LED);
        if((--flicker_counter) == 0)
        {
            NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE1_Disabled << TIMER_INTENSET_COMPARE1_Pos; 
            NRF_TIMER2->TASKS_STOP = 1;
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

void timer2_init()
{
    NRF_TIMER2->MODE           = TIMER_MODE_MODE_Timer;        // Set the timer in Timer Mode.
    NRF_TIMER2->PRESCALER      = 9;                            // Prescaler(max): 2^9  produces 31250 Hz timer frequency => 1 tick = 32 us.
    NRF_TIMER2->BITMODE        = TIMER_BITMODE_BITMODE_32Bit;  // 32 bit mode.
    /* Bit 16 : Enable interrupt on COMPARE[0] */
    NVIC_SetPriority(GPIOTE_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_EnableIRQ(TIMER2_IRQn);
    NRF_TIMER2->TASKS_CLEAR    = 1;                            // clear the task first to be usable for later.
    NRF_TIMER2->CC[0]          =  31250;   // 1s/32us =31250
    NRF_TIMER2->CC[1]          =  15625;   // 0.5s/32us =3125
}

void nRF_gpio_INT_config(uint32_t nRF_gpio_INT_pin)
{
    nrf_gpio_cfg_input(nRF_gpio_INT_pin, NRF_GPIO_PIN_PULLUP);
    // Enable interrupt:
    NVIC_SetPriority(GPIOTE_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_EnableIRQ(GPIOTE_IRQn);
    //sd_nvic_EnableIRQ(GPIOTE_IRQn);
    NRF_GPIOTE->CONFIG[1] =  (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos)
                             | (nRF_gpio_INT_pin << GPIOTE_CONFIG_PSEL_Pos)
                             | (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
    NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN1_Set << GPIOTE_INTENSET_IN1_Pos;
}


// acce tap interrupt
void nRF_tap_INT_config()
{
    nrf_gpio_cfg_input(ACCE_TAP_PIN_NUMBER, NRF_GPIO_PIN_PULLDOWN);
    // Enable interrupt:
    NVIC_SetPriority(GPIOTE_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_EnableIRQ(GPIOTE_IRQn);
    //sd_nvic_EnableIRQ(GPIOTE_IRQn);
    NRF_GPIOTE->CONFIG[0] =  (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos)
                             | (ACCE_TAP_PIN_NUMBER << GPIOTE_CONFIG_PSEL_Pos)
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
        xprintf("the time is :%d\r\n ", SYSTEM_LOCAL_TIMING);
        system_local_click = 0;
        //if(BL.E.OPEN_STATE == true)
        SYSTEM_LOCAL_TIMING++;
    }
    else
        system_local_click++;
}



void system_idle_operate()
{
    led_flicker();
    system_local_timing();
}
