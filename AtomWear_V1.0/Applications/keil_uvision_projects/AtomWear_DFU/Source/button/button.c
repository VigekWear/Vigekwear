#include "button.h"
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "xprintf.h"

#define nRF_gpio_INT_pin         6     

static uint8_t button_tmp = 0;
// 0: nothing happened, 1:single-click, 2:double-click, 3:long-click 
static uint8_t BUTTON_STATE = 0;

//NRF_TIMER_Type * p_timer; //NRF_TIMER0/1/2
#define p_timer             NRF_TIMER0

void GPIOTE_IRQHandler()
{
	 // Event causing the interrupt must be cleared.
    if((NRF_GPIOTE->EVENTS_IN[0] == 1) && (NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN0_Msk) && (NRF_GPIOTE->CONFIG[0] &  (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) ) )
    {
        NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN0_Disabled << GPIOTE_INTENSET_IN0_Pos;    
        NRF_GPIOTE->EVENTS_IN[0] = 0;
        NRF_GPIOTE->CONFIG[0] =  (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos)
                           | (nRF_gpio_INT_pin << GPIOTE_CONFIG_PSEL_Pos)  
                           | (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
               
        p_timer->TASKS_CLEAR = 1;
        p_timer->INTENSET       = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;  
        p_timer->TASKS_START = 1;
        //xprintf("High INT.\r\n");
    }
    // Event causing the interrupt must be cleared.
    if((NRF_GPIOTE->EVENTS_IN[0] == 1) && (NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN0_Msk) && (NRF_GPIOTE->CONFIG[0] &  (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos) ) ) 
        {
            //xprintf("Low INT.\r\n");
           // NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN0_Disabled << GPIOTE_INTENSET_IN0_Pos;    
            NRF_GPIOTE->EVENTS_IN[0] = 0;
            NRF_GPIOTE->CONFIG[0] =  (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos)
                           | (nRF_gpio_INT_pin << GPIOTE_CONFIG_PSEL_Pos)  
                           | (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
            
            p_timer->TASKS_CAPTURE[1] = 1;  
            if( (p_timer->CC[1] >= 28846) && (button_tmp == 1) )   
            {
                BUTTON_STATE = 3;
                button_tmp  = 0;
                p_timer->TASKS_STOP = 1;
                p_timer->TASKS_CLEAR = 1;
                p_timer->INTENSET = TIMER_INTENSET_COMPARE0_Disabled << TIMER_INTENSET_COMPARE0_Pos; 
                p_timer->INTENSET = TIMER_INTENSET_COMPARE3_Disabled << TIMER_INTENSET_COMPARE3_Pos; 
                xprintf("Long click.\r\n");
            }
            else
            {
                if(button_tmp == 1)    
                {
                   // p_timer->TASKS_CLEAR = 1;
                    //xprintf("1s jishi.\r\n");
                    p_timer->INTENSET      = TIMER_INTENSET_COMPARE3_Enabled << TIMER_INTENSET_COMPARE3_Pos;   
                }
            }
          //NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN0_Set << GPIOTE_INTENSET_IN0_Pos;     
        }
}
void TIMER0_IRQHandler()
{
    if((p_timer->EVENTS_COMPARE[0] ==1) && (p_timer->INTENSET & TIMER_INTENSET_COMPARE0_Msk))     
    {
        p_timer->EVENTS_COMPARE[0] = 0;
        p_timer->INTENSET = TIMER_INTENSET_COMPARE0_Disabled << TIMER_INTENSET_COMPARE0_Pos; 
        //p_timer->TASKS_CLEAR = 1;
        
        NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos;     
        
        if((nrf_gpio_pin_read(8) & 1) && (button_tmp == 0))     
        {
            button_tmp = 1;
           // xprintf("one ok.\r\n");
        }
        else 
        {
            if( (nrf_gpio_pin_read(8) & 1) && (button_tmp == 1))     
            {
                p_timer->TASKS_STOP = 1;
                p_timer->TASKS_CLEAR = 1;

                p_timer->INTENSET = TIMER_INTENSET_COMPARE3_Enabled << TIMER_INTENSET_COMPARE3_Disabled; 
                BUTTON_STATE = 2;
                button_tmp = 0;
                xprintf("Double click.\r\n");
            }
        }
        //xprintf("10ms.\r\n");
    }
    if( (p_timer->EVENTS_COMPARE[3] ==1) && (p_timer->INTENSET & TIMER_INTENSET_COMPARE3_Msk) && (nrf_gpio_pin_read(8) == 0 )&& (button_tmp == 1)) 
     {
         p_timer->EVENTS_COMPARE[3] = 0;
         p_timer->TASKS_STOP  = 1;
         p_timer->TASKS_CLEAR = 1;
         p_timer->INTENSET = TIMER_INTENSET_COMPARE0_Disabled << TIMER_INTENSET_COMPARE0_Pos; 
         p_timer->INTENSET = TIMER_INTENSET_COMPARE3_Disabled << TIMER_INTENSET_COMPARE3_Pos; 
         BUTTON_STATE = 1;
         button_tmp = 0;
         xprintf("Single click.\r\n");
     }
}

void nRF_gpio_INT_config()
{
  nrf_gpio_cfg_input(nRF_gpio_INT_pin, NRF_GPIO_PIN_PULLUP);
	// Enable interrupt:
  NVIC_EnableIRQ(GPIOTE_IRQn);
  
  NRF_GPIOTE->CONFIG[0] =  (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos)
                           | (nRF_gpio_INT_pin << GPIOTE_CONFIG_PSEL_Pos)  
                           | (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);  
  NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN0_Set << GPIOTE_INTENSET_IN0_Pos;
}


void button_start()
{
    // Start 16 MHz crystal oscillator.
    NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;    // HFCLK oscillator state = 0.
    NRF_CLOCK->TASKS_HFCLKSTART     = 1;    // Start HFCLK clock source

    // Wait for the external oscillator to start up.
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
    {
        // Do nothing.
    }
    p_timer->MODE           = TIMER_MODE_MODE_Timer;        // Set the timer in Timer Mode.
    p_timer->PRESCALER      = 9;                            // Prescaler 9 produces 31250 Hz timer frequency => 1 tick = 32 us.
    p_timer->BITMODE        = TIMER_BITMODE_BITMODE_32Bit;  // 16 bit mode.
    /* Bit 16 : Enable interrupt on COMPARE[0] */
    NVIC_EnableIRQ(TIMER0_IRQn);
    
    p_timer->TASKS_CLEAR    = 1;                            // clear the task first to be usable for later.
    
    //
    p_timer->CC[0]          = 288;    // 15ms
    p_timer->CC[1]          = 0;
    p_timer->CC[2]          = 0;
    p_timer->CC[3]          = 9615;    // 0.5s
    
    // p_timer->TASKS_START    = 1;                    // Start timer.

    
}


