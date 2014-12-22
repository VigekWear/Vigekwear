#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_soc.h"
#include "xprintf.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "mma8452q.h"
#include "step_count.h"
#include "interrupt_hander.h"
#include "main.h"
#include "oled.h"


uint8_t system_work_mode = 0;

uint8_t get_work_mode()
{
	return system_work_mode;
}

void set_work_mode(uint8_t mode)
{
	system_work_mode = mode;
}
/*
GPIO interrupt
switch the mode of work by button
*/
void GPIOTE_IRQHandler()
{
    if((NRF_GPIOTE->EVENTS_IN[0] == 1) && (NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN0_Msk) )  
    {
		nrf_delay_ms(200);
        NRF_GPIOTE->EVENTS_IN[0] = 0;
        //NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN0_Disabled << GPIOTE_INTENSET_IN0_Pos;
		
		switch(system_work_mode)
		{
		case SYSTEM_PEDOMETER_MODE :
			++system_work_mode;
			OLED_Clear();
			enter_temp_mode();
			break;
		
		case SYSTEM_TEMP_MODE :
			++system_work_mode;
			OLED_Clear();
			enter_cmps_mode();
			break;
		case SYSTEM_COMPASS_MODE :
			system_work_mode = 0;
			OLED_Clear();
			enter_pedo_mode();
			break;
		}
   
    }
}

void GPIO_HiToLo_INT_config(uint32_t pin)
{
    nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_PULLUP);
    // Enable interrupt:
    NVIC_SetPriority(GPIOTE_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_EnableIRQ(GPIOTE_IRQn);
    
    NRF_GPIOTE->CONFIG[0] =  (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos)
                             | (pin << GPIOTE_CONFIG_PSEL_Pos)
                             | (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
    
    NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN0_Set << GPIOTE_INTENSET_IN0_Pos;
}
