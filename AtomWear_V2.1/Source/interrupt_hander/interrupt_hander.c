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
#include "boards.h"

uint8_t system_work_mode = 0;
uint8_t	time_mode = 0;

uint8_t get_work_mode()
{
	return system_work_mode;
}

uint8_t get_time_mode()
{
	return time_mode;
}

void set_work_mode(uint8_t mode)
{
	system_work_mode = mode;
}

/*
Detection of key state
*/
uint8_t get_key_status(void)
{
	uint8_t	timer=0, key_value=0;
	while(nrf_gpio_pin_read(BUTTON_2) == 0)			//key is being pressed down
	{
		nrf_delay_ms(1000);
		++timer;
		if(timer >= 2) 
		{
			key_value = 1;
			break;
		}
	}
	return key_value;
}

/*
GPIO interrupt
switch the mode of work by button
*/
void GPIOTE_IRQHandler()
{
	uint32_t sec_count=0;
	uint8_t key_status;
	
	if((NRF_GPIOTE->EVENTS_IN[0] == 1) && (NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN0_Msk) )  
    {	
		nrf_delay_ms(200);
		
		if(get_work_mode == SYSTEM_DEFAULT_MODE)
		{
			uint8_t key_status = get_key_status();
// 			xprintf("%d\r\n", key_status);
		}
		
		NRF_GPIOTE->EVENTS_IN[0] = 0;		
		
        //NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN0_Disabled << GPIOTE_INTENSET_IN0_Pos;
		
		switch(system_work_mode)
		{
		case SYSTEM_DEFAULT_MODE :
			key_status = get_key_status();
			if(key_status == 1)	++time_mode;						
			switch(time_mode)
			{
			case MINUTE_MODE:
				if(key_status == 0)
				{
					sec_count = get_sec_count();
					sec_count += 60;
					set_sec_count(sec_count);
				}
				break;
			case HOUR_MODE:
				if(key_status == 0)
				{
					sec_count = get_sec_count();
					sec_count += 3600;
					set_sec_count(sec_count);
				}
				break;
			default:
				time_mode = 0;
				++system_work_mode;
				OLED_Clear();
				enter_pedo_mode();
				break;
			}
			break;			
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
			enter_default_mode();
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
    
    NRF_GPIOTE->CONFIG[0] =  (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos)
                             | (pin << GPIOTE_CONFIG_PSEL_Pos)
                             | (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);
    
    NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN0_Set << GPIOTE_INTENSET_IN0_Pos;
}
