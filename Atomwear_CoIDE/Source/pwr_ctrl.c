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
 
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "pwr_ctrl.h"
#include "xprintf.h"
#include "nrf_delay.h"
#include "boards.h"


#define DCDC_MODE   0
#define LOW_MODE    1


#define ADC_REF_VOLTAGE_IN_MILLIVOLTS        1200                                      /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION         4                                         /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS       270                                       /**< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com. */

#define HIGH_MILLIVOLTS						4200
#define LOW_MILLIVOLTS						3300

volatile uint8_t system_battery_level;

/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 * @retval     Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / 1024) * ADC_PRE_SCALING_COMPENSATION)


/** @brief Function for converting the input voltage (in milli volts) into percentage of 3.0 Volts.
 *
 *  @details The calculation is based on a linearized version of the battery's discharge
 *           curve. 3.0V returns 100% battery level. The limit for power failure is 2.1V and
 *           is considered to be the lower boundary.
 *
 *           The discharge curve for CR2032 is non-linear. In this model it is split into
 *           4 linear sections:
 *           - Section 1: 3.0V - 2.9V = 100% - 42% (58% drop on 100 mV)
 *           - Section 2: 2.9V - 2.74V = 42% - 18% (24% drop on 160 mV)
 *           - Section 3: 2.74V - 2.44V = 18% - 6% (12% drop on 300 mV)
 *           - Section 4: 2.44V - 2.1V = 6% - 0% (6% drop on 340 mV)
 *
 *           These numbers are by no means accurate. Temperature and
 *           load in the actual application is not accounted for!
 *
 *  @param[in] mvolts The voltage in mV
 *
 *  @return    Battery level in percent.
*/
static __INLINE uint8_t battery_level_in_percent(const uint16_t mvolts)
{
    uint8_t battery_level;

    if (mvolts >= HIGH_MILLIVOLTS)
    {
        battery_level = 100;
    }
    else if(mvolts <= LOW_MILLIVOLTS)
	{
		battery_level = 0;
	}
	else
	{
		battery_level = (mvolts - LOW_MILLIVOLTS) / 9;
	}

    return battery_level;
}

bool battery_self_detect()
{
    uint16_t     adc_result;
    uint16_t    batt_lvl_in_milli_volts;
	
	nrf_gpio_cfg_input(1, NRF_GPIO_PIN_NOPULL);

	// Configure ADC
    NRF_ADC->INTENSET   = ADC_INTENSET_END_Disabled;
    // 8bit, Supply voltage with 1/3 prescaling, Use internal 1.2V bandgap voltage
    // Analog input pins disabled, Analog external reference inputs disabled.
    NRF_ADC->CONFIG     = (ADC_CONFIG_RES_10bit                        << ADC_CONFIG_RES_Pos)     |
                          (ADC_CONFIG_INPSEL_AnalogInputNoPrescaling << ADC_CONFIG_INPSEL_Pos)  |
                          (ADC_CONFIG_REFSEL_VBG                      << ADC_CONFIG_REFSEL_Pos)  |
                          (ADC_CONFIG_PSEL_AnalogInput2               << ADC_CONFIG_PSEL_Pos)    |
                          (ADC_CONFIG_EXTREFSEL_None                  << ADC_CONFIG_EXTREFSEL_Pos);
    // Clear an ADC conversion is completed.
    NRF_ADC->EVENTS_END     = 0;
    NRF_ADC->ENABLE         = ADC_ENABLE_ENABLE_Enabled;

	NRF_ADC->EVENTS_END     = 0;
    NRF_ADC->TASKS_START    = 1;
    while(NRF_ADC->EVENTS_END == 0);

    NRF_ADC->EVENTS_END     = 0;

    adc_result              = NRF_ADC->RESULT;
    NRF_ADC->TASKS_STOP     = 1;

    // battery value
    batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result); //+
    //DIODE_FWD_VOLT_DROP_MILLIVOLTS;
    // battery percentage
    system_battery_level     = battery_level_in_percent(batt_lvl_in_milli_volts);

    xprintf("the adc_result is %d. \r\n", adc_result);
    xprintf("the battery's voltage value is %d.\r\n", batt_lvl_in_milli_volts );
    xprintf("the percentage of the battery is %d . \r\n", system_battery_level );
    if(system_battery_level > 6)
        return true;
    return false;
}

 
void battery_detect_start()
{
    NRF_ADC->TASKS_START = 1;
}

bool battery_is_available()
{
    if( (NRF_ADC->EVENTS_END) == 0)
        return false;
    else 
        return true;
    
}

void battery_detect_stop()
{
    NRF_ADC->EVENTS_END     = 0;
    NRF_ADC->TASKS_STOP     = 1;
}

uint8_t get_battery_value()
{
    uint16_t     adc_result;
    uint16_t    batt_lvl_in_milli_volts;
    
    NRF_ADC->EVENTS_END     = 0;
    adc_result              = NRF_ADC->RESULT;
// 	xprintf("the adc_result is %d. \r\n", adc_result);

    batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result); 
    return battery_level_in_percent(batt_lvl_in_milli_volts);
}

 
void system_off_set()
{               
    NRF_POWER->INTENSET = 0x01;  
    NRF_POWER->RAMON = NRF_POWER->RAMON|0x000f0000;
    nrf_delay_ms(30);
    
    NRF_POWER->SYSTEMOFF = 0x01;
}

 
void system_on_sleep()
{
    xprintf("system is going to sleep..z..z..Z...\r\n");
    NRF_POWER->TASKS_LOWPWR = 0x01;
    __WFI();
}

void system_low_battery()
{
    uint8_t i = 5;

    // LED0 blink.
    while(i--)
    {
        nrf_gpio_pin_set(LED_0);
        nrf_delay_ms(300);
        nrf_gpio_pin_clear(LED_0);
        nrf_delay_ms(300);
    }
    // shutdown...
    xprintf("The battery is too low...\r\n");
//     NRF_POWER->SYSTEMOFF = 0x01;
}
