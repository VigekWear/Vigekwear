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
 
#include <stdio.h> 
#include "compiler_abstraction.h"
#include "nrf.h"
#include "nrf_delay.h"

/*lint --e{438} "Variable not used" */
void nrf_delay_ms(uint32_t volatile number_of_ms)
{
    while(number_of_ms != 0)
    {
        number_of_ms--;
        nrf_delay_us(999);
    }
}



void Delay_s(uint32_t t)
{
    nrf_delay_ms(t*1000);    
}


void Delay_Ms(uint32_t i)
{
    nrf_delay_ms(i*10);
}



