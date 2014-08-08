#ifndef _TIME_H
#define _TIME_H


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



void time1_init(void);

void timer1_start(void);

void timer1_stop(void);

void TIMER1_INT_CLOSE(void);

void TIMER1_INT_OPEN(void);


#endif

