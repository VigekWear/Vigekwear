#ifndef _STEP_COUNT_H_
#define _STEP_COUNT_H_

#include "nrf_delay.h"
#include "simple_uart.h"
#include "xprintf.h"
#include "nrf_gpio.h"
#include "nrf_error.h"

//pedo_state
#define PEDO_COLLECT_PREPARE        1
#define PEDO_COLLECT_START          2
#define PEDO_COLLECT_STOP           3

struct g_node
{
    uint16_t value;
    uint8_t  time;
};



struct s_node  
{
    uint16_t value;
    uint8_t  time;
};

uint8_t get_pedo_start_state(void);

void set_pedo_start_state(uint8_t state);

void set_pedo_stop(void);

//Data send adapt CRC16 verification,The following is the function of CRC16,please refer
//-------------------------------------------------------------------------------------------
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);

int16_t byte_to_word(uint8_t LSB, uint8_t MSB);

void output_xyz_serial_send(uint8_t acce[]);

void output_modular_test(void);


void pedometer_startup(void);

void system_default_work_mode_init(void);

void system_default_work_mode_operate(void);

#endif
