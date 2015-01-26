#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpio.h"

typedef struct calendar
{
	uint16_t	year;
	uint8_t		month;
	uint8_t		day;
	uint8_t		hour;
	uint8_t 	minute;
	uint8_t 	second;
}calendar;

void enter_default_mode(void);
void enter_pedo_mode(void);
void enter_temp_mode(void);
void enter_cmps_mode(void);

void set_standard_time(uint16_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec);
void set_sec_count(uint32_t );
uint32_t get_sec_count(void);
calendar get_standard_time(void);
