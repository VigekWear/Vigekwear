#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpio.h"

#define SYSTEM_DEFAULT_MODE			0
#define SYSTEM_PEDOMETER_MODE		1
#define SYSTEM_TEMP_MODE			2
#define SYSTEM_COMPASS_MODE			3

#define MINUTE_MODE	1
#define HOUR_MODE	2

uint8_t get_key_status(void);
uint8_t get_time_mode(void);

void GPIO_HiToLo_INT_config(uint32_t pin);

uint8_t get_work_mode(void);
void set_work_mode(uint8_t mode);
void enter_pedo_mode(void);
void enter_temp_mode(void);


