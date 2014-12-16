#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpio.h"

#define SYSTEM_PEDOMETER_MODE		0
#define SYSTEM_TEMP_MODE			1

void GPIO_HiToLo_INT_config(uint32_t pin);

uint8_t get_work_mode(void);
void set_work_mode(uint8_t mode);
void enter_pedo_mode(void);
void enter_temp_mode(void);
