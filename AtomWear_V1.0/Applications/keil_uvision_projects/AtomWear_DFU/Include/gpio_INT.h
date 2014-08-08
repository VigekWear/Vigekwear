#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_gpio.h"

void nRF_tap_INT_config(void);

void nRF_gpio_INT_config(uint32_t nRF_gpio_INT_pin);

void TIMER1_OPEN(void);

void TIMER1_INT_CLOSE(void);

void TIMER1_INT_OPEN(void);

void timer2_init(void);
