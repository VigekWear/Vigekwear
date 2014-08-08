#ifndef I2C_BMA180_DRIVER_H
#define I2C_BMA180_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#define DEBUG_MODE		     1
#define TRANS_STOP		     1
#define TRANS_CONTINUE		 0
#define GPIO_INT_PIN         12

#define bma180_addr_write   0x80
#define bma180_addr_read    0x81
 
#include "stdint.h"


void  shake_dect_open(void);
        
void  shake_dect_close(void);

bool get_shake_state(void);

void clear_shake_state(void);


void IIC_Init(void );
 

void bma180_basic_config(void );


void bma180_read(uint8_t acce[6]);


float byte_to_float(uint8_t LSB, uint8_t MSB, int typ);

unsigned char Float_to_Char(float value, unsigned char* array);


void bma180_init(void);



#endif
