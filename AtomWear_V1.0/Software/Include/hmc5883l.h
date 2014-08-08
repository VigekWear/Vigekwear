#ifndef _HMC5883L_H_
#define _HMC5883L_H_

#include "stdint.h"

#define HMC5883L_ADDR   0x1E

#define HMC5883L_CRA    0x00
#define HMC5883L_CRB    0x01
#define HMC5883L_MODE   0x02

#define HMC5883L_XMSB   0x03
#define HMC5883L_XLSB   0x04
#define HMC5883L_ZMSB   0x05
#define HMC5883L_ZLSB   0x06
#define HMC5883L_YMSB   0x07
#define HMC5883L_YLSB   0x08

#define HMC5883L_STR    0x09
#define HMC5883L_IDEN_A 0x0A
#define HMC5883L_IDEN_B 0x0B
#define HMC5883L_IDEN_C 0x0C



typedef struct
{
    int16_t X;
    int16_t Y;
    int16_t Z;
    int16_t Angle;
}hmc5883l_cmps;




void hmc5883l_test(void);


#endif

