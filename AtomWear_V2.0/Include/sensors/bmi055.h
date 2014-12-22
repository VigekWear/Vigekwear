#ifndef __BMI055_H__
#define __BMI055_H__

#define ACCE_IIC_ADDRESS       0x19
#define GYRO_IIC_ADDRESS       0x69

void acce_init(void);
void gyro_init(void);
void bmi055_init(void);

#endif
