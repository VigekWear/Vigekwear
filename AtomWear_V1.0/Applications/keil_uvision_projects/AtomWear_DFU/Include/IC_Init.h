#include "IC.h"

#ifndef  _IC_Init_

#define  _IC_Init_

// void IC_IIC_Start(void);
// void IC_IIC_Stop(void);
//bit Write_IIC_Data(uchar Data);
void Write_Command(uchar command);
void Write_Data(uchar date);
void Reset_IC(void);
void Init_IC(void);

#endif
