#ifndef  _DISPLAY_MODE_H
#define  _DISPLAY_MODE_H

#include "oled.h"
//
//
void Clear_Screen(void);
void All_Screen(void); 
void oled_display(const uint8_t * pic);
void pic(const unsigned char *pic);
void Display(void);	
void oled_test(void);


#endif
