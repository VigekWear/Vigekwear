/* Copyright (c) 2012 Giayee Technology Co. Ltd. All Rights Reserved.
 *
 * Author: LL
 * Web: www.giayee.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
 
#include "twi_master.h"
#include "nrf_delay.h"
#include "oled.h"
#include "font.h"
#include "Photo_Show.h"

//OLED display buffer
//format:
//[0]0 1 2 3 ... 63	
//[1]0 1 2 3 ... 63	
//[2]0 1 2 3 ... 63	
//[3]0 1 2 3 ... 63			   
uint8_t OLED_GRAM[4][64] = 
{
{0x00,0x00,0x3E,0x22,0x22,0xBE,0x62,0x22,0x22,0x3E,0x22,0x22,0x3E,0x00,0x00,0x00},
{0x80,0x84,0x84,0x42,0x45,0x49,0x31,0x21,0x11,0x09,0x05,0x03,0x00,0x00,0x00,0x00},/*"?",0*/
{0x00,0x42,0x22,0x52,0x9E,0x12,0x92,0x72,0x02,0x00,0xFC,0x00,0x00,0xFF,0x00,0x00},
{0x88,0x48,0x34,0x04,0x02,0x11,0x60,0x00,0x10,0x60,0x00,0x02,0x14,0xE3,0x00,0x00},/*"?",1*/
};

/**********************************************
//
// write cmd api

**********************************************/


void Write_Command(uint8_t command)
{
    uint8_t send[2];
    // send[0] = Slave_Address;
    send[0] = OP_Command;
    send[1] = command;
    twi_master_transfer(Slave_Address, send, 2 ,TWI_ISSUE_STOP );
}

/**********************************************
//
// write data api
//
**********************************************/


void Write_Data(uint8_t data)
{
    uint8_t send[2];
    // send[0] = Slave_Address;
    send[0] = OP_Data;
    send[1] = data;
    twi_master_transfer(Slave_Address, send, 2 ,TWI_ISSUE_STOP );
}

/******************************************************
//
// IC init
//
******************************************************/

void Init_IC()
{

   Write_Command(0xAE);     //Set Display Off

   Write_Command(0xd5);     //display divide ratio/osc. freq. mode
   Write_Command(0xd1);     //105Hz	  click 470 KHz
 
   Write_Command(0xA8);     //multiplex ration mode:31
   Write_Command(0x1F);		//32

   Write_Command(0xD3);     //Set Display Offset
   Write_Command(0x00);
   
   Write_Command(0x40);     //Set Display Start Line

   Write_Command(0x8D);     //Set Display Offset
   //Write_Command(0x10);		//disable charge pump
   Write_Command(0x14);	//for enabling charge pump

   Write_Command(0xA1);     //Segment Remap

   Write_Command(0xC8);     //Sst COM Output Scan Direction

   Write_Command(0xDA);     //common pads hardware: alternative
   Write_Command(0x12);

   Write_Command(0x81);     //contrast control 
   Write_Command(0xcf);		//0xCF


   Write_Command(0xD9);	    //set pre-charge period
   Write_Command(0xF1);

   Write_Command(0xDB);     //VCOM deselect level mode
   Write_Command(0x40);	    //set Vvcomh=0.83*Vcc

   Write_Command(0xA4);     //Set Entire Display On/Off

   Write_Command(0xA6);     //Set Normal Display

   Write_Command(0xAF);     //Set Display On
}



//extern uint8_t ROW[3][8];

uint8_t ROW[3][8]={
{0x49,0x92,0x24,0x49,0x92,0x24,0x49,0x92},
{0x92,0x24,0x49,0x92,0x24,0x49,0x92,0x24},
{0x24,0x49,0x92,0x24,0x49,0x92,0x24,0x49}
};

/******************************************************
//
// clear oled
//
******************************************************/
void OLED_Clear(void)  
{  
	uint8_t i,n;  
	for(i=0;i<PAGE_TOTAL;i++)for(n=0;n<COLUMN_MAX;n++)OLED_GRAM[i][n] = 0X00;  
	OLED_Refresh_Gram();
}

void Clear_Screen()
{
   uint8_t page_number,column_number;
   for(page_number=OLED_MIN;page_number<PAGE_TOTAL;page_number++)			//page_number 0~3
   {
     Write_Command(START_PAGE+page_number);									//#define START_PAGE 0xB0
     Write_Command(START_HIGH_BIT);											//#define START_HIGH_BIT 0x12
     Write_Command(START_LOW_BIT);											//#define START_LOW_BIT 0x00
     for(column_number=OLED_MIN;column_number<COLUMN_MAX;column_number++)	//#define COLUMN_MAX 64
     {
        Write_Data(STATE_MIN);
     }
   }
}

/******************************************************
//
// full screen
//
******************************************************/

void All_Screen()
{
   uint8_t page_number,column_number;
   for(page_number=OLED_MIN;page_number<PAGE_TOTAL;page_number++)
   {
     Write_Command(START_PAGE+page_number);
     Write_Command(START_HIGH_BIT);
     Write_Command(START_LOW_BIT);
     for(column_number=OLED_MIN;column_number<COLUMN_MAX;column_number++)
     {
        Write_Data(STATE_MAX);
     }
   }
}

//x1,y1,x2,y2
//x1<=x2;y1<=y2 0<=x1<=63 0<=y1<=31	 	 
//dot:0,clear;1,fill	  
void OLED_Fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t dot)  
{  
	uint8_t x,y;  
	for(x=x1;x<=x2;x++)
	{
		for(y=y1;y<=y2;y++)OLED_DrawPoint(x,y,dot);
	}													    
	OLED_Refresh_Gram();
}

//DrawPoint
//x:0~63
//y:0~31
//t:1 fill; 0  clear				   
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t mode)
{
	uint8_t page, temp;
	if(x>63 || y>31)	return;
	page = y / 8;
	temp = 1 << (y%8);
	if(mode)	OLED_GRAM[page][x] |= temp;
	else 	OLED_GRAM[page][x] &= ~temp;	    
}

//Show char
//x:0~63
//y:0~31				 
//size:select the font 16/12 
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode)
{      			    
	uint8_t temp,t,t1;
	uint8_t y0=y;
	chr=chr-' ';//get the offset
	
    for(t=0;t<size;t++)
    {   
		if(size == 12)	temp = asc2_1206[chr][t];  //font:1206
		else temp=asc2_1608[chr][t];		 //font:1608
		
        for(t1=0; t1<8; t1++)
		{
			if(temp&0x80)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			
			temp<<=1;
			y++;
			if((y-y0) == size)
			{
				y=y0;
				x++;
				break;
			}
		}  	 
    }          
}

//fuction:m^n
uint32_t mypow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}	

//display a number
//x,y :address	 
//len :length of the number
//size:size of the font
//num:value(0~4294967295);	 		  
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size)
{         	 
	uint8_t t,temp;
	uint8_t enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,1); 
	}
}


/******************************************************
// #define OLED_MIN 0
// #define PAGE_TOTAL 4
// #define COLUMN_MAX 64
// #define ROW_MAX 32
******************************************************/
void OLED_Refresh_Gram(void)
{
	uint8_t page_number,column_number;
	
	for(page_number=0; page_number<PAGE_TOTAL; ++page_number)
	{
		Write_Command(START_PAGE + page_number);
		Write_Command(START_HIGH_BIT);
		Write_Command(START_LOW_BIT);
		
		for(column_number=OLED_MIN; column_number<COLUMN_MAX; ++column_number)
		{
			Write_Data(OLED_GRAM[page_number][column_number]);		
		}
	} 
}

void oled_display(const uint8_t * pic)
{
   uint8_t page_number,column_number;
   uint32_t piexl=OLED_MIN;
   for(page_number=OLED_MIN;page_number<PAGE_TOTAL;page_number++)
   {
      Write_Command(START_PAGE+page_number);
      Write_Command(START_HIGH_BIT);
      Write_Command(START_LOW_BIT);
      for(column_number=OLED_MIN;column_number<COLUMN_MAX;column_number++)
      {
		 Write_Data(pic[piexl]);
		 piexl++;
	  }
  }
}


void OLED_ShowString(uint8_t x,uint8_t y,const uint8_t *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58          
    while(*p!='\0')
    {       
        if(x>MAX_CHAR_POSX){x=0;y+=12;}
        if(y>MAX_CHAR_POSY){y=x=0;OLED_Clear();}
        OLED_ShowChar(x,y,*p,12,1);	 
        x+=8;
        p++;
    }  
}



void oled_test(void)
{
   //Reset_IC();                    
	Init_IC();                      
	Clear_Screen();
	All_Screen();					 
	

	OLED_Clear();
	OLED_ShowNum(0, 0, 2567890134, 10, 12);
	OLED_ShowString(0, 12, "abcdefghijk");
	
	while(1)
	{       
		OLED_Refresh_Gram();
		nrf_delay_ms(1500);
		Clear_Screen();

	}
}


/***********************************************************************/

