#line 1 "..\\..\\..\\Source\\rtc\\timer_rtc.c"
#line 1 "..\\..\\..\\Include\\nrf.h"




























 





 
#line 1 "..\\..\\..\\Include\\nrf51.h"

 








































 





 



 









 

typedef enum {
 
  Reset_IRQn                    = -15,               
  NonMaskableInt_IRQn           = -14,               
  HardFault_IRQn                = -13,               
  SVCall_IRQn                   =  -5,               
  DebugMonitor_IRQn             =  -4,               
  PendSV_IRQn                   =  -2,               
  SysTick_IRQn                  =  -1,               
 
  POWER_CLOCK_IRQn              =   0,               
  RADIO_IRQn                    =   1,               
  UART0_IRQn                    =   2,               
  SPI0_TWI0_IRQn                =   3,               
  SPI1_TWI1_IRQn                =   4,               
  GPIOTE_IRQn                   =   6,               
  ADC_IRQn                      =   7,               
  TIMER0_IRQn                   =   8,               
  TIMER1_IRQn                   =   9,               
  TIMER2_IRQn                   =  10,               
  RTC0_IRQn                     =  11,               
  TEMP_IRQn                     =  12,               
  RNG_IRQn                      =  13,               
  ECB_IRQn                      =  14,               
  CCM_AAR_IRQn                  =  15,               
  WDT_IRQn                      =  16,               
  RTC1_IRQn                     =  17,               
  QDEC_IRQn                     =  18,               
  LPCOMP_IRQn                   =  19,               
  SWI0_IRQn                     =  20,               
  SWI1_IRQn                     =  21,               
  SWI2_IRQn                     =  22,               
  SWI3_IRQn                     =  23,               
  SWI4_IRQn                     =  24,               
  SWI5_IRQn                     =  25                
} IRQn_Type;




 


 
 
 

 




   

#line 1 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"
 




















 






















 




 


 

 













#line 85 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"


 







#line 110 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"

#line 1 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"
 
 





 










#line 26 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"







 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 197 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"

     







     










     











#line 261 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"



 



#line 112 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"
#line 1 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cmInstr.h"
 




















 





 



 


 









 







 







 






 








 







 







 









 









 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}










 



#line 267 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cmInstr.h"



#line 618 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cmInstr.h"

   

#line 113 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"
#line 1 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cmFunc.h"
 




















 





 



 


 





 
 






 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}







 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}







 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}







 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}







 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}







 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}







 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}







 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}


#line 257 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cmFunc.h"


#line 293 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cmFunc.h"


#line 612 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cmFunc.h"

 


#line 114 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"








 
#line 139 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"

 






 
#line 155 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"

 










 


 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       

    uint32_t _reserved0:15;               





    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[1];                  
       uint32_t RESERVED0[31];
  volatile uint32_t ICER[1];                  
       uint32_t RSERVED1[31];
  volatile uint32_t ISPR[1];                  
       uint32_t RESERVED2[31];
  volatile uint32_t ICPR[1];                  
       uint32_t RESERVED3[31];
       uint32_t RESERVED4[64];
  volatile uint32_t IP[8];                    
}  NVIC_Type;

 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
       uint32_t RESERVED0;
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
       uint32_t RESERVED1;
  volatile uint32_t SHP[2];                   
  volatile uint32_t SHCSR;                    
} SCB_Type;

 















 



























 















 









 






 



 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 








 
 






 

 










 









 

 



 




 

 
 










 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}











 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}







 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] = (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] = (((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
}












 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 2)));  }  
  else {
    return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( ((uint32_t)(IRQn) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 2)));  }  
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FA << 16)      |
                 (1UL << 2));
  __dsb(0xF);                                                      
  while(1);                                                     
}

 



 




 

















 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1) > (0xFFFFFFUL << 0))  return (1);       

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = ticks - 1;                                   
  NVIC_SetPriority (SysTick_IRQn, (1<<2) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL << 0);                     
  return (0);                                                   
}



 








#line 120 "..\\..\\..\\Include\\nrf51.h"
#line 1 "..\\..\\..\\Include\\system_nrf51.h"




























 







#line 38 "..\\..\\..\\Include\\system_nrf51.h"


extern uint32_t SystemCoreClock;     









 
extern void SystemInit (void);









 
extern void SystemCoreClockUpdate (void);





#line 121 "..\\..\\..\\Include\\nrf51.h"


 
 
 




 


 

  #pragma push
  #pragma anon_unions
#line 148 "..\\..\\..\\Include\\nrf51.h"


typedef struct {
  volatile uint32_t  CPU0;                               
  volatile uint32_t  SPIS1;                              
  volatile uint32_t  RADIO;                              
  volatile uint32_t  ECB;                                
  volatile uint32_t  CCM;                                
  volatile uint32_t  AAR;                                
} AMLI_RAMPRI_Type;

typedef struct {
  volatile  uint32_t  EN;                                 
  volatile  uint32_t  DIS;                                
} PPI_TASKS_CHG_Type;

typedef struct {
  volatile uint32_t  EEP;                                
  volatile uint32_t  TEP;                                
} PPI_CH_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[30];
  volatile  uint32_t  TASKS_CONSTLAT;                     
  volatile  uint32_t  TASKS_LOWPWR;                       
  volatile const  uint32_t  RESERVED1[34];
  volatile uint32_t  EVENTS_POFWARN;                     
  volatile const  uint32_t  RESERVED2[126];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[61];
  volatile uint32_t  RESETREAS;                          
  volatile const  uint32_t  RESERVED4[9];
  volatile const  uint32_t  RAMSTATUS;                          
  volatile const  uint32_t  RESERVED5[53];
  volatile  uint32_t  SYSTEMOFF;                          
  volatile const  uint32_t  RESERVED6[3];
  volatile uint32_t  POFCON;                             
  volatile const  uint32_t  RESERVED7[2];
  volatile uint32_t  GPREGRET;                          
 
  volatile const  uint32_t  RESERVED8;
  volatile uint32_t  RAMON;                              
  volatile const  uint32_t  RESERVED9[7];
  volatile uint32_t  RESET;                             
 
  volatile const  uint32_t  RESERVED10[3];
  volatile uint32_t  RAMONB;                             
  volatile const  uint32_t  RESERVED11[8];
  volatile uint32_t  DCDCEN;                             
  volatile const  uint32_t  RESERVED12[291];
  volatile uint32_t  DCDCFORCE;                          
} NRF_POWER_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_HFCLKSTART;                   
  volatile  uint32_t  TASKS_HFCLKSTOP;                    
  volatile  uint32_t  TASKS_LFCLKSTART;                   
  volatile  uint32_t  TASKS_LFCLKSTOP;                    
  volatile  uint32_t  TASKS_CAL;                          
  volatile  uint32_t  TASKS_CTSTART;                      
  volatile  uint32_t  TASKS_CTSTOP;                       
  volatile const  uint32_t  RESERVED0[57];
  volatile uint32_t  EVENTS_HFCLKSTARTED;                
  volatile uint32_t  EVENTS_LFCLKSTARTED;                
  volatile const  uint32_t  RESERVED1;
  volatile uint32_t  EVENTS_DONE;                        
  volatile uint32_t  EVENTS_CTTO;                        
  volatile const  uint32_t  RESERVED2[124];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[63];
  volatile const  uint32_t  HFCLKRUN;                           
  volatile const  uint32_t  HFCLKSTAT;                          
  volatile const  uint32_t  RESERVED4;
  volatile const  uint32_t  LFCLKRUN;                           
  volatile const  uint32_t  LFCLKSTAT;                          
  volatile const  uint32_t  LFCLKSRCCOPY;                      
 
  volatile const  uint32_t  RESERVED5[62];
  volatile uint32_t  LFCLKSRC;                           
  volatile const  uint32_t  RESERVED6[7];
  volatile uint32_t  CTIV;                               
  volatile const  uint32_t  RESERVED7[5];
  volatile uint32_t  XTALFREQ;                           
} NRF_CLOCK_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[330];
  volatile uint32_t  PERR0;                              
  volatile uint32_t  RLENR0;                             
  volatile const  uint32_t  RESERVED1[52];
  volatile uint32_t  PROTENSET0;                         
  volatile uint32_t  PROTENSET1;                         
  volatile uint32_t  DISABLEINDEBUG;                     
  volatile uint32_t  PROTBLOCKSIZE;                      
} NRF_MPU_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[448];
  volatile uint32_t  REPLACEADDR[8];                     
  volatile const  uint32_t  RESERVED1[24];
  volatile uint32_t  PATCHADDR[8];                       
  volatile const  uint32_t  RESERVED2[24];
  volatile uint32_t  PATCHEN;                            
  volatile uint32_t  PATCHENSET;                         
  volatile uint32_t  PATCHENCLR;                         
} NRF_PU_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[896];
  AMLI_RAMPRI_Type RAMPRI;                           
} NRF_AMLI_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_TXEN;                         
  volatile  uint32_t  TASKS_RXEN;                         
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_DISABLE;                      
  volatile  uint32_t  TASKS_RSSISTART;                    
  volatile  uint32_t  TASKS_RSSISTOP;                     
  volatile  uint32_t  TASKS_BCSTART;                      
  volatile  uint32_t  TASKS_BCSTOP;                       
  volatile const  uint32_t  RESERVED0[55];
  volatile uint32_t  EVENTS_READY;                       
  volatile uint32_t  EVENTS_ADDRESS;                     
  volatile uint32_t  EVENTS_PAYLOAD;                     
  volatile uint32_t  EVENTS_END;                         
  volatile uint32_t  EVENTS_DISABLED;                    
  volatile uint32_t  EVENTS_DEVMATCH;                    
  volatile uint32_t  EVENTS_DEVMISS;                     
  volatile uint32_t  EVENTS_RSSIEND;                    
 
  volatile const  uint32_t  RESERVED1[2];
  volatile uint32_t  EVENTS_BCMATCH;                     
  volatile const  uint32_t  RESERVED2[53];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED3[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED4[61];
  volatile const  uint32_t  CRCSTATUS;                          
  volatile const  uint32_t  CD;                                 
  volatile const  uint32_t  RXMATCH;                            
  volatile const  uint32_t  RXCRC;                              
  volatile const  uint32_t  DAI;                                
  volatile const  uint32_t  RESERVED5[60];
  volatile uint32_t  PACKETPTR;                          
  volatile uint32_t  FREQUENCY;                          
  volatile uint32_t  TXPOWER;                            
  volatile uint32_t  MODE;                               
  volatile uint32_t  PCNF0;                              
  volatile uint32_t  PCNF1;                              
  volatile uint32_t  BASE0;                              
  volatile uint32_t  BASE1;                              
  volatile uint32_t  PREFIX0;                            
  volatile uint32_t  PREFIX1;                            
  volatile uint32_t  TXADDRESS;                          
  volatile uint32_t  RXADDRESSES;                        
  volatile uint32_t  CRCCNF;                             
  volatile uint32_t  CRCPOLY;                            
  volatile uint32_t  CRCINIT;                            
  volatile uint32_t  TEST;                               
  volatile uint32_t  TIFS;                               
  volatile const  uint32_t  RSSISAMPLE;                         
  volatile const  uint32_t  RESERVED6;
  volatile const  uint32_t  STATE;                              
  volatile uint32_t  DATAWHITEIV;                        
  volatile const  uint32_t  RESERVED7[2];
  volatile uint32_t  BCC;                                
  volatile const  uint32_t  RESERVED8[39];
  volatile uint32_t  DAB[8];                             
  volatile uint32_t  DAP[8];                             
  volatile uint32_t  DACNF;                              
  volatile const  uint32_t  RESERVED9[56];
  volatile uint32_t  OVERRIDE0;                          
  volatile uint32_t  OVERRIDE1;                          
  volatile uint32_t  OVERRIDE2;                          
  volatile uint32_t  OVERRIDE3;                          
  volatile uint32_t  OVERRIDE4;                          
  volatile const  uint32_t  RESERVED10[561];
  volatile uint32_t  POWER;                              
} NRF_RADIO_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_STARTRX;                      
  volatile  uint32_t  TASKS_STOPRX;                       
  volatile  uint32_t  TASKS_STARTTX;                      
  volatile  uint32_t  TASKS_STOPTX;                       
  volatile const  uint32_t  RESERVED0[3];
  volatile  uint32_t  TASKS_SUSPEND;                      
  volatile const  uint32_t  RESERVED1[56];
  volatile uint32_t  EVENTS_CTS;                         
  volatile uint32_t  EVENTS_NCTS;                        
  volatile uint32_t  EVENTS_RXDRDY;                      
  volatile const  uint32_t  RESERVED2[4];
  volatile uint32_t  EVENTS_TXDRDY;                      
  volatile const  uint32_t  RESERVED3;
  volatile uint32_t  EVENTS_ERROR;                       
  volatile const  uint32_t  RESERVED4[7];
  volatile uint32_t  EVENTS_RXTO;                        
  volatile const  uint32_t  RESERVED5[46];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED6[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED7[93];
  volatile uint32_t  ERRORSRC;                           
  volatile const  uint32_t  RESERVED8[31];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED9;
  volatile uint32_t  PSELRTS;                            
  volatile uint32_t  PSELTXD;                            
  volatile uint32_t  PSELCTS;                            
  volatile uint32_t  PSELRXD;                            
  volatile const  uint32_t  RXD;                               

 
  volatile  uint32_t  TXD;                                
  volatile const  uint32_t  RESERVED10;
  volatile uint32_t  BAUDRATE;                           
  volatile const  uint32_t  RESERVED11[17];
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED12[675];
  volatile uint32_t  POWER;                              
} NRF_UART_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[66];
  volatile uint32_t  EVENTS_READY;                       
  volatile const  uint32_t  RESERVED1[126];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[125];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED3;
  volatile uint32_t  PSELSCK;                            
  volatile uint32_t  PSELMOSI;                           
  volatile uint32_t  PSELMISO;                           
  volatile const  uint32_t  RESERVED4;
  volatile const  uint32_t  RXD;                                
  volatile uint32_t  TXD;                                
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  FREQUENCY;                          
  volatile const  uint32_t  RESERVED6[11];
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED7[681];
  volatile uint32_t  POWER;                              
} NRF_SPI_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_STARTRX;                      
  volatile const  uint32_t  RESERVED0;
  volatile  uint32_t  TASKS_STARTTX;                      
  volatile const  uint32_t  RESERVED1[2];
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED2;
  volatile  uint32_t  TASKS_SUSPEND;                      
  volatile  uint32_t  TASKS_RESUME;                       
  volatile const  uint32_t  RESERVED3[56];
  volatile uint32_t  EVENTS_STOPPED;                     
  volatile uint32_t  EVENTS_RXDREADY;                    
  volatile const  uint32_t  RESERVED4[4];
  volatile uint32_t  EVENTS_TXDSENT;                     
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  EVENTS_ERROR;                       
  volatile const  uint32_t  RESERVED6[4];
  volatile uint32_t  EVENTS_BB;                          
  volatile const  uint32_t  RESERVED7[3];
  volatile uint32_t  EVENTS_SUSPENDED;                   
  volatile const  uint32_t  RESERVED8[45];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED9[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED10[110];
  volatile uint32_t  ERRORSRC;                           
  volatile const  uint32_t  RESERVED11[14];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED12;
  volatile uint32_t  PSELSCL;                            
  volatile uint32_t  PSELSDA;                            
  volatile const  uint32_t  RESERVED13[2];
  volatile const  uint32_t  RXD;                                
  volatile uint32_t  TXD;                                
  volatile const  uint32_t  RESERVED14;
  volatile uint32_t  FREQUENCY;                          
  volatile const  uint32_t  RESERVED15[24];
  volatile uint32_t  ADDRESS;                            
  volatile const  uint32_t  RESERVED16[668];
  volatile uint32_t  POWER;                              
} NRF_TWI_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[9];
  volatile  uint32_t  TASKS_ACQUIRE;                      
  volatile  uint32_t  TASKS_RELEASE;                      
  volatile const  uint32_t  RESERVED1[54];
  volatile uint32_t  EVENTS_END;                         
  volatile const  uint32_t  RESERVED2[8];
  volatile uint32_t  EVENTS_ACQUIRED;                    
  volatile const  uint32_t  RESERVED3[53];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED4[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED5[61];
  volatile const  uint32_t  SEMSTAT;                            
  volatile const  uint32_t  RESERVED6[15];
  volatile uint32_t  STATUS;                             
  volatile const  uint32_t  RESERVED7[47];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED8;
  volatile uint32_t  PSELSCK;                            
  volatile uint32_t  PSELMISO;                           
  volatile uint32_t  PSELMOSI;                           
  volatile uint32_t  PSELCSN;                            
  volatile const  uint32_t  RESERVED9[7];
  volatile uint32_t  RXDPTR;                             
  volatile uint32_t  MAXRX;                              
  volatile const  uint32_t  AMOUNTRX;                           
  volatile const  uint32_t  RESERVED10;
  volatile uint32_t  TXDPTR;                             
  volatile uint32_t  MAXTX;                              
  volatile const  uint32_t  AMOUNTTX;                           
  volatile const  uint32_t  RESERVED11;
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED12;
  volatile uint32_t  DEF;                                
  volatile const  uint32_t  RESERVED13[24];
  volatile uint32_t  ORC;                                
  volatile const  uint32_t  RESERVED14[654];
  volatile uint32_t  POWER;                              
} NRF_SPIS_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_OUT[4];                       
  volatile const  uint32_t  RESERVED0[60];
  volatile uint32_t  EVENTS_IN[4];                       
  volatile const  uint32_t  RESERVED1[27];
  volatile uint32_t  EVENTS_PORT;                        
  volatile const  uint32_t  RESERVED2[97];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[129];
  volatile uint32_t  CONFIG[4];                          
  volatile const  uint32_t  RESERVED4[695];
  volatile uint32_t  POWER;                              
} NRF_GPIOTE_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED0[62];
  volatile uint32_t  EVENTS_END;                         
  volatile const  uint32_t  RESERVED1[128];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[61];
  volatile const  uint32_t  BUSY;                               
  volatile const  uint32_t  RESERVED3[63];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESULT;                             
  volatile const  uint32_t  RESERVED4[700];
  volatile uint32_t  POWER;                              
} NRF_ADC_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_COUNT;                        
  volatile  uint32_t  TASKS_CLEAR;                        
  volatile  uint32_t  TASKS_SHUTDOWN;                     
  volatile const  uint32_t  RESERVED0[11];
  volatile  uint32_t  TASKS_CAPTURE[4];                   
  volatile const  uint32_t  RESERVED1[60];
  volatile uint32_t  EVENTS_COMPARE[4];                  
  volatile const  uint32_t  RESERVED2[44];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED3[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED4[126];
  volatile uint32_t  MODE;                               
  volatile uint32_t  BITMODE;                            
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  PRESCALER;                         
 
  volatile const  uint32_t  RESERVED6[11];
  volatile uint32_t  CC[4];                              
  volatile const  uint32_t  RESERVED7[683];
  volatile uint32_t  POWER;                              
} NRF_TIMER_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_CLEAR;                        
  volatile  uint32_t  TASKS_TRIGOVRFLW;                   
  volatile const  uint32_t  RESERVED0[60];
  volatile uint32_t  EVENTS_TICK;                        
  volatile uint32_t  EVENTS_OVRFLW;                      
  volatile const  uint32_t  RESERVED1[14];
  volatile uint32_t  EVENTS_COMPARE[4];                  
  volatile const  uint32_t  RESERVED2[109];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[13];
  volatile uint32_t  EVTEN;                              
  volatile uint32_t  EVTENSET;                          
 
  volatile uint32_t  EVTENCLR;                          
 
  volatile const  uint32_t  RESERVED4[110];
  volatile const  uint32_t  COUNTER;                            
  volatile uint32_t  PRESCALER;                         
 
  volatile const  uint32_t  RESERVED5[13];
  volatile uint32_t  CC[4];                              
  volatile const  uint32_t  RESERVED6[683];
  volatile uint32_t  POWER;                              
} NRF_RTC_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED0[62];
  volatile uint32_t  EVENTS_DATARDY;                     
  volatile const  uint32_t  RESERVED1[128];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[127];
  volatile const  int32_t   TEMP;                               
  volatile const  uint32_t  RESERVED3[700];
  volatile uint32_t  POWER;                              
} NRF_TEMP_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED0[62];
  volatile uint32_t  EVENTS_VALRDY;                      
  volatile const  uint32_t  RESERVED1[63];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED2[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[126];
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  VALUE;                              
  volatile const  uint32_t  RESERVED4[700];
  volatile uint32_t  POWER;                              
} NRF_RNG_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_STARTECB;                    

 
  volatile  uint32_t  TASKS_STOPECB;                     
 
  volatile const  uint32_t  RESERVED0[62];
  volatile uint32_t  EVENTS_ENDECB;                      
  volatile uint32_t  EVENTS_ERRORECB;                   
 
  volatile const  uint32_t  RESERVED1[127];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[126];
  volatile uint32_t  ECBDATAPTR;                         
  volatile const  uint32_t  RESERVED3[701];
  volatile uint32_t  POWER;                              
} NRF_ECB_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                       
 
  volatile const  uint32_t  RESERVED0;
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED1[61];
  volatile uint32_t  EVENTS_END;                         
  volatile uint32_t  EVENTS_RESOLVED;                    
  volatile uint32_t  EVENTS_NOTRESOLVED;                 
  volatile const  uint32_t  RESERVED2[126];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[61];
  volatile const  uint32_t  STATUS;                             
  volatile const  uint32_t  RESERVED4[63];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  NIRK;                               
  volatile uint32_t  IRKPTR;                             
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  ADDRPTR;                            
  volatile uint32_t  SCRATCHPTR;                        
 
  volatile const  uint32_t  RESERVED6[697];
  volatile uint32_t  POWER;                              
} NRF_AAR_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_KSGEN;                       
 
  volatile  uint32_t  TASKS_CRYPT;                       
 
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED0[61];
  volatile uint32_t  EVENTS_ENDKSGEN;                    
  volatile uint32_t  EVENTS_ENDCRYPT;                    
  volatile uint32_t  EVENTS_ERROR;                       
  volatile const  uint32_t  RESERVED1[61];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED2[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[61];
  volatile const  uint32_t  MICSTATUS;                          
  volatile const  uint32_t  RESERVED4[63];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  MODE;                               
  volatile uint32_t  CNFPTR;                             
  volatile uint32_t  INPTR;                              
  volatile uint32_t  OUTPTR;                             
  volatile uint32_t  SCRATCHPTR;                        
 
  volatile const  uint32_t  RESERVED5[697];
  volatile uint32_t  POWER;                              
} NRF_CCM_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile const  uint32_t  RESERVED0[63];
  volatile uint32_t  EVENTS_TIMEOUT;                     
  volatile const  uint32_t  RESERVED1[128];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[61];
  volatile const  uint32_t  RUNSTATUS;                          
  volatile const  uint32_t  REQSTATUS;                          
  volatile const  uint32_t  RESERVED3[63];
  volatile uint32_t  CRV;                                
  volatile uint32_t  RREN;                               
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED4[60];
  volatile  uint32_t  RR[8];                              
  volatile const  uint32_t  RESERVED5[631];
  volatile uint32_t  POWER;                              
} NRF_WDT_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_READCLRACC;                  
 
  volatile const  uint32_t  RESERVED0[61];
  volatile uint32_t  EVENTS_SAMPLERDY;                   
  volatile uint32_t  EVENTS_REPORTRDY;                  
 
  volatile uint32_t  EVENTS_ACCOF;                       
  volatile const  uint32_t  RESERVED1[61];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED2[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[125];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  LEDPOL;                             
  volatile uint32_t  SAMPLEPER;                          
  volatile const  int32_t   SAMPLE;                             
  volatile uint32_t  REPORTPER;                          
  volatile const  int32_t   ACC;                                
  volatile const  int32_t   ACCREAD;                           
 
  volatile uint32_t  PSELLED;                            
  volatile uint32_t  PSELA;                              
  volatile uint32_t  PSELB;                              
  volatile uint32_t  DBFEN;                              
  volatile const  uint32_t  RESERVED4[5];
  volatile uint32_t  LEDPRE;                             
  volatile const  uint32_t  ACCDBL;                             
  volatile const  uint32_t  ACCDBLREAD;                        
 
  volatile const  uint32_t  RESERVED5[684];
  volatile uint32_t  POWER;                              
} NRF_QDEC_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_SAMPLE;                       
  volatile const  uint32_t  RESERVED0[61];
  volatile uint32_t  EVENTS_READY;                       
  volatile uint32_t  EVENTS_DOWN;                        
  volatile uint32_t  EVENTS_UP;                          
  volatile uint32_t  EVENTS_CROSS;                       
  volatile const  uint32_t  RESERVED1[60];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED2[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[61];
  volatile const  uint32_t  RESULT;                             
  volatile const  uint32_t  RESERVED4[63];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  PSEL;                               
  volatile uint32_t  REFSEL;                             
  volatile uint32_t  EXTREFSEL;                          
  volatile const  uint32_t  RESERVED5[4];
  volatile uint32_t  ANADETECT;                          
  volatile const  uint32_t  RESERVED6[694];
  volatile uint32_t  POWER;                              
} NRF_LPCOMP_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  UNUSED;                             
} NRF_SWI_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[256];
  volatile const  uint32_t  READY;                              
  volatile const  uint32_t  RESERVED1[64];
  volatile uint32_t  CONFIG;                             
  volatile uint32_t  ERASEPAGE;                          
  volatile uint32_t  ERASEALL;                           
  volatile uint32_t  ERASEPROTECTEDPAGE;                 
  volatile uint32_t  ERASEUICR;                          
} NRF_NVMC_Type;


 
 
 




 

typedef struct {                                     
  PPI_TASKS_CHG_Type TASKS_CHG[4];                   
  volatile const  uint32_t  RESERVED0[312];
  volatile uint32_t  CHEN;                               
  volatile uint32_t  CHENSET;                            
  volatile uint32_t  CHENCLR;                            
  volatile const  uint32_t  RESERVED1;
  PPI_CH_Type CH[16];                                
  volatile const  uint32_t  RESERVED2[156];
  volatile uint32_t  CHG[4];                             
} NRF_PPI_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[4];
  volatile const  uint32_t  CODEPAGESIZE;                       
  volatile const  uint32_t  CODESIZE;                           
  volatile const  uint32_t  RESERVED1[4];
  volatile const  uint32_t  CLENR0;                             
  volatile const  uint32_t  PPFC;                               
  volatile const  uint32_t  RESERVED2;
  volatile const  uint32_t  NUMRAMBLOCK;                        
  
  union {
    volatile const  uint32_t  SIZERAMBLOCK[4];                 

 
    volatile const  uint32_t  SIZERAMBLOCKS;                    
  };
  volatile const  uint32_t  RESERVED3[5];
  volatile const  uint32_t  CONFIGID;                           
  volatile const  uint32_t  DEVICEID[2];                        
  volatile const  uint32_t  RESERVED4[6];
  volatile const  uint32_t  ER[4];                              
  volatile const  uint32_t  IR[4];                              
  volatile const  uint32_t  DEVICEADDRTYPE;                     
  volatile const  uint32_t  DEVICEADDR[2];                      
  volatile const  uint32_t  OVERRIDEEN;                         
  volatile const  uint32_t  NRF_1MBIT[5];                      
 
  volatile const  uint32_t  RESERVED5[10];
  volatile const  uint32_t  BLE_1MBIT[5];                      
 
} NRF_FICR_Type;


 
 
 




 

typedef struct {                                     
  volatile uint32_t  CLENR0;                             
  volatile uint32_t  RBPCONF;                            
  volatile uint32_t  XTALFREQ;                           
  volatile const  uint32_t  RESERVED0;
  volatile const  uint32_t  FWID;                               
  volatile uint32_t  BOOTLOADERADDR;                     
} NRF_UICR_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[321];
  volatile uint32_t  OUT;                                
  volatile uint32_t  OUTSET;                             
  volatile uint32_t  OUTCLR;                             
  volatile const  uint32_t  IN;                                 
  volatile uint32_t  DIR;                                
  volatile uint32_t  DIRSET;                             
  volatile uint32_t  DIRCLR;                             
  volatile const  uint32_t  RESERVED1[120];
  volatile uint32_t  PIN_CNF[32];                        
} NRF_GPIO_Type;


 

  #pragma pop
#line 1138 "..\\..\\..\\Include\\nrf51.h"




 
 
 

#line 1179 "..\\..\\..\\Include\\nrf51.h"


 
 
 

#line 1218 "..\\..\\..\\Include\\nrf51.h"


   
   
   








#line 38 "..\\..\\..\\Include\\nrf.h"
#line 1 "..\\..\\..\\Include\\nrf51_bitfields.h"




























 



 

#line 1 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"
 




















 








#line 116 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"



#line 662 "d:\\Keil\\ARM\\CMSIS\\Include\\core_cm0.h"



#line 36 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
 

 
 

 






 






 






 
 

 






 






 






 
 

 



 
 

 





 
 

 



 
 

 






 
 

 
 

 






 
 

 






 
 

 





 
 

 





 
 

 






 
#line 184 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 192 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 201 "..\\..\\..\\Include\\nrf51_bitfields.h"

 






 
 

 



 
 

 






 
 

 
 

 
#line 243 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 255 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 267 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 279 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 291 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 303 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 315 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 327 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
 

 
#line 342 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 354 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 366 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 378 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 390 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 402 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 414 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 426 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
 

 
#line 441 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 453 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 465 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 477 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 489 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 501 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 513 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 525 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
 

 
#line 540 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 552 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 564 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 576 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 588 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 600 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 612 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 624 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
 

 
#line 639 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 651 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 663 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 675 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 687 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 699 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 711 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 723 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
 

 
#line 738 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 750 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 762 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 774 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 786 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 798 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 810 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
#line 822 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
 

 
 

 





 
 

 






 






 






 
 

 






 






 






 
 

 





 
 

 





 
 

 





 
 

 






 
 

 
 

 






 






 






 






 
 

 






 






 






 






 
 

 





 
 

 





 





 
 

 





 
 

 





 






 
 

 






 
 

 






 
 

 



 
 

 






 
 

 
 

 






 






 
 

 






 






 
 

 






 
 

 
 

 





 
 

 



 



 
 

 





 
 

 





 






 
 

 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 
#line 2683 "..\\..\\..\\Include\\nrf51_bitfields.h"

 






 





 






 
 

 
 

 






 






 






 






 






 
 

 






 






 






 






 






 
 

 





 






 



 






 
 

 






 
 

 
 

 





 





 





 





 





 
 

 






 






 






 






 
 

 






 






 






 






 
 

 





 
 

 





 
 

 
#line 2950 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
 

 
#line 2965 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
 

 





 
 

 






 
 

 






 
 

 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 





 
 

 





 
 

 
 

 





 
 

 






 
 

 





 
 

 






 
 

 
 

 






 
 

 






 
 

 



 



 



 



 



 



 



 
 

 





 





 





 





 
 

 




 
 

 
#line 3739 "..\\..\\..\\Include\\nrf51_bitfields.h"

 





 
 

 



 
 

 





 





 





 





 
 

 





 
 

 





 





 





 





 
 

 





 
 

 





 






 
 

 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 






 
 

 
 

 



 
 

 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 







 
 

 
 

 





 





 
 

 






 






 






 
 

 






 






 






 
 

 





 
 

 





 
 

 
#line 4863 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 
#line 4885 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
 

 





 
 

 



 
 

 



 
 

 



 
 

 






 
 

 
 

 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 
 

 





 
 

 



 
 

 



 
 

 



 
 

 



 
 

 



 
 

 
#line 5170 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
 

 
#line 5181 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
 

 



 



 



 
 

 





 





 



 



 



 
 

 



 



 



 



 
 

 



 



 



 



 
 

 



 
 

 





 





 





 





 





 





 





 





 
 

 





 
#line 5336 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 



 
 

 





 





 
 

 



 
 

 



 
 

 
#line 5395 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 



 
 

 



 



 



 



 



 



 



 



 





 





 





 





 





 





 





 





 
 

 



 
 

 



 
 

 



 
 

 



 
 

 





 



 
 

 






 
 

 
 

 





 
 

 






 
 

 






 
 

 





 
 

 



 
 

 






 
 

 
 

 






 






 






 






 






 






 
 

 






 






 






 






 






 






 
 

 





 





 





 





 





 





 
 

 






 






 






 






 






 






 
 

 






 






 






 






 






 






 
 

 



 
 

 



 
 

 



 
 

 






 
 

 
 

 






 
 

 






 
 

 





 
 

 



 
 

 



 
 

 
#line 5914 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
 

 





 





 





 
 

 






 
 

 
 

 





 
 

 






 






 
 

 






 






 
 

 
#line 6002 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
 

 






 






 
 

 





 
 

 



 
 

 



 
 

 



 
 

 



 
 

 





 





 





 
 

 



 
 

 



 
 

 






 
 

 
 

 






 
 

 






 
 

 






 
 

 
 

 





 





 





 





 





 





 





 





 
 

 






 






 






 






 
 

 






 






 






 






 
 

 





 
 

 
#line 6270 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 






 
 

 
 

 





 





 
 

 






 






 






 






 






 






 
 

 






 






 






 






 






 






 
 

 






 






 
 

 





 
 

 



 
 

 



 
 

 






 
 

 



 
 

 






 
 

 
 

 





 





 
 

 






 






 






 






 






 






 
 

 






 






 






 






 






 






 
 

 






 






 






 






 
 

 





 
 

 



 
 

 



 
 

 
#line 6647 "..\\..\\..\\Include\\nrf51_bitfields.h"

 
 

 





 





 
 

 






 
 

 
 

 





 





 
 

 





 
 

 




 
 

 
 

 






 
 

 






 
 

 





 
 

 





 





 





 





 





 





 





 





 
 

 





 





 





 





 





 





 





 





 
 

 





 





 
 

 




 
 

 






 
#line 39 "..\\..\\..\\Include\\nrf.h"
#line 1 "..\\..\\..\\Include\\nrf51_deprecated.h"




























 



 




 

 
 

 

 
 
 

 
 
 
 




 
 
 
 




 




 




 





 
 
 

 




 




 






 
 




 


 




 




 




 
 
#line 136 "..\\..\\..\\Include\\nrf51_deprecated.h"
 
#line 169 "..\\..\\..\\Include\\nrf51_deprecated.h"
 




 
#line 431 "..\\..\\..\\Include\\nrf51_deprecated.h"



 



#line 40 "..\\..\\..\\Include\\nrf.h"




#line 1 "..\\..\\..\\Include\\compiler_abstraction.h"




























 



 


    



    



    

  
#line 90 "..\\..\\..\\Include\\compiler_abstraction.h"

 

#line 45 "..\\..\\..\\Include\\nrf.h"





#line 2 "..\\..\\..\\Source\\rtc\\timer_rtc.c"
#line 1 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"
 
 
 




 
 



 













  


 








#line 46 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"


  
  typedef unsigned int size_t;










    



    typedef unsigned short wchar_t;  
#line 75 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"

typedef struct div_t { int quot, rem; } div_t;
    
typedef struct ldiv_t { long int quot, rem; } ldiv_t;
    

typedef struct lldiv_t { __int64 quot, rem; } lldiv_t;
    


#line 96 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"
   



 

   




 
#line 115 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"
   


 
extern __declspec(__nothrow) int __aeabi_MB_CUR_MAX(void);

   




 

   




 




extern __declspec(__nothrow) double atof(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int atoi(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) long int atol(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) __int64 atoll(const char *  ) __attribute__((__nonnull__(1)));
   



 


extern __declspec(__nothrow) double strtod(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

















 

extern __declspec(__nothrow) float strtof(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) long double strtold(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

 

extern __declspec(__nothrow) long int strtol(const char * __restrict  ,
                        char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



























 
extern __declspec(__nothrow) unsigned long int strtoul(const char * __restrict  ,
                                       char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   


























 

 
extern __declspec(__nothrow) __int64 strtoll(const char * __restrict  ,
                               char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) unsigned __int64 strtoull(const char * __restrict  ,
                                         char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) int rand(void);
   







 
extern __declspec(__nothrow) void srand(unsigned int  );
   






 

struct _rand_state { int __x[57]; };
extern __declspec(__nothrow) int _rand_r(struct _rand_state *);
extern __declspec(__nothrow) void _srand_r(struct _rand_state *, unsigned int);
struct _ANSI_rand_state { int __x[1]; };
extern __declspec(__nothrow) int _ANSI_rand_r(struct _ANSI_rand_state *);
extern __declspec(__nothrow) void _ANSI_srand_r(struct _ANSI_rand_state *, unsigned int);
   


 

extern __declspec(__nothrow) void *calloc(size_t  , size_t  );
   



 
extern __declspec(__nothrow) void free(void *  );
   





 
extern __declspec(__nothrow) void *malloc(size_t  );
   



 
extern __declspec(__nothrow) void *realloc(void *  , size_t  );
   













 

extern __declspec(__nothrow) int posix_memalign(void **  , size_t  , size_t  );
   









 

typedef int (*__heapprt)(void *, char const *, ...);
extern __declspec(__nothrow) void __heapstats(int (*  )(void *  ,
                                           char const *  , ...),
                        void *  ) __attribute__((__nonnull__(1)));
   










 
extern __declspec(__nothrow) int __heapvalid(int (*  )(void *  ,
                                           char const *  , ...),
                       void *  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) __declspec(__noreturn) void abort(void);
   







 

extern __declspec(__nothrow) int atexit(void (*  )(void)) __attribute__((__nonnull__(1)));
   




 
#line 415 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"


extern __declspec(__nothrow) __declspec(__noreturn) void exit(int  );
   












 

extern __declspec(__nothrow) __declspec(__noreturn) void _Exit(int  );
   







      

extern __declspec(__nothrow) char *getenv(const char *  ) __attribute__((__nonnull__(1)));
   









 

extern __declspec(__nothrow) int  system(const char *  );
   









 

extern  void *bsearch(const void *  , const void *  ,
              size_t  , size_t  ,
              int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,2,5)));
   












 
#line 503 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"


extern  void qsort(void *  , size_t  , size_t  ,
           int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,4)));
   









 

#line 532 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"

extern __declspec(__nothrow) __pure int abs(int  );
   



 

extern __declspec(__nothrow) __pure div_t div(int  , int  );
   









 
extern __declspec(__nothrow) __pure long int labs(long int  );
   



 




extern __declspec(__nothrow) __pure ldiv_t ldiv(long int  , long int  );
   











 







extern __declspec(__nothrow) __pure __int64 llabs(__int64  );
   



 




extern __declspec(__nothrow) __pure lldiv_t lldiv(__int64  , __int64  );
   











 
#line 613 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"



 
typedef struct __sdiv32by16 { int quot, rem; } __sdiv32by16;
typedef struct __udiv32by16 { unsigned int quot, rem; } __udiv32by16;
    
typedef struct __sdiv64by32 { int rem, quot; } __sdiv64by32;

__value_in_regs extern __declspec(__nothrow) __pure __sdiv32by16 __rt_sdiv32by16(
     int  ,
     short int  );
   

 
__value_in_regs extern __declspec(__nothrow) __pure __udiv32by16 __rt_udiv32by16(
     unsigned int  ,
     unsigned short  );
   

 
__value_in_regs extern __declspec(__nothrow) __pure __sdiv64by32 __rt_sdiv64by32(
     int  , unsigned int  ,
     int  );
   

 



 
extern __declspec(__nothrow) unsigned int __fp_status(unsigned int  , unsigned int  );
   







 























 
extern __declspec(__nothrow) int mblen(const char *  , size_t  );
   












 
extern __declspec(__nothrow) int mbtowc(wchar_t * __restrict  ,
                   const char * __restrict  , size_t  );
   















 
extern __declspec(__nothrow) int wctomb(char *  , wchar_t  );
   













 





 
extern __declspec(__nothrow) size_t mbstowcs(wchar_t * __restrict  ,
                      const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 
extern __declspec(__nothrow) size_t wcstombs(char * __restrict  ,
                      const wchar_t * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 

extern __declspec(__nothrow) void __use_realtime_heap(void);
extern __declspec(__nothrow) void __use_realtime_division(void);
extern __declspec(__nothrow) void __use_two_region_memory(void);
extern __declspec(__nothrow) void __use_no_heap(void);
extern __declspec(__nothrow) void __use_no_heap_region(void);

extern __declspec(__nothrow) char const *__C_library_version_string(void);
extern __declspec(__nothrow) int __C_library_version_number(void);











#line 867 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"


 

#line 3 "..\\..\\..\\Source\\rtc\\timer_rtc.c"
#line 1 "..\\..\\..\\Include\\boards.h"










 



#line 1 "..\\..\\..\\Include\\boards/nrf6310.h"










 



#line 1 "..\\..\\..\\Include\\nrf_gpio.h"



#line 5 "..\\..\\..\\Include\\nrf_gpio.h"
#line 6 "..\\..\\..\\Include\\nrf_gpio.h"












 




 
typedef enum
{
    NRF_GPIO_PORT_DIR_OUTPUT,       
    NRF_GPIO_PORT_DIR_INPUT         
} nrf_gpio_port_dir_t;




 
typedef enum
{
    NRF_GPIO_PIN_DIR_INPUT,   
    NRF_GPIO_PIN_DIR_OUTPUT   
} nrf_gpio_pin_dir_t;




 
typedef enum
{
    NRF_GPIO_PORT_SELECT_PORT0 = 0,           
    NRF_GPIO_PORT_SELECT_PORT1,               
    NRF_GPIO_PORT_SELECT_PORT2,               
    NRF_GPIO_PORT_SELECT_PORT3,               
} nrf_gpio_port_select_t;




 
typedef enum
{
    NRF_GPIO_PIN_NOPULL   = (0x00UL),                 
    NRF_GPIO_PIN_PULLDOWN = (0x01UL),                 
    NRF_GPIO_PIN_PULLUP   = (0x03UL),                   
} nrf_gpio_pin_pull_t;




 
typedef enum
{
    NRF_GPIO_PIN_NOSENSE    = (0x00UL),              
    NRF_GPIO_PIN_SENSE_LOW  = (0x03UL),                   
    NRF_GPIO_PIN_SENSE_HIGH = (0x02UL),                  
} nrf_gpio_pin_sense_t;











 
static __inline void nrf_gpio_range_cfg_output(uint32_t pin_range_start, uint32_t pin_range_end)
{
     
    for (; pin_range_start <= pin_range_end; pin_range_start++)
    {
        ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_range_start] = ((0x00UL) << (16UL))
                                        | ((0x00UL) << (8UL))
                                        | ((0x00UL) << (2UL))
                                        | ((1UL) << (1UL))
                                        | ((1UL) << (0UL));
    }
}













 
static __inline void nrf_gpio_range_cfg_input(uint32_t pin_range_start, uint32_t pin_range_end, nrf_gpio_pin_pull_t pull_config)
{
     
    for (; pin_range_start <= pin_range_end; pin_range_start++)
    {
        ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_range_start] = ((0x00UL) << (16UL))
                                        | ((0x00UL) << (8UL))
                                        | (pull_config << (2UL))
                                        | ((0UL) << (1UL))
                                        | ((0UL) << (0UL));
    }
}








 
static __inline void nrf_gpio_cfg_output(uint32_t pin_number)
{
     
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] = ((0x00UL) << (16UL))
                                            | ((0x00UL) << (8UL))
                                            | ((0x00UL) << (2UL))
                                            | ((1UL) << (1UL))
                                            | ((1UL) << (0UL));
}










 
static __inline void nrf_gpio_cfg_input(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config)
{
     
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] = ((0x00UL) << (16UL))
                                        | ((0x00UL) << (8UL))
                                        | (pull_config << (2UL))
                                        | ((0UL) << (1UL))
                                        | ((0UL) << (0UL));
}











 
static __inline void nrf_gpio_cfg_sense_input(uint32_t pin_number, nrf_gpio_pin_pull_t pull_config, nrf_gpio_pin_sense_t sense_config)
{
     
    ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] = (sense_config << (16UL))
                                        | ((0x00UL) << (8UL))
                                        | (pull_config << (2UL))
                                        | ((0UL) << (1UL))
                                        | ((0UL) << (0UL));
}








 
static __inline void nrf_gpio_pin_dir_set(uint32_t pin_number, nrf_gpio_pin_dir_t direction)
{
    if(direction == NRF_GPIO_PIN_DIR_INPUT)
    {
        ((NRF_GPIO_Type *) 0x50000000UL)->PIN_CNF[pin_number] =
          ((0x00UL) << (16UL))
        | ((0x00UL) << (8UL))
        | ((0x00UL) << (2UL))
        | ((0UL) << (1UL))
        | ((0UL) << (0UL));
    }
    else
    {
        ((NRF_GPIO_Type *) 0x50000000UL)->DIRSET = (1UL << pin_number);
    }
}









 
static __inline void nrf_gpio_pin_set(uint32_t pin_number)
{
    ((NRF_GPIO_Type *) 0x50000000UL)->OUTSET = (1UL << pin_number);
}









 
static __inline void nrf_gpio_pin_clear(uint32_t pin_number)
{
    ((NRF_GPIO_Type *) 0x50000000UL)->OUTCLR = (1UL << pin_number);
}









 
static __inline void nrf_gpio_pin_toggle(uint32_t pin_number)
{
    const uint32_t pin_bit   = 1UL << pin_number;
    const uint32_t pin_state = ((((NRF_GPIO_Type *) 0x50000000UL)->OUT >> pin_number) & 1UL);
    
    if (pin_state == 0)
    {
        
        ((NRF_GPIO_Type *) 0x50000000UL)->OUTSET = pin_bit;        
    }
    else
    {
        
        ((NRF_GPIO_Type *) 0x50000000UL)->OUTCLR = pin_bit;       
    }
}













 
static __inline void nrf_gpio_pin_write(uint32_t pin_number, uint32_t value)
{
    if (value == 0)
    {
        nrf_gpio_pin_clear(pin_number);
    }
    else
    {
        nrf_gpio_pin_set(pin_number);
    }
}














 
static __inline uint32_t nrf_gpio_pin_read(uint32_t pin_number)
{
    return  ((((NRF_GPIO_Type *) 0x50000000UL)->IN >> pin_number) & 1UL);
}














 
static __inline void nrf_gpio_word_byte_write(volatile uint32_t * word_address, uint8_t byte_no, uint8_t value)
{
    *((volatile uint8_t*)(word_address) + byte_no) = value;
}













 
static __inline uint8_t nrf_gpio_word_byte_read(const volatile uint32_t* word_address, uint8_t byte_no)
{
    return (*((const volatile uint8_t*)(word_address) + byte_no));
}







 
static __inline void nrf_gpio_port_dir_set(nrf_gpio_port_select_t port, nrf_gpio_port_dir_t dir)
{
    if (dir == NRF_GPIO_PORT_DIR_OUTPUT)
    {
        nrf_gpio_word_byte_write(&((NRF_GPIO_Type *) 0x50000000UL)->DIRSET, port, 0xFF);
    }
    else
    {
        nrf_gpio_range_cfg_input(port*8, (port+1)*8-1, NRF_GPIO_PIN_NOPULL);
    }
}







 
static __inline uint8_t nrf_gpio_port_read(nrf_gpio_port_select_t port)
{
    return nrf_gpio_word_byte_read(&((NRF_GPIO_Type *) 0x50000000UL)->IN, port);
}









 
static __inline void nrf_gpio_port_write(nrf_gpio_port_select_t port, uint8_t value)
{
    nrf_gpio_word_byte_write(&((NRF_GPIO_Type *) 0x50000000UL)->OUT, port, value);
}











 
static __inline void nrf_gpio_port_set(nrf_gpio_port_select_t port, uint8_t set_mask)
{
    nrf_gpio_word_byte_write(&((NRF_GPIO_Type *) 0x50000000UL)->OUTSET, port, set_mask);
}











 
static __inline void nrf_gpio_port_clear(nrf_gpio_port_select_t port, uint8_t clr_mask)
{
    nrf_gpio_word_byte_write(&((NRF_GPIO_Type *) 0x50000000UL)->OUTCLR, port, clr_mask);
}

 

#line 16 "..\\..\\..\\Include\\boards/nrf6310.h"

#line 27 "..\\..\\..\\Include\\boards/nrf6310.h"

#line 39 "..\\..\\..\\Include\\boards/nrf6310.h"































#line 84 "..\\..\\..\\Include\\boards/nrf6310.h"




#line 99 "..\\..\\..\\Include\\boards/nrf6310.h"

#line 115 "..\\..\\..\\Include\\boards/nrf6310.h"



#line 17 "..\\..\\..\\Include\\boards.h"
#line 26 "..\\..\\..\\Include\\boards.h"

#line 4 "..\\..\\..\\Source\\rtc\\timer_rtc.c"
#line 1 "..\\..\\..\\Source\\rtc\\timer_rtc.h"



#line 1 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdbool.h"
 






 





#line 25 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdbool.h"



#line 5 "..\\..\\..\\Source\\rtc\\timer_rtc.h"
#line 6 "..\\..\\..\\Source\\rtc\\timer_rtc.h"

#line 18 "..\\..\\..\\Source\\rtc\\timer_rtc.h"

typedef void (*timer1_irq_callback_t)(void);

typedef struct timer1_irq_handler_node{
    timer1_irq_callback_t func;
    uint32_t interval;
    uint32_t phase;
    struct timer1_irq_handler_node *next;
} timer1_irq_handler_node_t;

typedef struct {
    uint32_t alarm_time;
    uint8_t alarm_status;
} alarm_struct_t;

void timer1_init(void);
void timer1_suspend(void);
void timer1_resume(void);
uint32_t get_timer1_cnt(void);
void add_timer1_irq_handler(timer1_irq_handler_node_t *p);
void remove_timer_irq_handler(timer1_irq_callback_t p);

void rtc_init(void);
uint32_t get_time(void);
void set_time(uint32_t tim);
void set_alarm(uint32_t time, uint8_t type);

void rtc1_suspend(void);
void rtc1_resume(void);
void switch_real_time_counter(void);



#line 5 "..\\..\\..\\Source\\rtc\\timer_rtc.c"
#line 1 "..\\..\\..\\Include\\app_common\\app_timer.h"










 



























 




#line 45 "..\\..\\..\\Include\\app_common\\app_timer.h"
#line 46 "..\\..\\..\\Include\\app_common\\app_timer.h"
#line 1 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"
 
 
 





 






 













#line 38 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"


  
  typedef unsigned int size_t;    








 
 

 
  typedef struct __va_list __va_list;





   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 129 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 
extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 948 "d:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdio.h"



 

#line 47 "..\\..\\..\\Include\\app_common\\app_timer.h"
#line 1 "..\\..\\..\\Include\\app_common\\app_error.h"










 
 







 




#line 26 "..\\..\\..\\Include\\app_common\\app_error.h"
#line 27 "..\\..\\..\\Include\\app_common\\app_error.h"
#line 1 "..\\..\\..\\Include\\s110\\nrf_error.h"







 
 




 

 




 




 

#line 46 "..\\..\\..\\Include\\s110\\nrf_error.h"





 
#line 28 "..\\..\\..\\Include\\app_common\\app_error.h"






 
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name);




 









     
#line 60 "..\\..\\..\\Include\\app_common\\app_error.h"
    



     
#line 74 "..\\..\\..\\Include\\app_common\\app_error.h"



 
#line 48 "..\\..\\..\\Include\\app_common\\app_timer.h"
#line 1 "..\\..\\..\\Include\\app_common\\app_util.h"










 








 




#line 26 "..\\..\\..\\Include\\app_common\\app_util.h"
#line 27 "..\\..\\..\\Include\\app_common\\app_util.h"
#line 28 "..\\..\\..\\Include\\app_common\\app_util.h"

enum
{
    UNIT_0_625_MS = 625,                                 
    UNIT_1_25_MS  = 1250,                                
    UNIT_10_MS    = 10000                                
};














 








 
typedef uint8_t uint16_le_t[2];

 
typedef uint8_t uint32_le_t[4];

 
typedef struct
{
    uint16_t  size;                  
    uint8_t * p_data;                
} uint8_array_t;
    






 








 





 












 











 
static __inline uint8_t uint16_encode(uint16_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x00FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0xFF00) >> 8);
    return sizeof(uint16_t);
}
    






 
static __inline uint8_t uint32_encode(uint32_t value, uint8_t * p_encoded_data)
{
    p_encoded_data[0] = (uint8_t) ((value & 0x000000FF) >> 0);
    p_encoded_data[1] = (uint8_t) ((value & 0x0000FF00) >> 8);
    p_encoded_data[2] = (uint8_t) ((value & 0x00FF0000) >> 16);
    p_encoded_data[3] = (uint8_t) ((value & 0xFF000000) >> 24);
    return sizeof(uint32_t);
}






 
static __inline uint16_t uint16_decode(const uint8_t * p_encoded_data)
{
        return ( (((uint16_t)((uint8_t *)p_encoded_data)[0])) | 
                 (((uint16_t)((uint8_t *)p_encoded_data)[1]) << 8 ));
}






 
static __inline uint32_t uint32_decode(const uint8_t * p_encoded_data)
{
    return ( (((uint32_t)((uint8_t *)p_encoded_data)[0]) << 0)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[1]) << 8)  |
             (((uint32_t)((uint8_t *)p_encoded_data)[2]) << 16) |
             (((uint32_t)((uint8_t *)p_encoded_data)[3]) << 24 ));
}
    



















 
static __inline uint8_t battery_level_in_percent(const uint16_t mvolts)
{
    uint8_t battery_level;

    if (mvolts >= 3000)
    {
        battery_level = 100;
    }
    else if (mvolts > 2900)
    {
        battery_level = 100 - ((3000 - mvolts) * 58) / 100;
    }
    else if (mvolts > 2740)
    {
        battery_level = 42 - ((2900 - mvolts) * 24) / 160;
    }
    else if (mvolts > 2440)
    {
        battery_level = 18 - ((2740 - mvolts) * 12) / 300;
    }
    else if (mvolts > 2100)
    {
        battery_level = 6 - ((2440 - mvolts) * 6) / 340;
    }
    else
    {
        battery_level = 0;
    }

    return battery_level;
}






 
static __inline _Bool is_word_aligned(void * p)
{
    return (((uintptr_t)p & 0x03) == 0);
}



 
#line 49 "..\\..\\..\\Include\\app_common\\app_timer.h"
#line 1 "..\\..\\..\\Include\\app_common\\app_scheduler.h"










 


































 




#line 52 "..\\..\\..\\Include\\app_common\\app_scheduler.h"
#line 53 "..\\..\\..\\Include\\app_common\\app_scheduler.h"










 


            
 
typedef void (*app_sched_event_handler_t)(void * p_event_data, uint16_t event_size);












 
#line 90 "..\\..\\..\\Include\\app_common\\app_scheduler.h"


















 
uint32_t app_sched_init(uint16_t max_event_size, uint16_t queue_size, void * p_evt_buffer);





 
void app_sched_execute(void);










 
uint32_t app_sched_event_put(void *                    p_event_data,
                             uint16_t                  event_size,
                             app_sched_event_handler_t handler);



 
#line 50 "..\\..\\..\\Include\\app_common\\app_timer.h"
#line 51 "..\\..\\..\\Include\\app_common\\app_timer.h"


















 
#line 80 "..\\..\\..\\Include\\app_common\\app_timer.h"


















 



 
typedef uint32_t app_timer_id_t;

 
typedef void (*app_timer_timeout_handler_t)(void * p_context);

 
typedef uint32_t (*app_timer_evt_schedule_func_t) (app_timer_timeout_handler_t timeout_handler,
                                                   void *                      p_context);

 
typedef enum
{
    APP_TIMER_MODE_SINGLE_SHOT,                  
    APP_TIMER_MODE_REPEATED                      
} app_timer_mode_t;























 
   
#line 156 "..\\..\\..\\Include\\app_common\\app_timer.h"
























 
uint32_t app_timer_init(uint32_t                      prescaler, 
                        uint8_t                       max_timers,
                        uint8_t                       op_queues_size,
                        void *                        p_buffer,
                        app_timer_evt_schedule_func_t evt_schedule_func);















 
uint32_t app_timer_create(app_timer_id_t *            p_timer_id,
                          app_timer_mode_t            mode,
                          app_timer_timeout_handler_t timeout_handler);




















 
uint32_t app_timer_start(app_timer_id_t timer_id, uint32_t timeout_ticks, void * p_context);










 
uint32_t app_timer_stop(app_timer_id_t timer_id);






 
uint32_t app_timer_stop_all(void);






 
uint32_t app_timer_cnt_get(uint32_t * p_ticks);








 
uint32_t app_timer_cnt_diff_compute(uint32_t   ticks_to,
                                    uint32_t   ticks_from,
                                    uint32_t * p_ticks_diff);




 
typedef struct
{
    app_timer_timeout_handler_t timeout_handler;
    void *                      p_context;
} app_timer_event_t;

static __inline void app_timer_evt_get(void * p_event_data, uint16_t event_size)
{
    app_timer_event_t * p_timer_event = (app_timer_event_t *)p_event_data;
    
    do { const _Bool LOCAL_BOOLEAN_VALUE = (event_size == sizeof(app_timer_event_t)); if (!LOCAL_BOOLEAN_VALUE) { do { app_error_handler((0), 282, (uint8_t*) "..\\..\\..\\Include\\app_common\\app_timer.h"); } while (0); } } while (0);
    p_timer_event->timeout_handler(p_timer_event->p_context);
}

static __inline uint32_t app_timer_evt_schedule(app_timer_timeout_handler_t timeout_handler,
                                                void *                      p_context)
{
    app_timer_event_t timer_event;

    timer_event.timeout_handler = timeout_handler;
    timer_event.p_context       = p_context;
    
    return app_sched_event_put(&timer_event, sizeof(timer_event), app_timer_evt_get);
}
 



 
#line 6 "..\\..\\..\\Source\\rtc\\timer_rtc.c"
#line 1 "..\\..\\..\\Include\\nrf_delay.h"



#line 5 "..\\..\\..\\Include\\nrf_delay.h"

 

static __asm void __inline nrf_delay_us(uint32_t volatile number_of_us)
{
loop
        SUBS    R0, R0, #1
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        BNE    loop
        BX     LR
}
#line 71 "..\\..\\..\\Include\\nrf_delay.h"

void nrf_delay_ms(uint32_t volatile number_of_ms);

#line 7 "..\\..\\..\\Source\\rtc\\timer_rtc.c"
#line 1 "..\\..\\..\\Include\\sd_common\\app_util_platform.h"










 








 




#line 26 "..\\..\\..\\Include\\sd_common\\app_util_platform.h"
#line 27 "..\\..\\..\\Include\\sd_common\\app_util_platform.h"
#line 28 "..\\..\\..\\Include\\sd_common\\app_util_platform.h"
#line 29 "..\\..\\..\\Include\\sd_common\\app_util_platform.h"

 
typedef enum
{
    APP_IRQ_PRIORITY_HIGH = 1,
    APP_IRQ_PRIORITY_LOW  = 3
} app_irq_priority_t;



 

 








 
#line 67 "..\\..\\..\\Include\\sd_common\\app_util_platform.h"
    





 
#line 86 "..\\..\\..\\Include\\sd_common\\app_util_platform.h"
        






 
static __inline uint8_t current_int_priority_get(void)
{
    uint32_t isr_vector_num = (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->ICSR & (0x1FFUL << 0));
    if (isr_vector_num > 0)
    {
        int32_t irq_type = ((int32_t)isr_vector_num - 16);
        return (NVIC_GetPriority((IRQn_Type)irq_type) & 0xFF);
    }
    else
    {
        return 4;
    }
}



 
#line 8 "..\\..\\..\\Source\\rtc\\timer_rtc.c"

#line 1 "..\\..\\..\\Include\\xprintf.h"
 
 
 













extern void (*xfunc_out)(unsigned char);
void xputc (char c);
void xputs (const char* str);
void xfputs (void (*func)(unsigned char), const char* str);
void xprintf (const char* fmt, ...);
void xsprintf (char* buff, const char* fmt, ...);
void xfprintf (void (*func)(unsigned char), const char*	fmt, ...);
void put_dump (const void* buff, unsigned long addr, int len, int width);





#line 37 "..\\..\\..\\Include\\xprintf.h"

#line 12 "..\\..\\..\\Source\\rtc\\timer_rtc.c"






static timer1_irq_handler_node_t *timer1_irq_handler_list = 0;

static uint32_t standard_time = 0;

static alarm_struct_t alarm = {0, 0};

static uint32_t timer1_cnt = 0;
static uint8_t rtc_is_available = 1;
static uint8_t rtc_resume_cnt = 0;

uint8_t is_time_calibrated = 0;



 
void timer1_init(void)
{
    timer1_irq_handler_list = 0;
    
    
    

    
    
    
        
    

    ((NRF_TIMER_Type *) 0x40009000UL)->TASKS_STOP     = 1;
    ((NRF_TIMER_Type *) 0x40009000UL)->MODE           = (0UL);        
    ((NRF_TIMER_Type *) 0x40009000UL)->PRESCALER      = 4;                            
    ((NRF_TIMER_Type *) 0x40009000UL)->BITMODE        = (0x03UL);  
    ((NRF_TIMER_Type *) 0x40009000UL)->TASKS_CLEAR    = 1;                            









    
    ((NRF_TIMER_Type *) 0x40009000UL)->CC[0]          = 10000;               
    ((NRF_TIMER_Type *) 0x40009000UL)->INTENSET       = ((1UL) << (16UL));   
    ((NRF_TIMER_Type *) 0x40009000UL)->TASKS_START    = 1;                    
    
}

void add_timer1_irq_handler(timer1_irq_handler_node_t *p)
{
    timer1_irq_handler_node_t *q = timer1_irq_handler_list;
    
    if(q == 0)
    {
        q = p;
        timer1_irq_handler_list = p;
    } else if(q->next == 0){
        q->next = p;
    } else {
        while(q->next != 0) q = q->next;
        q->next = p;
    }
    p->next = 0;
}

void remove_timer_irq_handler(timer1_irq_callback_t p)
{
    timer1_irq_handler_node_t *q,*r = timer1_irq_handler_list;
    if(r == 0) return;
    while((r->func != p) && (r->next!=0))
    {
        q = r;
        r = r->next;
    }
    if(r == timer1_irq_handler_list)
    {
        timer1_irq_handler_list = 0;
    } else {
        q->next = r->next;
    }
}

#line 149 "..\\..\\..\\Source\\rtc\\timer_rtc.c"

uint32_t get_timer1_cnt(void)
{
    return timer1_cnt;
}

void timer1_suspend(void)
{
    NVIC_DisableIRQ(TIMER1_IRQn);

    ((NRF_TIMER_Type *) 0x40009000UL)->EVENTS_COMPARE[0] = 0;
    ((NRF_TIMER_Type *) 0x40009000UL)->INTENCLR = ((1UL) << (16UL));
    ((NRF_TIMER_Type *) 0x40009000UL)->TASKS_STOP = 1;
}

void timer1_resume(void)
{
    ((NRF_TIMER_Type *) 0x40009000UL)->TASKS_STOP     = 1;
    ((NRF_TIMER_Type *) 0x40009000UL)->TASKS_CLEAR    = 1;                            









    
    
    ((NRF_TIMER_Type *) 0x40009000UL)->INTENSET       = ((1UL) << (16UL));   
    ((NRF_TIMER_Type *) 0x40009000UL)->TASKS_START    = 1;                    
}



void rtc_init()
{
    ((NRF_CLOCK_Type *) 0x40000000UL)->LFCLKSRC             = ((1UL) << (0UL));
    ((NRF_CLOCK_Type *) 0x40000000UL)->EVENTS_LFCLKSTARTED  = 0;
    ((NRF_CLOCK_Type *) 0x40000000UL)->TASKS_LFCLKSTART     = 1;
    while (((NRF_CLOCK_Type *) 0x40000000UL)->EVENTS_LFCLKSTARTED == 0)
    {
        
    }
    ((NRF_CLOCK_Type *) 0x40000000UL)->EVENTS_LFCLKSTARTED = 0;

    ((NRF_RTC_Type *) 0x40011000UL)->PRESCALER     = (((32768UL)/(8UL)) - 1);                    
    ((NRF_RTC_Type *) 0x40011000UL)->CC[0]         = (1UL) * (8UL);  

    
    ((NRF_RTC_Type *) 0x40011000UL)->EVTENSET      = (0x1UL << (16UL));
    ((NRF_RTC_Type *) 0x40011000UL)->INTENSET      = (0x1UL << (16UL));

    NVIC_SetPriority(RTC1_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_EnableIRQ(RTC1_IRQn);
    
    ((NRF_RTC_Type *) 0x40011000UL)->TASKS_START   = 1;
    rtc_is_available = 1;
}


 


 
void RTC1_IRQHandler(void)
{
    
    
    ((NRF_RTC_Type *) 0x40011000UL)->EVENTS_COMPARE[1] = 0;
    ((NRF_RTC_Type *) 0x40011000UL)->EVENTS_COMPARE[2] = 0;
    ((NRF_RTC_Type *) 0x40011000UL)->EVENTS_COMPARE[3] = 0;
    
    
    if(rtc_is_available)
    {
        if(0)
        {
            
            ((NRF_RTC_Type *) 0x40011000UL)->EVENTS_COMPARE[0] = 0;
            rtc_resume_cnt++;
            xprintf("RTC1 resume %d\n\r", rtc_resume_cnt);
            
            
        } else {
            if ((((NRF_RTC_Type *) 0x40011000UL)->EVENTS_COMPARE[0] != 0) && 
                ((((NRF_RTC_Type *) 0x40011000UL)->INTENSET & (0x1UL << (16UL))) != 0))
            {
                ((NRF_RTC_Type *) 0x40011000UL)->TASKS_CLEAR = 1;
                ((NRF_RTC_Type *) 0x40011000UL)->EVENTS_COMPARE[0] = 0;
                standard_time++;
                if(alarm.alarm_time == standard_time && alarm.alarm_status != 0)
                {


                    if(alarm.alarm_status == 3) {
                        alarm.alarm_time += 86400; 
                    } else {
                        alarm.alarm_status = 0;
                    }
                    xprintf("Alarm triggered....!!\n\r");
                }
                xprintf("RTC1_IRQHandler trigered, standard_time is: %d\n\r", standard_time);
                
            } else if(((NRF_RTC_Type *) 0x40011000UL)->EVENTS_OVRFLW) {
                
                ((NRF_RTC_Type *) 0x40011000UL)->EVENTS_OVRFLW     = 0;
                standard_time++;
            }
        }
    } else {
        ((NRF_RTC_Type *) 0x40011000UL)->EVENTS_OVRFLW     = 0;
        ((NRF_RTC_Type *) 0x40011000UL)->EVENTS_COMPARE[0] = 0;
        ((NRF_RTC_Type *) 0x40011000UL)->EVENTS_TICK       = 0;

    }
}

uint32_t get_time(void)
{
    return standard_time;
}

void set_time(uint32_t time)
{
    standard_time = time;
    is_time_calibrated = 1;
}

void set_alarm(uint32_t time, uint8_t type)
{
    alarm.alarm_time = time;
    alarm.alarm_status = type | 1;
}

static void rtc1_start(void)
{
    ((NRF_RTC_Type *) 0x40011000UL)->TASKS_STOP    = 1;
    ((NRF_RTC_Type *) 0x40011000UL)->TASKS_CLEAR   = 1;
    ((NRF_RTC_Type *) 0x40011000UL)->PRESCALER     = (((32768UL)/(8UL)) - 1);                       
    ((NRF_RTC_Type *) 0x40011000UL)->CC[0]         = (1UL) * (8UL);  

    
    ((NRF_RTC_Type *) 0x40011000UL)->EVTENSET      = (0x1UL << (16UL));
    ((NRF_RTC_Type *) 0x40011000UL)->INTENSET      = (0x1UL << (16UL));
    
    NVIC_SetPriority(RTC1_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_EnableIRQ(RTC1_IRQn);
    
    ((NRF_RTC_Type *) 0x40011000UL)->TASKS_START   = 1;
}

void rtc1_suspend(void)
{
    ((NRF_RTC_Type *) 0x40011000UL)->TASKS_CLEAR   = 1; 
    ((NRF_RTC_Type *) 0x40011000UL)->CC[0]         = (3600 * (8UL));
    nrf_delay_us(100);
}

void rtc1_resume(void)
{
    uint32_t tmp;
    tmp = ((NRF_RTC_Type *) 0x40011000UL)->COUNTER;
    standard_time += (tmp >> 3); 
    rtc1_start();
    rtc_is_available = 1;
}

void switch_real_time_counter(void)
{
    if(rtc_is_available) {
        rtc_is_available = 0;
    } else {
        rtc_is_available = 1;
        rtc1_start();
    }
}
