/**
 ******************************************************************************
 * @file      startup_cortex_m0.c
 * @author    Coocox
 * @version   V1.0
 * @date      09/03/2011
 * @brief     Cortex M0 Devices Startup code.
 *            This module performs:
 *                - Set the initial SP
 *                - Set the vector table entries 
 *                - Initialize data and bss 			
 *                - Call the application's entry point.
 *            After Reset the Cortex-M0 processor is in Thread mode,
 *            priority is Privileged, and the Stack is set to Main.
 *******************************************************************************
 */
 
 
/*----------Stack Configuration-----------------------------------------------*/  
#define STACK_SIZE       0x00000100      /*!< Stack size (in Words)           */
__attribute__ ((section(".co_stack")))
unsigned long pulStack[STACK_SIZE];      


/*----------Macro definition--------------------------------------------------*/  
#define WEAK __attribute__ ((weak))           


/*----------Declaration of the default fault handlers-------------------------*/  
/* System exception vector handler */
__attribute__ ((used))
void WEAK  Reset_Handler(void);
void WEAK  NMI_Handler(void);
void WEAK  HardFault_Handler(void);
void WEAK  MemManage_Handler(void);
void WEAK  BusFault_Handler(void);
void WEAK  UsageFault_Handler(void);
void WEAK  SVC_Handler(void);
void WEAK  DebugMon_Handler(void);
void WEAK  PendSV_Handler(void);
void WEAK  SysTick_Handler(void);
void WEAK  POWER_CLOCK_IRQHandler(void);
void WEAK  RADIO_IRQHandler(void);
void WEAK  UART0_IRQHandler(void);
void WEAK  SPI0_TWI0_IRQHandler(void);
void WEAK  SPI1_TWI1_IRQHandler(void);
void WEAK  Reverved(void);
void WEAK  GPIOTE_IRQHandler(void);
void WEAK  ADC_IRQHandler(void);
void WEAK  TIMER0_IRQHandler(void);
void WEAK  TIMER1_IRQHandler(void);
void WEAK  TIMER2_IRQHandler(void);
void WEAK  RTC0_IRQHandler(void);
void WEAK  TEMP_IRQHandler(void);
void WEAK  RNG_IRQHandler(void);
void WEAK  ECB_IRQHandler(void);
void WEAK  CCM_AAR_IRQHandler(void);
void WEAK  WDT_IRQHandler(void);
void WEAK  RTC1_IRQHandler(void);
void WEAK  QDEC_IRQHandler(void);
void WEAK  LPCOMP_COMP_IRQHandler(void);
void WEAK  SWI0_IRQHandler(void);
void WEAK  SWI1_IRQHandler(void);
void WEAK  SWI2_IRQHandler(void);
void WEAK  SWI3_IRQHandler(void);
void WEAK  SWI4_IRQHandler(void);
void WEAK  SWI5_IRQHandler(void);

/*----------Symbols defined in linker script----------------------------------*/  
extern unsigned long _sidata;    /*!< Start address for the initialization 
                                      values of the .data section.            */
extern unsigned long _sdata;     /*!< Start address for the .data section     */    
extern unsigned long _edata;     /*!< End address for the .data section       */    
extern unsigned long _sbss;      /*!< Start address for the .bss section      */
extern unsigned long _ebss;      /*!< End address for the .bss section        */      
extern void _eram;               /*!< End address for ram                     */


/*----------Function prototypes-----------------------------------------------*/  
extern void SystemInit(void);
extern int main(void);           /*!< The entry point for the application.    */
void Default_Reset_Handler(void);   /*!< Default reset handler                */
static void Default_Handler(void);  /*!< Default exception handler            */


/**
  *@brief The minimal vector table for a Cortex M0.  Note that the proper constructs
  *       must be placed on this to ensure that it ends up at physical address
  *       0x00000000.  
  */
__attribute__ ((used,section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{
  /*----------Core Exceptions------------------------------------------------ */
  (void *)&pulStack[STACK_SIZE],       /*!< The initial stack pointer         */
  Reset_Handler,                       /*!< The reset handler                 */
  NMI_Handler,                         /*!< The NMI handler                   */ 
  HardFault_Handler,                   /*!< The hard fault handler            */
  0,                   /*!< The MPU fault handler             */
  0,                    /*!< The bus fault handler             */
  0,                  /*!< The usage fault handler           */ 
  0,0,0,0,                             /*!< Reserved                          */
  SVC_Handler,                         /*!< SVCall handler                    */
  0,                    /*!< Debug monitor handler             */
  0,                                   /*!< Reserved                          */
  PendSV_Handler,                      /*!< The PendSV handler                */
  SysTick_Handler,                     /*!< The SysTick handler               */ 
  POWER_CLOCK_IRQHandler,
  RADIO_IRQHandler,
  UART0_IRQHandler,
  SPI0_TWI0_IRQHandler,
  SPI1_TWI1_IRQHandler,
  0,
  GPIOTE_IRQHandler,
  ADC_IRQHandler,
  TIMER0_IRQHandler,
  TIMER1_IRQHandler,
  TIMER2_IRQHandler,
  RTC0_IRQHandler,
  TEMP_IRQHandler,
  RNG_IRQHandler,
  ECB_IRQHandler,
  CCM_AAR_IRQHandler,
  WDT_IRQHandler,
  RTC1_IRQHandler,
  QDEC_IRQHandler,
  LPCOMP_COMP_IRQHandler,
  SWI0_IRQHandler,
  SWI1_IRQHandler,
  SWI2_IRQHandler,
  SWI3_IRQHandler,
  SWI4_IRQHandler,
  SWI5_IRQHandler,
  0,0,0,0,0,0
  /*----------External Exceptions---------------------------------------------*/

};


/**
  * @brief  This is the code that gets called when the processor first
  *         starts execution following a reset event. Only the absolutely
  *         necessary set is performed, after which the application
  *         supplied main() routine is called. 
  * @param  None
  * @retval None
  */
void Default_Reset_Handler(void)
{
  /* Initialize data and bss */
  unsigned long *pulSrc, *pulDest;

  /* Copy the data segment initializers from flash to SRAM */
  pulSrc = &_sidata;

  for(pulDest = &_sdata; pulDest < &_edata; )
  {
    *(pulDest++) = *(pulSrc++);
  }
  
  /* Zero fill the bss segment. */
  for(pulDest = &_sbss; pulDest < &_ebss; )
  {
    *(pulDest++) = 0;
  }

  SystemInit();
  /* Call the application's entry point.*/
  main();
}


/**
  *@brief Provide weak aliases for each Exception handler to the Default_Handler. 
  *       As they are weak aliases, any function with the same name will override 
  *       this definition.
  */
  
#pragma weak Reset_Handler = Default_Reset_Handler
#pragma weak NMI_Handler = Default_Handler
#pragma weak HardFault_Handler = Default_Handler
#pragma weak MemManage_Handler = Default_Handler
#pragma weak BusFault_Handler = Default_Handler
#pragma weak UsageFault_Handler = Default_Handler
#pragma weak SVC_Handler = Default_Handler
#pragma weak DebugMon_Handler = Default_Handler
#pragma weak PendSV_Handler = Default_Handler
#pragma weak SysTick_Handler = Default_Handler
 

/**
  * @brief  This is the code that gets called when the processor receives an 
  *         unexpected interrupt.  This simply enters an infinite loop, 
  *         preserving the system state for examination by a debugger.
  * @param  None
  * @retval None  
  */
static void Default_Handler(void) 
{
  /* Go into an infinite loop. */
  while (1) 
  {
  }
}

/*********************** (C) COPYRIGHT 2009 Coocox ************END OF FILE*****/
