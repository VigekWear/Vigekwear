/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/**@file
 *
 * @defgroup ble_sdk_app_bootloader_main main.c
 * @{
 * @ingroup dfu_bootloader_api
 * @brief Bootloader project main file.
 *
 * -# Receive start data package. 
 * -# Based on start packet, prepare NVM area to store received data. 
 * -# Receive data packet. 
 * -# Validate data packet.
 * -# Write Data packet to NVM.
 * -# If not finished - Wait for next packet.
 * -# Receive stop data packet.
 * -# Activate Image, boot application.
 *
 */
#include "dfu.h"
#include "dfu_transport.h"
#include "bootloader.h"
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include "nordic_common.h"
#include "nrf.h"
#ifndef S310_STACK
#include "nrf_mbr.h"
#endif // S310_STACK
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "nrf51.h"
#include "ble_hci.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "app_gpiote.h"
#include "nrf_error.h"
#include "boards.h"
#include "ble_debug_assert_handler.h"
#include "softdevice_handler.h"
#include "pstorage_platform.h"
#include "nrf_delay.h"
#include "xprintf.h"
#include "simple_uart.h"
#include "oled_dfu.h"
#include "twi_master.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                                       /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define BOOTLOADER_BUTTON_PIN           BUTTON_2                                                /**< Button used to enter SW update mode. */

#define APP_GPIOTE_MAX_USERS            1                                                       /**< Number of GPIOTE users in total. Used by button module and dfu_transport_serial module (flow control). */

#define APP_TIMER_PRESCALER             0                                                       /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            3                                                       /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                                       /**< Size of timer operation queues. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)                /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVT_SIZE, 0)                        /**< Maximum size of scheduler events. */

#define SCHED_QUEUE_SIZE                20                                                      /**< Maximum number of events in the scheduler queue. */


/**@brief Function for error handling, which is called when an error has occurred. 
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{    
    nrf_gpio_pin_set(LED_7);
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover on reset.
    NVIC_SystemReset();
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] file_name   File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for initialization of LEDs.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    nrf_gpio_cfg_output(LED_0);
    nrf_gpio_cfg_output(LED_1);
    nrf_gpio_cfg_output(LED_2);
    nrf_gpio_cfg_output(LED_7);
}


/**@brief Function for clearing the LEDs.
 *
 * @details Clears all LEDs used by the application.
 */
static void leds_off(void)
{
    nrf_gpio_pin_clear(LED_0);
    nrf_gpio_pin_clear(LED_1);
    nrf_gpio_pin_clear(LED_2);
    nrf_gpio_pin_clear(LED_7);
}


/**@brief Function for initializing the GPIOTE handler module.
 */
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);
}


/**@brief Function for initializing the button module.
 */
static void buttons_init(void)
{   
    nrf_gpio_cfg_sense_input(BOOTLOADER_BUTTON_PIN,
                             BUTTON_PULL, 
                             NRF_GPIO_PIN_SENSE_LOW);
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void sys_evt_dispatch(uint32_t event)
{
    pstorage_sys_event_handler(event);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

#ifndef S310_STACK
    sd_mbr_command_t com = {SD_MBR_COMMAND_INIT_SD, };

    err_code = sd_mbr_command(&com);
    APP_ERROR_CHECK(err_code);

    err_code = sd_softdevice_vector_table_base_set(BOOTLOADER_REGION_START);
    APP_ERROR_CHECK(err_code);
#endif // S310_STACK

    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, true);

#ifndef S310_STACK
    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
#endif // S310_STACK

    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for event scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

#define system_I2C_init()                   \
        do                                  \
        {                                   \
            if(!twi_master_init())          \
                {	                        \
                    while(1);               \
                }                           \
        }                                   \
        while(0)

void flash_word_write(uint32_t *address, uint32_t value)
{
    // Turn on flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);
    
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
  
    *address = value;
  
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
  
    // Turn off flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
  
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
}

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool     bootloader_is_pushed = false;
	
	NRF_UICR->RBPCONF = UICR_RBPCONF_PALL_Disabled << UICR_RBPCONF_PALL_Pos;
	flash_word_write((uint32_t*)NRF_UICR_BOOT_START_ADDRESS, BOOTLOADER_REGION_START);
	
	simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, 0); 
    xprintf("DFU start up..\r\n");
    
    leds_init();

    APP_ERROR_CHECK_BOOL(*((uint32_t *)NRF_UICR_BOOT_START_ADDRESS) == BOOTLOADER_REGION_START);
    APP_ERROR_CHECK_BOOL(NRF_FICR->CODEPAGESIZE == CODE_PAGE_SIZE);

    // Initialize.
    timers_init();
    gpiote_init();
    buttons_init();
    ble_stack_init();
    scheduler_init();
	
	//I2C and oled initial
// 	system_I2C_init();
// 	OLED_Init();
	
	nrf_gpio_pin_clear(LED_0);
	nrf_gpio_pin_set(LED_1);
	nrf_delay_ms(3000);

    bootloader_is_pushed = ((nrf_gpio_pin_read(BOOTLOADER_BUTTON_PIN) == 0)? true: false);
    
    if (bootloader_is_pushed)// || (!bootloader_app_is_valid(DFU_BANK_0_REGION_START)))
    {
		uint8_t i = 5;
		while(--i)
		{
			nrf_gpio_pin_clear(LED_0);
			nrf_gpio_pin_clear(LED_1);
			nrf_delay_ms(300);
			nrf_gpio_pin_set(LED_0);
			nrf_gpio_pin_set(LED_1);
			nrf_delay_ms(300);
		}
        nrf_gpio_pin_clear(LED_1);
		xprintf("Enter the DFU mode...\r\n");
// 		oled_DFU_mode();
        // Initiate an update of the firmware.
        err_code = bootloader_dfu_start();
        APP_ERROR_CHECK(err_code);

//         nrf_gpio_pin_set(LED_1);
    }

    //if (bootloader_app_is_valid(DFU_BANK_0_REGION_START)) 
    {
        leds_off();
        xprintf("Enter the APP mode...\r\n");
        // Select a bank region to use as application region.
        // @note: Only applications running from DFU_BANK_0_REGION_START is supported.
        bootloader_app_start(DFU_BANK_0_REGION_START);        
    }

    nrf_gpio_pin_clear(LED_0);
    nrf_gpio_pin_clear(LED_1);
        \
    NVIC_SystemReset();
}
