#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "app_scheduler.h"
#include "ble_stack_handler.h"
#include "app_timer.h"
#include "ble_error_log.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "ble_debug_assert_handler.h"
#include "ble_nus.h"
#include "boards.h"
#include <stdbool.h>
#include "nrf_delay.h"

#include "simple_uart.h"
#include "nrf_gpio.h"



#define WAKEUP_BUTTON_PIN               BUTTON0_PIN_NUMBER                          /**< Button used to wake up the application. */

#define DEVICE_NAME                     "BLE_UART_LL"                                  /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      60 /*180*/                                         /**< The advertising timeout (in units of seconds). */

// YOUR_JOB: Modify these according to requirements.
#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            2                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define SECOND_1_25_MS_UNITS            800                                         /**< Definition of 1 second, when 1 unit is 1.25 ms. */
#define SECOND_10_MS_UNITS              100                                         /**< Definition of 1 second, when 1 unit is 10 ms. */
#define MIN_CONN_INTERVAL               (SECOND_1_25_MS_UNITS  / 20)                  /**< Minimum acceptable connection interval (0.25 seconds), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               (SECOND_1_25_MS_UNITS * 2)                  /**< Maximum acceptable connection interval (2 second), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                (4 * SECOND_10_MS_UNITS)                    /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (1500 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(500, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first (500 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of users of the GPIOTE handler. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_TIMEOUT               30                                          /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */



#define BLE_REC_SIZE                    1034
#define BLE_SEND_SIZE                   20

#define SYSTEM_UPDATE_PREPARE           0x01
#define SYSTEM_UPDATE_ING               0x02
#define SYSTEM_UPDATE_END               0x03
#define SYSTEM_UPDATE_FAILURE           0x04

#define DFU_SUCC                        0x01
#define DFU_END                         0x02
#define DFU_ERROR                       0x03
#define DFU_PREPARE                     0x04

#define BLE_RESPONSE                    0x08
#define BLE_DFU_AMOUNT                  0xE
#define BLE_DFU_SUCC                    0xF
#define BLE_DFU_ERROR                   0x10

uint32_t get_dfu_amount(void);

uint8_t get_dfu_state(void);

//uint32_t get_ble_rec_count(void);

//void set_ble_rec_state(uint8_t rec_state);


//void ble_send_state_set( uint8_t state);

// void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t *p_file_name);


// void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name);

// static void timers_init(void);


// static void gap_params_init(void);


// static void advertising_init(void);

// bool data_compare(uint8_t *data1, uint16_t length1, uint8_t *data2, uint16_t length2);

// void data_handler(ble_nus_t *p_nus, uint8_t *data, uint16_t length);


// static void services_init(void);



// static void sec_params_init(void);



// static void on_conn_params_evt(ble_conn_params_evt_t *p_evt);


// static void conn_params_error_handler(uint32_t nrf_error);



// static void conn_params_init(void);



// static void advertising_start(void);




// static void on_ble_evt(ble_evt_t *p_ble_evt);



// static void ble_evt_dispatch(ble_evt_t *p_ble_evt);



// static void ble_stack_init(void);


// static void power_manage(void);


// void ble_prepare(void);

// void ble_close(void);

void ble_start(void);

void ble_rec_handle(void);

void set_ble_rec_state(uint8_t rec_state);



/**
 * @}
 */
