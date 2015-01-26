/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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

/** @file
 *
 * @defgroup ble_sdk_app_rsc_main main.c
 * @{
 * @ingroup ble_sdk_app_rsc
 * @brief Running Speed and Cadence Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Running Speed and Cadence service
 * It also includes the sample code for Battery and Device Information services.
 * This application uses the @ref srvlib_conn_params module.
 */
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
#include "ble_bas.h"
#include "ble_hts.h"
#include "ble_rscs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "ble_sensorsim.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "ble_debug_assert_handler.h"
#include "pstorage.h"
#include "app_trace.h"

#include "mma8452q.h"
#include "bmp180.h"
#include "ak8963.h"
#include "bmi055.h"
#include "step_count.h"
#include "oled.h"
#include "simple_uart.h"
#include "twi_master.h"
#include "xprintf.h"
#include "interrupt_hander.h"
#include "nrf_delay.h"
#include "pwr_ctrl.h"
#include "main.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT  0                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define WAKEUP_BUTTON_PIN                BUTTON_0                                   /**< Button used to wake up the application. */
#define BOND_DELETE_ALL_BUTTON_ID        BUTTON_1                                   /**< Button used for deleting all bonded centrals during startup. */

#define ADVERTISING_LED_PIN_NO           LED_0                                      /**< Is on when device is advertising. */
#define CONNECTED_LED_PIN_NO             LED_1                                      /**< Is on when device has connected. */
#define ASSERT_LED_PIN_NO                LED_7                                      /**< Is on when application has asserted. */

#define DEVICE_NAME                      "AtomWear"                               /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "AtomWear"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                 40                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS       0                                        /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS             3                                          /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                          /**< Size of timer operation queues. */

#define BATTERY_LEVEL_MEAS_INTERVAL      APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                81                                         /**< Minimum battery level as returned by the simulated measurement function. */
#define MAX_BATTERY_LEVEL                100                                        /**< Maximum battery level as returned by the simulated measurement function. */
#define BATTERY_LEVEL_INCREMENT          1                                          /**< Value by which the battery level is incremented/decremented for each call to the simulated measurement function. */

#define SPEED_AND_CADENCE_MEAS_INTERVAL  10	                                       /**< Speed and cadence measurement interval (milliseconds). */
#define TEMP_TYPE_AS_CHARACTERISTIC      0                                          /**< Determines if temperature type is given as characteristic (1) or as a field of measurement (0). */

#define MIN_SPEED_MPS                    0.5                                        /**< Minimum speed in meters per second for use in the simulated measurement function. */
#define MAX_SPEED_MPS                    6.5                                        /**< Maximum speed in meters per second for use in the simulated measurement function. */
#define SPEED_MPS_INCREMENT              1.5                                        /**< Value by which speed is incremented/decremented for each call to the simulated measurement function. */
#define MIN_RUNNING_SPEED                1.5                                          /**< speed threshold to set the running bit. */

#define MIN_CADENCE_RPM                  40                                         /**< Minimum cadence in revolutions per minute for use in the simulated measurement function. */
#define MAX_CADENCE_RPM                  160                                        /**< Maximum cadence in revolutions per minute for use in the simulated measurement function. */
#define CADENCE_RPM_INCREMENT            20                                         /**< Value by which cadence is incremented/decremented in the simulated measurement function. */

#define MIN_STRIDE_LENGTH                20                                         /**< Minimum stride length in decimeter for use in the simulated measurement function. */
#define MAX_STRIDE_LENGTH                125                                        /**< Maximum stride length in decimeter for use in the simulated measurement function. */
#define STRIDE_LENGTH_INCREMENT          5                                          /**< Value by which stride length is incremented/decremented in the simulated measurement function. */

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(500, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(1000, UNIT_1_25_MS)          /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                    0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_TIMEOUT                30                                         /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                        0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define NRF_UICR_BOOT_START_ADDRESS     (NRF_UICR_BASE + 0x14)                                  /**< Register where the bootloader start address is stored in the UICR register. */
#define BOOTLOADER_ADDRESS_DEFAULT		0x3C000

typedef enum
{
    BLE_NO_ADV,                                                                     /**< No advertising running. */
    BLE_DIRECTED_ADV,                                                               /**< Direct advertising to the latest central. */
    BLE_FAST_ADV_WHITELIST,                                                         /**< Advertising with whitelist. */
    BLE_FAST_ADV,                                                                   /**< Fast advertising running. */
    BLE_SLOW_ADV,                                                                   /**< Slow advertising running. */
    BLE_SLEEP,                                                                      /**< Go to system-off. */
} ble_advertising_mode_t;

static uint16_t                          m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
static ble_gap_adv_params_t              m_adv_params;                              /**< Parameters to be passed to the stack when starting advertising. */
static ble_bas_t                         m_bas;                                     /**< Structure used to identify the battery service. */
static ble_rscs_t                        m_rscs;                                    /**< Structure used to identify the running speed and cadence service. */
static ble_hts_t                         m_hts;                                     /**< Structure used to identify the health thermometer service. */

static ble_sensorsim_cfg_t               m_battery_sim_cfg;                         /**< Battery Level sensor simulator configuration. */
static ble_sensorsim_state_t             m_battery_sim_state;                       /**< Battery Level sensor simulator state. */

static ble_sensorsim_cfg_t               m_speed_mps_sim_cfg;                       /**< Speed simulator configuration. */
static ble_sensorsim_state_t             m_speed_mps_sim_state;                     /**< Speed simulator state. */
static ble_sensorsim_cfg_t               m_cadence_rpm_sim_cfg;                     /**< Cadence simulator configuration. */
static ble_sensorsim_state_t             m_cadence_rpm_sim_state;                   /**< Cadence simulator state. */
static ble_sensorsim_cfg_t               m_cadence_stl_sim_cfg;                     /**< stride length simulator configuration. */
static ble_sensorsim_state_t             m_cadence_stl_sim_state;                   /**< stride length simulator state. */

static app_timer_id_t                    m_battery_timer_id;                        /**< Battery timer. */
static app_timer_id_t                    m_rsc_meas_timer_id;                       /**< RSC measurement timer. */
static app_timer_id_t					 m_pedo_timer_id;
static dm_application_instance_t         m_app_handle;                              /**< Application identifier allocated by device manager. */

static bool                              m_memory_access_in_progress = false;       /**< Flag to keep track of ongoing operations on persistent memory. */
static bool                              m_hts_meas_ind_conf_pending = false;       /**< Flag to keep track of when an indication confirmation is pending. */
//static bool								 rsc_first_conn_flag = false;
extern uint32_t		step_count;
static uint32_t		second_count = 0;
static calendar 	standard_time;
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
//     while(1)
// 	{
// 		nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
// 		nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
// 		nrf_delay_ms(1000);
// 		nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
// 		nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
// 		nrf_delay_ms(1000);
// 	}

    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Uncomment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
//     NVIC_SystemReset();
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

uint8_t Is_Leap_Year(uint16_t year)
{			  
	if(year%4==0) 
	{ 
		if(year%100==0) 
		{ 
			if(year%400==0)return 1;  
			else return 0;   
		}else return 1;   
	}else return 0;	
}

//set the clock
//1970~2099

uint8_t const table_week[12]={0,3,3,6,1,4,6,2,5,0,3,5}; //offset	  
const uint8_t mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};
void set_standard_time(uint16_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec)
{
	uint16_t	t; 
	uint32_t	seccount = 0;
	
	if(syear<1970 || syear>2099)	return ;
	
	for(t=1970;t<syear;t++)	//add up the seccount of all of the years
	{
		if(Is_Leap_Year(t))seccount+=31622400;//seccount of leap year
		else seccount+=31536000;			  //seccount of not the leap year
	}
	smon-=1;
	for(t=0;t<smon;t++)
	{
		seccount+=(uint32_t)mon_table[t]*86400;
		if(Is_Leap_Year(syear)&&t==1)seccount+=86400;   
	}
	seccount+=(uint32_t)(sday-1)*86400; 
	seccount+=(uint32_t)hour*3600;
    seccount+=(uint32_t)min*60;
	seccount+=sec;
	
	second_count = seccount;
}

void set_sec_count(uint32_t sec_count)
{
	second_count = sec_count;
}

uint32_t get_sec_count()
{
	return second_count;
}

calendar get_standard_time()
{
	static uint16_t daycnt=0;
	uint32_t timecount=0; 
	uint32_t temp=0;
	uint16_t temp1=0;	  
    timecount=second_count;	 
 	temp=timecount/86400;   //get the count of day
	if(daycnt!=temp)//beyond one day
	{	  
		daycnt=temp;
		temp1=1970;	//begin from 1970
		while(temp>=365)
		{				 
			if(Is_Leap_Year(temp1))
			{
				if(temp>=366)temp-=366;
				else {temp1++;break;}  
			}
			else temp-=365;	
			temp1++;  
		}   
		standard_time.year=temp1;//get the year of standard
		temp1=0;
		while(temp>=28)//beyond one month
		{
			if(Is_Leap_Year(standard_time.year)&&temp1==1)
			{
				if(temp>=29)temp-=29;
				else break; 
			}
			else 
			{
				if(temp>=mon_table[temp1])temp-=mon_table[temp1];
				else break;
			}
			temp1++;  
		}
		standard_time.month=temp1+1;
		standard_time.day=temp+1;
	}
	temp=timecount%86400;   	   
	standard_time.hour=temp/3600; 
	standard_time.minute=(temp%3600)/60;
	standard_time.second=(temp%3600)%60;
	
	return standard_time;
}


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update(void)
{
    uint32_t err_code;
    uint8_t  battery_level;
	
	battery_detect_start();
	while(!battery_is_available());
	
	battery_level = get_battery_value();
	battery_detect_stop();
	
// 	xprintf("...battery_level = %d% \r\n", battery_level);
	
// 	if(battery_level < 20)
// 		system_low_battery(); 
	
    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        APP_ERROR_HANDLER(err_code);
    }
}




/**@brief Function for populating simulated health thermometer measurement.
 */
static void hts_sim_measurement(ble_hts_meas_t * p_meas)
{
    static ble_date_time_t time_stamp = { 2012, 12, 5, 11, 50, 0 };
	bmp180_env_value *env_value;

    uint32_t celciusX100;

    p_meas->temp_in_fahr_units = false;
    p_meas->time_stamp_present = false;
    p_meas->temp_type_present  = TEMP_TYPE_AS_CHARACTERISTIC ? false : true;

	env_value = get_environment_value();
// 	xprintf("temperature:%d pressure:%d altitude:%d\r\n", env_value->temperature, env_value->press, (uint32_t)env_value->altitude);
    oled_update_temp();
	celciusX100 = (uint32_t)(env_value->temperature)*10;

    p_meas->temp_in_celcius.exponent = -2;
    p_meas->temp_in_celcius.mantissa = celciusX100;
    p_meas->temp_in_fahr.exponent    = -2;
    p_meas->temp_in_fahr.mantissa    = (32 * 100) + ((celciusX100 * 9) / 5);
    p_meas->time_stamp               = time_stamp;
    p_meas->temp_type                = BLE_HTS_TEMP_TYPE_FINGER;

    // update simulated time stamp
    time_stamp.seconds += 27;
    if (time_stamp.seconds > 59)
    {
        time_stamp.seconds -= 60;
        time_stamp.minutes++;
        if (time_stamp.minutes > 59)
        {
            time_stamp.minutes = 0;
        }
    }
}

/**@brief Function for simulating and sending one Temperature Measurement.
 */
static void temperature_measurement_send(void)
{
    ble_hts_meas_t simulated_meas;
    uint32_t       err_code;

	if (!m_hts_meas_ind_conf_pending)
    {
		hts_sim_measurement(&simulated_meas);
        err_code = ble_hts_measurement_send(&m_hts, &simulated_meas);
        switch (err_code)
        {
            case NRF_SUCCESS:
                // Measurement was successfully sent, wait for confirmation.
                m_hts_meas_ind_conf_pending = true;
                break;

            case NRF_ERROR_INVALID_STATE:
                // Ignore error.
                break;

            default:
                APP_ERROR_HANDLER(err_code);
                break;
        }
    }
}

/**@brief Function for populating simulated running speed and cadence measurement.
 */
static void rsc_sim_measurement(ble_rscs_meas_t * p_measurement)
{
	static uint32_t last_step_count = 0, last_step = 0;
	static uint8_t	sec = 0;
	static uint32_t	sigma_step = 0;
	uint32_t step;
	int32_t delta_step, cadence;
	
	#define P	45
	#define I	0
	#define D	10
	
	if(++sec >= 60)
	{
		sec = 0;
		sigma_step = 0;
	}
	
	
	step = get_the_step_count() - last_step_count;
	last_step_count = get_the_step_count();
	
	delta_step = step - last_step;
	last_step = step;
	
	sigma_step += step;
	
	cadence = (uint32_t)(P*step + I*sigma_step + D*delta_step);
	if(cadence < 0) cadence = 0;
	
	oled_update_step_count();
	
    p_measurement->is_inst_stride_len_present = true;
    p_measurement->is_total_distance_present  = true;
    p_measurement->is_running                 = false;

    p_measurement->inst_cadence       = cadence;
	p_measurement->inst_speed = (uint16_t)(cadence * 3.2);	//uint:m/s (3.2 = 0.75/60*256)

    p_measurement->inst_stride_length = 75;
	
	p_measurement->total_distance = get_total_distance();

    if (p_measurement->inst_speed > (uint32_t)(MIN_RUNNING_SPEED * 256))
    {
        p_measurement->is_running = true;
    }
	//first connect to send the steps to mobile app
// 	if(rsc_first_conn_flag)
// 	{
// 		p_measurement->inst_stride_length = get_the_step_count();
// 		rsc_first_conn_flag = false;
// 	}
}

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{	
	uint32_t        err_code;
    ble_rscs_meas_t rscs_measurement;
	ak8963_cmps 	*cmps;
	uint8_t			sys_work_mode = get_work_mode();
	
    UNUSED_PARAMETER(p_context);
	
	++second_count;	

	battery_level_update();
	
	switch(sys_work_mode)
	{
	case SYSTEM_DEFAULT_MODE :
		oled_default_mode();
		break;
	case SYSTEM_PEDOMETER_MODE :
		rsc_sim_measurement(&rscs_measurement);

		err_code = ble_rscs_measurement_send(&m_rscs, &rscs_measurement);
		break;
	
	case SYSTEM_TEMP_MODE :
		temperature_measurement_send();
		break;
	case SYSTEM_COMPASS_MODE :
		cmps = ak8963_get_value();
		oled_update_cmps(cmps);
		break;
	}
	if (
		(err_code != NRF_SUCCESS)
		&&
		(err_code != NRF_ERROR_INVALID_STATE)
		&&
		(err_code != BLE_ERROR_NO_TX_BUFFERS)
		&&
		(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
	)
	{
// 		APP_ERROR_HANDLER(err_code);
	}
}

/**@brief Function for handling the Running Speed and Cadence measurement timer timeout.
 *
 * @details This function will be called each time the running speed and cadence
 *          measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void rsc_meas_timeout_handler(void * p_context)
{            
    UNUSED_PARAMETER(p_context);
	
	if(SYSTEM_PEDOMETER_MODE == get_work_mode())	pedometer_startup();
}

/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
static void leds_init(void)
{
    nrf_gpio_cfg_output(ADVERTISING_LED_PIN_NO);
    nrf_gpio_cfg_output(CONNECTED_LED_PIN_NO);
    nrf_gpio_cfg_output(ASSERT_LED_PIN_NO);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create battery timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    // Create rsc_meas timer.
    err_code = app_timer_create(&m_rsc_meas_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                rsc_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_RUNNING_WALKING_SENSOR);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    ble_uuid_t adv_uuids[] =
    {
		{BLE_UUID_HEALTH_THERMOMETER_SERVICE, BLE_UUID_TYPE_BLE},
        {BLE_UUID_RUNNING_SPEED_AND_CADENCE,  BLE_UUID_TYPE_BLE},
        {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
    };

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;                           // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;
}

/**@brief Function for handling the Health Thermometer Service events.
 *
 * @details This function will be called for all Health Thermometer Service events which are passed
 *          to the application.
 *
 * @param[in]   p_hts   Health Thermometer Service structure.
 * @param[in]   p_evt   Event received from the Health Thermometer Service.
 */
static void on_hts_evt(ble_hts_t * p_hts, ble_hts_evt_t *p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_HTS_EVT_INDICATION_ENABLED:
            // Indication has been enabled, send a single temperature measurement
            temperature_measurement_send();
            break;

        case BLE_HTS_EVT_INDICATION_CONFIRMED:
            m_hts_meas_ind_conf_pending = false;
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Running Speed and Cadence, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t        err_code;
	ble_hts_init_t   hts_init;
    ble_rscs_init_t rscs_init;
    ble_bas_init_t  bas_init;
    ble_dis_init_t  dis_init;
	
	// Initialize Health Thermometer Service
    memset(&hts_init, 0, sizeof(hts_init));

    hts_init.evt_handler                 = on_hts_evt;
    hts_init.temp_type_as_characteristic = 0;
    hts_init.temp_type                   = BLE_HTS_TEMP_TYPE_BODY;

    // Here the sec level for the Health Thermometer Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hts_init.hts_meas_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hts_init.hts_meas_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hts_init.hts_meas_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hts_init.hts_temp_type_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hts_init.hts_temp_type_attr_md.write_perm);

    err_code = ble_hts_init(&m_hts, &hts_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Running Speed and Cadence Service

    memset(&rscs_init, 0, sizeof(rscs_init));

    rscs_init.evt_handler = NULL;
    rscs_init.feature     = BLE_RSCS_FEATURE_INSTANT_STRIDE_LEN_BIT|
								BLE_RSCS_FEATURE_TOTAL_DISTANCE_BIT|			//
									BLE_RSCS_FEATURE_WALKING_OR_RUNNING_STATUS_BIT;

    // Here the sec level for the Running Speed and Cadence Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&rscs_init.rsc_meas_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&rscs_init.rsc_meas_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&rscs_init.rsc_meas_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&rscs_init.rsc_feature_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&rscs_init.rsc_feature_attr_md.write_perm);

    err_code = ble_rscs_init(&m_rscs, &rscs_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the sensor simulators.
 */
static void sensor_sim_init(void)
{
    m_battery_sim_cfg.min              = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max              = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr             = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max     = true;

    ble_sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);

    // speed is in units of meters per second divided by 256
    m_speed_mps_sim_cfg.min            = (uint32_t)(MIN_SPEED_MPS * 256);
    m_speed_mps_sim_cfg.max            = (uint32_t)(MAX_SPEED_MPS * 256);
    m_speed_mps_sim_cfg.incr           = (uint32_t)(SPEED_MPS_INCREMENT * 256);
    m_speed_mps_sim_cfg.start_at_max   = false;

    ble_sensorsim_init(&m_speed_mps_sim_state, &m_speed_mps_sim_cfg);

    m_cadence_rpm_sim_cfg.min          = MIN_CADENCE_RPM;
    m_cadence_rpm_sim_cfg.max          = MAX_CADENCE_RPM;
    m_cadence_rpm_sim_cfg.incr         = CADENCE_RPM_INCREMENT;
    m_cadence_rpm_sim_cfg.start_at_max = false;

    ble_sensorsim_init(&m_cadence_rpm_sim_state, &m_cadence_rpm_sim_cfg);

    m_cadence_stl_sim_cfg.min          = MIN_STRIDE_LENGTH;
    m_cadence_stl_sim_cfg.max          = MAX_STRIDE_LENGTH;
    m_cadence_stl_sim_cfg.incr         = STRIDE_LENGTH_INCREMENT;
    m_cadence_stl_sim_cfg.start_at_max = false;

    ble_sensorsim_init(&m_cadence_stl_sim_state, &m_cadence_stl_sim_cfg);
}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;
    uint32_t rsc_meas_timer_ticks;
	uint32_t pedo_timer_ticks;

    // Start application timers.
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    rsc_meas_timer_ticks = APP_TIMER_TICKS(SPEED_AND_CADENCE_MEAS_INTERVAL, APP_TIMER_PRESCALER);

    err_code = app_timer_start(m_rsc_meas_timer_id, rsc_meas_timer_ticks, NULL);
    APP_ERROR_CHECK(err_code);

	pedo_timer_ticks = APP_TIMER_TICKS(4000, APP_TIMER_PRESCALER);

    err_code = app_timer_start(m_pedo_timer_id, pedo_timer_ticks, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;
    uint32_t count;

    // Verify if there is any flash access pending, if yes delay starting advertising until 
    // it's complete.
    err_code = pstorage_access_status_get(&count);
    APP_ERROR_CHECK(err_code);
    
    if (count != 0)
    {
        m_memory_access_in_progress = true;
        return;
    }
    
    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);

    nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = m_rscs.meas_handles.cccd_handle;//BLE_GATT_HANDLE_INVALID;//
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
            nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			
// 			rsc_first_conn_flag = true;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
			
			m_hts_meas_ind_conf_pending = false;
// 			rsc_first_conn_flag = true;

            advertising_start();
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            {
                nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
				xprintf("\r\nBle gap event timeout...\r\n");

                // Go to system-off mode (this function will not return; wakeup will cause a reset).
                err_code = sd_power_system_off();
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.
 */
static void on_sys_evt(uint32_t sys_evt)
{
    switch(sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        case NRF_EVT_FLASH_OPERATION_ERROR:
            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                advertising_start();
            }
            break;
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
	ble_hts_on_ble_evt(&m_hts, p_ble_evt);
    ble_rscs_on_ble_evt(&m_rscs, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing buttons.
 */
static void buttons_init(void)
{
    // Set Wakeup and Bonds Delete buttons as wakeup sources
    nrf_gpio_cfg_sense_input(WAKEUP_BUTTON_PIN,
                             BUTTON_PULL, 
                             NRF_GPIO_PIN_SENSE_LOW);
    
    nrf_gpio_cfg_sense_input(BOND_DELETE_ALL_BUTTON_ID,
                             BUTTON_PULL, 
                             NRF_GPIO_PIN_SENSE_LOW);
}


/**@brief Function for handling the Device Manager events.
 *
 * @param[in]   p_evt   Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const    * p_handle,
                                           dm_event_t const     * p_event,
                                           api_result_t           event_result)
{
    APP_ERROR_CHECK(event_result);
    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 */
static void device_manager_init(void)
{
    uint32_t                err_code;
    dm_init_param_t         init_data;
    dm_application_param_t  register_param;
    
    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    // Clear all bonded centrals if the Bonds Delete button is pushed.
    init_data.clear_persistent_data = (nrf_gpio_pin_read(BOND_DELETE_ALL_BUTTON_ID) == 0);

    err_code = dm_init(&init_data);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));
    
    register_param.sec_param.timeout      = SEC_PARAM_TIMEOUT;
    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/*---------------------------  -------------------------*/
void system_HFCLK_init()
{
    // Start 16 MHz crystal oscillator.
    NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;    // HFCLK oscillator state = 0.
    NRF_CLOCK->TASKS_HFCLKSTART     = 1;    // Start HFCLK clock source

    // Wait for the external oscillator to start up.
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
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

void enter_default_mode()
{
	xprintf("\r\nEnter the default mode...\r\n");
	oled_default_mode();
}

void enter_pedo_mode()
{
	xprintf("\r\nEnter the pedometer mode...\r\n");
	oled_pedo_mode();
	
    MMA845x_Init();				//Initial the 3-Axis Accelerometer
	MMA845x_Active();	
    pedometer_init();
}

void enter_temp_mode()
{
	xprintf("\r\nEnter the temp mode...\r\n");
	oled_temp_mode();
	
	bmp180_init_interface();
}

void enter_cmps_mode()
{
	xprintf("\r\nEnter the compass mode...\r\n");
	oled_cmps_mode();
	
	ak8963_init();
	ak8963_cs1_mode();
}

/** @brief Function for filling a page in flash with a value.
 *
 * @param[in] address Address of the first word in the page to be filled.
 * @param[in] value Value to be written to flash.
 */
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

/*--------------------------- read back disable ------------------------*/
void read_back_disable(void)
{
     NRF_UICR->RBPCONF = UICR_RBPCONF_PALL_Disabled << UICR_RBPCONF_PALL_Pos;
}
/*--------------------------- check the bootloader of mcu ---------------*/
void mcu_bootloader_check(void)
{    
    if(*((uint32_t *)NRF_UICR_BOOT_START_ADDRESS) != BOOTLOADER_ADDRESS_DEFAULT)
    {                                                
        read_back_disable();
        flash_word_write((uint32_t*)NRF_UICR_BOOT_START_ADDRESS, BOOTLOADER_ADDRESS_DEFAULT);
        xprintf("switch bootloader to 0xx%, begin to reset.\r\n", BOOTLOADER_ADDRESS_DEFAULT);
        NVIC_SystemReset();
    }
    
    xprintf("system app start work...\r\n");
}

void sensor_init()
{
	nrf_gpio_cfg_input(4, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(5, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(6, NRF_GPIO_PIN_PULLDOWN);
	
	MMA845x_Init();
	bmp180_init_interface();
	bmi055_init();
	ak8963_init();
}

/**@brief Function for application main entry.
 */
int main(void)
{
	system_HFCLK_init();
	// uart init
    simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, 0); 
    xprintf("APP start up..\r\n");
	
	mcu_bootloader_check();
    // Initialize.
    app_trace_init();
    leds_init();
    buttons_init();
    ble_stack_init();
    device_manager_init();
    timers_init();
    gap_params_init();
    advertising_init();
    services_init();
    sensor_sim_init();
    conn_params_init();
	
	//I2C and sensor initial
	system_I2C_init();
	OLED_Init();
	sensor_init();
	set_standard_time(2015, 1, 22, 14, 23, 0);
	enter_default_mode();	
	
	GPIO_HiToLo_INT_config(BUTTON_2);
	
	// Test battery
	if(!battery_self_detect())
    {
        system_low_battery();
    }
	
	// Start execution.
    application_timers_start();
    advertising_start();
	
	// Enter main loop.
    for (;;)
    {		
        power_manage();
    }
}

/**
 * @}
 */
