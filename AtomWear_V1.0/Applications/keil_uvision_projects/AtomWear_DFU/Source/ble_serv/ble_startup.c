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

#include "xprintf.h"
#include "ble_startup.h"
#include "flash_ctrl.h"
#include "dfu_ctrl.h"
#include "timer.h"



static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_nus_t                     m_nus;
ble_nus_t                      m_nus;



struct ble_rec_que
{
    uint8_t  data[BLE_REC_SIZE];    
    uint32_t front;                  
    uint32_t rear;                  
    uint32_t count;                 
    uint8_t  state;                  

    uint8_t  cur_rec_inst;           // pre-inst
};

struct ble_rec_que ble_rec;

struct dfu_struct
{
    uint32_t amount;             
    uint8_t  pages;              
    uint32_t count;              
    uint16_t CRC;                
    uint8_t  state;              
};

struct dfu_struct dfu;

void set_ble_rec_state(uint8_t rec_state)
{
    ble_rec.state  = rec_state;
}

void dfu_init()
{
    dfu.amount  =  0;
    dfu.pages   =  0;
    dfu.count   =  0;
    dfu.CRC     =  0;
    dfu.state   =  SYSTEM_UPDATE_PREPARE;
}



uint8_t get_dfu_state()
{
    return dfu.state;
}

uint32_t get_dfu_amount()
{
    return dfu.amount;
}


void set_dfu_state(uint8_t dfu_state)
{
    dfu.state = dfu_state;
}

uint32_t get_ble_rec_count()
{
    return ble_rec.count;
}

void ble_rec_input(uint8_t *ch, uint16_t length)
{
    uint16_t i = 0;
    if(length <= (BLE_REC_SIZE - ble_rec.count))
    {
        for(; i < length; i++)
        {
            ble_rec.data[ble_rec.rear] = ch[i];
            ble_rec.rear = (++ble_rec.rear)%BLE_REC_SIZE;
            ble_rec.count +=1;
        }
    }
}

uint8_t ble_rec_output()
{
    uint8_t fronter = 0;
    if(ble_rec.count != 0)
    {
        fronter = ble_rec.front;
        ble_rec.front  = (++ble_rec.front)%BLE_REC_SIZE;
        ble_rec.count -=1;
        return ble_rec.data[fronter];
    }
    return false;
}


void ble_rec_init()
{
    memset(ble_rec.data, 0, BLE_REC_SIZE);
    ble_rec.front  =  0;    
    ble_rec.rear   =  0;    
    ble_rec.count  =  0;
    ble_rec.state  =  DFU_PREPARE;
    ble_rec.cur_rec_inst = 0;
}

void dfu_refresh()
{
    dfu.pages++;
    dfu.count += ble_rec.count -2;
    dfu.CRC    =  0;
}

void ble_write_page_state(uint8_t state)
{
    uint8_t send_char;
    
    send_char = state;
    ble_nus_send_string(&m_nus, &send_char, 1);
}
  


void system_device_firmware_update()
{
    uint8_t error_count = 0;
   
    ble_rec_init();
    time1_init();

    set_dfu_state(SYSTEM_UPDATE_ING);
    //timer1_start();
    
    xprintf("begin to recieve data.\r\n");

    while(dfu.state == SYSTEM_UPDATE_ING)
    {

        if( (((dfu.pages)*1024 + ble_rec.count - 2) == dfu.amount ) || (get_ble_rec_count() == 1026) )
        {
            //timer1_stop();            
            ble_rec.state = system_page_refresh(dfu.pages, ble_rec.data, ble_rec.count);                               
        }
    

        switch(ble_rec.state)
        {
        case DFU_SUCC :                      
            dfu_refresh();
            ble_rec_init();
            error_count = 0;  
              
              
            xprintf("DFU_SUCC, pages:%d, counts:%d \r\n", dfu.pages, dfu.count);        
            ble_write_page_state(BLE_DFU_SUCC);        
            break;

        case DFU_END :
            dfu_refresh();
            xprintf("DFU_END, pages:%d, counts:%d \r\n", dfu.pages, dfu.count); 
            set_dfu_state(SYSTEM_UPDATE_END);

            ble_write_page_state(BLE_DFU_SUCC);
            nrf_delay_ms(500);
            break;

        case DFU_ERROR :   // data translate error or timeout counts < 5            
            xprintf("DFU_ERROR \r\n");
            error_count++;
            // too many translste errors or timeout conditions during update
            if(error_count == 5)
            {
                xprintf("DFU_ERROR > 5 \r\n");
                set_dfu_state(SYSTEM_UPDATE_FAILURE);              
                
                ble_write_page_state(BLE_DFU_ERROR);
            }
            else
            {
                xprintf("DFU_ERROR  \r\n");
                ble_write_page_state(BLE_DFU_ERROR);
                               
                ble_rec_init();
                set_ble_rec_state(DFU_PREPARE);                
            }
            break;

        case DFU_PREPARE :

        break;

     }

    }

}


    
void dpu_amount_handle()
{
    uint32_t tmp = 0;
    uint8_t  i = 0;

    dfu.amount = 0;
    
    for(i = 0; i < 4; i++)
    {
        tmp = ble_rec_output();
        dfu.amount |= tmp << i*8;
    }

    xprintf("The amount is %d \r\n", dfu.amount);    
    
    system_device_firmware_update();
    
    
}


void ble_response()
{
}

void ble_rec_handle()
{
    static uint32_t i = 0;
    if(get_ble_rec_count() > 0)
    {
        ble_rec.cur_rec_inst = ble_rec_output();
        xprintf("......handle inst is: %d  ..\r\n", ble_rec.cur_rec_inst);
        // ble_sending();
        switch(ble_rec.cur_rec_inst)
        {
        case BLE_DFU_AMOUNT:            
            dpu_amount_handle();
        break;

        default :
            xprintf("what?..\r\n");
            // set_ble_send_state(SEND_CLOSE);
        }
    }
    else
    {
        if(i > 400000)
        {
            xprintf("there is no data in buffer..\r\n");
            i = 0;
        }
        else
        {
             i++; 
        }
            
    }
}



void ble_prepare()
{
    ble_rec_init();
    dfu_init();
}


void ble_close()
{
    sd_softdevice_disable();

    xprintf("ble time out or transport finished, turn out to default mode");

}



/**@brief Error handler function, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t *p_file_name)
{
    //nrf_gpio_pin_set(ASSERT_LED_PIN_NO);
    xprintf("%s(): line: %d, error_code: %d");
    // This call can be used for debug purposes during development of an application.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    //NVIC_SystemReset();
}


/**@brief Assert macro callback function.
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
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler    
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
    err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
    APP_ERROR_CHECK(err_code); */
}

/**@brief Service error handler.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
/*
// YOUR_JOB: Uncomment this function and make it handle error situations sent back to your
//           application by the services it uses.
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
} */

/**@brief GAP initialization.
 *
 * @details This function shall be used to setup all the necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    //                                                                  Ble_uart
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *) DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;   //0.5s
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;   //2s
    gap_conn_params.slave_latency     = SLAVE_LATENCY;       //0
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;    //4s

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    ble_uuid_t adv_uuids[] = {{BLE_UUID_NUS_SERVICE, m_nus.uuid_type}};

    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}

bool data_compare(uint8_t *data1, uint16_t length1, uint8_t *data2, uint16_t length2)
{
    uint16_t i = 0;
    if(length1 != length2)
    {
        return false;
    }
    else
    {
        while(i < length1 && data1[i] == data2[i])
            i++;
        if(i != length1)
            return false;
        return true;
    }
}



void data_handler(ble_nus_t *p_nus, uint8_t *data, uint16_t length)
{
    uint8_t send;
    ble_rec_input(data, length);
         
   // xprintf("current length: %d, data amount is:%d \r\n", length, ble_rec.count);
    
    send = BLE_RESPONSE;
    ble_nus_send_string(&m_nus, &send, 1);   
}


/**@brief Initialize services that will be used by the application.
  * pass the data_handler method to the m_nus->data_handler through the nus_init
 */
static void services_init(void)
{
    uint32_t err_code;
    static ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof nus_init);
    nus_init.data_handler = data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Initialize security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**@brief Connection Parameters Module handler.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt)
{
    uint32_t err_code;

    APP_ERROR_CHECK_BOOL(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED);

    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Connection Parameters module error handler.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Initialize the Connection Parameters module.
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
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Start advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
    //nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}


/**@brief Application's BLE Stack event handler.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */

static void on_ble_evt(ble_evt_t *p_ble_evt)
{
    uint32_t                         err_code = NRF_SUCCESS;
    static ble_gap_evt_auth_status_t m_auth_status;
    ble_gap_enc_info_t              *p_enc_info;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        xprintf("ble is conneting..\r\n");
       
    
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

        /* YOUR_JOB: Uncomment this part if you are using the app_button module to handle button
                     events (assuming that the button events are only needed in connected
                     state). If this is uncommented out here,
                        1. Make sure that app_button_disable() is called when handling
                           BLE_GAP_EVT_DISCONNECTED below.
                        2. Make sure the app_button module is initialized.
        err_code = app_button_enable();
        */
        break;

    case BLE_GAP_EVT_DISCONNECTED:

        xprintf("ble is disconneting..\r\n");
        //nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
        m_conn_handle = BLE_CONN_HANDLE_INVALID;

        /* YOUR_JOB: Uncomment this part if you are using the app_button module to handle button
                     events. This should be done to save power when not connected
                     to a peer.
        err_code = app_button_disable();
        */
        if (err_code == NRF_SUCCESS)
        {
            advertising_start();
        }
        break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        xprintf("ble  3..\r\n");
        err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                               BLE_GAP_SEC_STATUS_SUCCESS,
                                               &m_sec_params);
        break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        xprintf("ble  4..\r\n");
        err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
        break;

    case BLE_GAP_EVT_AUTH_STATUS:
        xprintf("ble  5..\r\n");
        m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
        break;

    case BLE_GAP_EVT_SEC_INFO_REQUEST:
        xprintf("ble  6..\r\n");
        p_enc_info = &m_auth_status.periph_keys.enc_info;
        if (p_enc_info->div == p_ble_evt->evt.gap_evt.params.sec_info_request.div)
        {
            err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, NULL);
        }
        else
        {
            // No keys found for this device
            err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL);
        }
        break;

    case BLE_GAP_EVT_TIMEOUT:
        if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
        {
            //nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);

            // Go to system-off mode (this function will not return; wakeup will cause a reset)
            //GPIO_WAKEUP_BUTTON_CONFIG(WAKEUP_BUTTON_PIN);
            //err_code = sd_power_system_off();
            xprintf("ble time out..\r\n");
            
            
 
        }
        break;

    default:
        break;
    }

    APP_ERROR_CHECK(err_code);
}


/**@brief Dispatches a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t *p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief BLE stack initialization.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    // YOUR_JOB: If the MTU size is changed by the application, the MTU_SIZE parameter to
    //           BLE_STACK_HANDLER_INIT() must be changed accordingly.
    BLE_STACK_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM,
                           BLE_L2CAP_MTU_DEF,
                           ble_evt_dispatch,
                           false);
}

/**@brief Power manager.
 */
// static void power_manage(void)
// {
//     uint32_t err_code = sd_app_event_wait();
//     APP_ERROR_CHECK(err_code);
// }

/**@brief Application main function.
 */
void ble_start(void)
{
    //simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);
    xprintf("System start...\r\n");
    
    timers_init();  //a must or the application can't run normally
    xprintf("timers start...\r\n");
    ble_stack_init();
    xprintf("stack start...\r\n");
    gap_params_init();
    xprintf("gap start...\r\n");
    services_init();
    xprintf("service start...\r\n");
    advertising_init();
    xprintf("advertising start...\r\n");
    conn_params_init();
    xprintf("connect start...\r\n");
    sec_params_init();
    xprintf("security start...\r\n");
    advertising_start();
    xprintf("All components have initialized!\r\n");
    
    ble_rec_init();
    dfu_init();
    
}

/**
 * @}
 */
