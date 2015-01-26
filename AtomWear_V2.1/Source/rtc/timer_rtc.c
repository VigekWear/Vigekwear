#include "nrf.h"
#include <stdlib.h>
#include "boards.h"
#include "timer_rtc.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "app_util_platform.h"

#define DEBUG_DEV
#ifdef DEBUG_DEV
#include "xprintf.h"
#else
#define xprintf(...)
#endif

#define MY_APP_TIMER                    NRF_TIMER1

static timer1_irq_handler_node_t *timer1_irq_handler_list = 0;

static uint32_t standard_time = 0;

static alarm_struct_t alarm = {0, ALARM_STATUS_INVALID};

static uint32_t timer1_cnt = 0;
static uint8_t rtc_is_available = 1;
static uint8_t rtc_resume_cnt = 0;

uint8_t is_time_calibrated = 0;

/**
 * @brief Function for timer initialization.
 */
void timer1_init(void)
{
    timer1_irq_handler_list = 0;
    // Start 16 MHz crystal oscillator.
    //NRF_CLOCK->EVENTS_HFCLKSTARTED  = 0;
    //NRF_CLOCK->TASKS_HFCLKSTART     = 1;

    // Wait for the external oscillator to start up.
    //while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
    //{
        // Do nothing.
    //}

    MY_APP_TIMER->TASKS_STOP     = 1;
    MY_APP_TIMER->MODE           = TIMER_MODE_MODE_Timer;        // Set the timer in Timer Mode.
    MY_APP_TIMER->PRESCALER      = 4;                            // Prescaler 4 produces 1 MHz timer frequency => 1 tick = 1 us.
    MY_APP_TIMER->BITMODE        = TIMER_BITMODE_BITMODE_32Bit;  // 32 bit mode.
    MY_APP_TIMER->TASKS_CLEAR    = 1;                            // clear the task first to be usable for later.

//     if(get_ble_status() == BLE_STATUS_NOTWORK)
//     {
//         NVIC_SetPriority(TIMER1_IRQn, APP_IRQ_PRIORITY_LOW);
//         NVIC_EnableIRQ(TIMER1_IRQn);
//     } else {
//         sd_nvic_SetPriority(TIMER1_IRQn, APP_IRQ_PRIORITY_LOW);
//         sd_nvic_EnableIRQ(TIMER1_IRQn);
//     }
    
    MY_APP_TIMER->CC[0]          = 10000;               // 10ms interrupt cycle
    MY_APP_TIMER->INTENSET       = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);   // Enable compare interrupt
    MY_APP_TIMER->TASKS_START    = 1;                    // Start timer.
    
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

#if 0
extern uint8_t g_sleep_resume_flag;
void TIMER1_IRQHandler(void)
{
    uint32_t tmp;
    //led0_on();
    timer1_irq_handler_node_t *p = timer1_irq_handler_list;
    NRF_TIMER1->TASKS_CLEAR = 1;
    NRF_TIMER1->SHORTS = 1;
    NRF_TIMER1->EVENTS_COMPARE[0] = 0;
    tmp = NRF_TIMER1->CC[0];
    tmp /= 10000;
    timer1_cnt += tmp;
    if(timer1_cnt%100 == 0) {
        // clear g_sleep_resume_flag
        if(g_sleep_resume_flag) g_sleep_resume_flag = 0;
        if(!rtc_is_available)
        {
            standard_time++;
            if(alarm.alarm_time == standard_time && alarm.alarm_status != ALARM_STATUS_INVALID)
            {
//                 misc_output_config(LED1, 15, 100, 25);
//                 misc_output_config(VIBRATOR, 8, 200, 100);
                if(alarm.alarm_status == ALARM_STATUS_CYCLE) {
                    alarm.alarm_time += 86400;
                } else {
                    alarm.alarm_status = ALARM_STATUS_INVALID;
                }
                xprintf("Alarm triggered....!!\n\r");
            }
            xprintf("TIMER1_IRQHandler triggered, standard_time is: %d!!\n\r", standard_time);
        }
    }
    while(p)
    {
        if(p->func != 0)
        {
            if((p->interval == 1) || (timer1_cnt % p->interval == p->phase))
            {
                
                p->func();
            }
        }
        p = p->next;
    }
    //led0_off();
}
#endif

uint32_t get_timer1_cnt(void)
{
    return timer1_cnt;
}

void timer1_suspend(void)
{
    NVIC_DisableIRQ(TIMER1_IRQn);

    MY_APP_TIMER->EVENTS_COMPARE[0] = 0;
    MY_APP_TIMER->INTENCLR = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);
    MY_APP_TIMER->TASKS_STOP = 1;
}

void timer1_resume(void)
{
    MY_APP_TIMER->TASKS_STOP     = 1;
    MY_APP_TIMER->TASKS_CLEAR    = 1;                            // clear the task first to be usable for later.

//     if(get_ble_status() == BLE_STATUS_NOTWORK)
//     {
//         NVIC_SetPriority(TIMER1_IRQn, APP_IRQ_PRIORITY_LOW);
//         NVIC_EnableIRQ(TIMER1_IRQn);
//     } else {
//         sd_nvic_SetPriority(TIMER1_IRQn, APP_IRQ_PRIORITY_LOW);
//         sd_nvic_EnableIRQ(TIMER1_IRQn);
//     }
    
    //MY_APP_TIMER->CC[0]          = 10000;               // 10ms interrupt cycle
    MY_APP_TIMER->INTENSET       = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);   // Enable compare interrupt
    MY_APP_TIMER->TASKS_START    = 1;                    // Start timer.
}


//当前配置下每12分钟慢2秒
void rtc_init()
{
    NRF_CLOCK->LFCLKSRC             = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED  = 0;
    NRF_CLOCK->TASKS_LFCLKSTART     = 1;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        //Do nothing.
    }
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;

    NRF_RTC1->PRESCALER     = RTC1_PRESCALER;                    // Set prescaler to a TICK of RTC_FREQUENCY.
    NRF_RTC1->CC[0]         = COMPARE_COUNTERTIME * RTC_FREQUENCY;  // Compare0 after approx COMPARE_COUNTERTIME seconds.

    // Enable COMPARE0 event and COMPARE0 interrupt:
    NRF_RTC1->EVTENSET      = RTC_EVTENSET_COMPARE0_Msk;
    NRF_RTC1->INTENSET      = RTC_INTENSET_COMPARE0_Msk;

    NVIC_SetPriority(RTC1_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_EnableIRQ(RTC1_IRQn);
    
    NRF_RTC1->TASKS_START   = 1;
    rtc_is_available = 1;
}


 
/** @brief: Function for handling the RTC1 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
void RTC1_IRQHandler(void)
{
    // Clear all events (also unexpected ones)
    //NRF_RTC1->EVENTS_COMPARE[0] = 0;
    NRF_RTC1->EVENTS_COMPARE[1] = 0;
    NRF_RTC1->EVENTS_COMPARE[2] = 0;
    NRF_RTC1->EVENTS_COMPARE[3] = 0;
    //NRF_RTC1->EVENTS_TICK       = 0;
    //NRF_RTC1->EVENTS_OVRFLW     = 0;
    if(rtc_is_available)
    {
        if(0)//(g_sleep_resume_flag)
        {
            //rtc1_resume();
            NRF_RTC1->EVENTS_COMPARE[0] = 0;
            rtc_resume_cnt++;
            xprintf("RTC1 resume %d\n\r", rtc_resume_cnt);
            //if(rtc_resume_cnt > MAX_RTC1_RESUME_TIMES)
            //    set_working_mode(WORKING_MODE_SHUTDOWN);
        } else {
            if ((NRF_RTC1->EVENTS_COMPARE[0] != 0) && 
                ((NRF_RTC1->INTENSET & RTC_INTENSET_COMPARE0_Msk) != 0))
            {
                NRF_RTC1->TASKS_CLEAR = 1;
                NRF_RTC1->EVENTS_COMPARE[0] = 0;
                standard_time++;
                if(alarm.alarm_time == standard_time && alarm.alarm_status != ALARM_STATUS_INVALID)
                {
//                     misc_output_config(LED1, 15, 100, 25);
//                     misc_output_config(VIBRATOR, 8, 200, 100);
                    if(alarm.alarm_status == ALARM_STATUS_CYCLE) {
                        alarm.alarm_time += 86400; // 加上1天的秒数
                    } else {
                        alarm.alarm_status = ALARM_STATUS_INVALID;
                    }
                    xprintf("Alarm triggered....!!\n\r");
                }
                xprintf("RTC1_IRQHandler trigered, standard_time is: %d\n\r", standard_time);
                //nrf_gpio_pin_write(GPIO_TOGGLE_COMPARE_EVENT, 1);
            } else if(NRF_RTC1->EVENTS_OVRFLW) {
                // 由于是8Hz计时当计数溢出归零时也是1s钟
                NRF_RTC1->EVENTS_OVRFLW     = 0;
                standard_time++;
            }
        }
    } else {
        NRF_RTC1->EVENTS_OVRFLW     = 0;
        NRF_RTC1->EVENTS_COMPARE[0] = 0;
        NRF_RTC1->EVENTS_TICK       = 0;
//         timer_timeouts_check();
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
    NRF_RTC1->TASKS_STOP    = 1;
    NRF_RTC1->TASKS_CLEAR   = 1;
    NRF_RTC1->PRESCALER     = RTC1_PRESCALER;                       // Set prescaler to a TICK of RTC_FREQUENCY.
    NRF_RTC1->CC[0]         = COMPARE_COUNTERTIME * RTC_FREQUENCY;  // Compare0 after approx COMPARE_COUNTERTIME seconds.

    // Enable COMPARE0 event and COMPARE0 interrupt:
    NRF_RTC1->EVTENSET      = RTC_EVTENSET_COMPARE0_Msk;
    NRF_RTC1->INTENSET      = RTC_INTENSET_COMPARE0_Msk;
    
    NVIC_SetPriority(RTC1_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_EnableIRQ(RTC1_IRQn);
    
    NRF_RTC1->TASKS_START   = 1;
}

void rtc1_suspend(void)
{
    NRF_RTC1->TASKS_CLEAR   = 1; // clear counter
    NRF_RTC1->CC[0]         = RTC1_AUTO_WAKEUP_TIME;
    nrf_delay_us(100);
}

void rtc1_resume(void)
{
    uint32_t tmp;
    tmp = NRF_RTC1->COUNTER;
    standard_time += (tmp >> 3); // The count frequency is 8Hz, So make it to second. 
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
