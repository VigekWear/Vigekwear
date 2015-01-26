#ifndef __TIMER_RTC_H
#define __TIMER_RTC_H

#include <stdbool.h>
#include <stdint.h>

#define GPIO_TOGGLE_TICK_EVENT    (8)                                     /**< Pin number to toggle when there is a tick event in RTC. */
#define GPIO_TOGGLE_COMPARE_EVENT (9)                                     /**< Pin number to toggle when there is compare event in RTC. */
#define LFCLK_FREQUENCY           (32768UL)                               /**< LFCLK frequency in Hertz, constant. */
#define RTC_FREQUENCY             (8UL)                                   /**< Required RTC working clock RTC_FREQUENCY Hertz. Changable. */
#define COMPARE_COUNTERTIME       (1UL)                                   /**< Get Compare event COMPARE_TIME seconds after the counter starts from 0. */
#define RTC1_PRESCALER            ((LFCLK_FREQUENCY/RTC_FREQUENCY) - 1)   /* f = LFCLK/(prescaler + 1) */
#define RTC1_AUTO_WAKEUP_TIME     (3600 * RTC_FREQUENCY)                  /* Sleep for 1 hour then wake up to detect movement */
#define MAX_RTC1_RESUME_TIMES     10                                      /* This macro defines when the system should enter powerdown mode.*/
#define ALARM_STATUS_INVALID      0
#define ALARM_STATUS_SINGLE_TIME  1
#define ALARM_STATUS_CYCLE        3

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

#endif

