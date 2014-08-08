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
#ifndef BOARDS_H
#define BOARDS_H

#define LED0_PIN_NUMBER                 18
#define LED1_PIN_NUMBER                 19

#define BUTTON0_PIN_NUMBER              16
#define BUTTON1_PIN_NUMBER              17

#define RX_PIN_NUMBER        12 // 0
#define TX_PIN_NUMBER        11 // 2
#define CTS_PIN_NUMBER       0  // not used but need to be defined.
#define RTS_PIN_NUMBER       0  // not used but need to be defined.
#define HWFC                 false

#define TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER  (7U) // (21U)
#define TWI_MASTER_CONFIG_DATA_PIN_NUMBER   (8U) // (22U)

/*  SPI0 */
#define SPI_FLASH_PORT            0
#define SPI_FLASH_BASE            0x40003000UL
#define SPI_PSELSCK0              9   /*!< GPIO pin number for SPI clock (note that setting this to 31 will only work for loopback purposes as it not connected to a pin) */
#define SPI_PSELMOSI0             10   /*!< GPIO pin number for Master Out Slave In    */
#define SPI_PSELMISO0             11   /*!< GPIO pin number for Master In Slave Out    */
#define SPI_PSELSS0               8   /*!< GPIO pin number for Slave Select           */
/*  SPI1 */   // not used but need to be defined.
#define SPI_PSELSCK1              0   /*!< GPIO pin number for SPI clock              */
#define SPI_PSELMOSI1             0   /*!< GPIO pin number for Master Out Slave In    */
#define SPI_PSELMISO1             0   /*!< GPIO pin number for Master In Slave Out    */
#define SPI_PSELSS1               0   /*!< GPIO pin number for Slave Select           */

#define VIBRATOR_PIN_NUMBER             20  //17
#define CHARGE_DET_PIN_NUMBER           23
#define POWER_GOOD_PIN_NUMBER           24

#define BAT_ADC_PIN_NUMBER              5
#define ADC_EXT_REF_PIN_NUMBER          6
#define BAT_PULLUP_RES                  100
#define BAT_PULLDOWN_RES                100

#define ACCE_TAP_PIN_NUMBER             12

#define gpio_sleep_INT                  6
#define GREEN_LED                       17 // 28
#define RED_LED                         18 // 29

#endif
