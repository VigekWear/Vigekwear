 /* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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

#ifndef SER_CONFIG_H__
#define SER_CONFIG_H__

#include <stdint.h>

/***********************************************************************************************//**
 * General parameters configuration.
 **************************************************************************************************/

/** Value used as error code on SoftDevice stack dump. Can be used to identify stack location on
 *  stack unwind.*/
#define SER_SD_ERROR_CODE    (uint32_t)(0xDEADBEEF)

/** Value used as error code indicating warning - unusual situation but not critical so system
 *  should NOT be reseted. */
#define SER_WARNING_CODE     (uint32_t)(0xBADDCAFE)

/***********************************************************************************************//**
 * HAL Transport layer configuration.
 **************************************************************************************************/

/** Max packets size in serialization HAL Transport layer (packets before adding PHY header i.e.
 *  packet length). */
#define SER_HAL_TRANSPORT_APP_TO_CONN_MAX_PKT_SIZE    (uint32_t)(512)
#define SER_HAL_TRANSPORT_CONN_TO_APP_MAX_PKT_SIZE    (uint32_t)(512)

#ifdef SER_CONNECTIVITY
    #define SER_HAL_TRANSPORT_TX_MAX_PKT_SIZE         SER_HAL_TRANSPORT_CONN_TO_APP_MAX_PKT_SIZE
    #define SER_HAL_TRANSPORT_RX_MAX_PKT_SIZE         SER_HAL_TRANSPORT_APP_TO_CONN_MAX_PKT_SIZE

#else /* APPLICATION SIDE */
    #define SER_HAL_TRANSPORT_TX_MAX_PKT_SIZE         SER_HAL_TRANSPORT_APP_TO_CONN_MAX_PKT_SIZE
    #define SER_HAL_TRANSPORT_RX_MAX_PKT_SIZE         SER_HAL_TRANSPORT_CONN_TO_APP_MAX_PKT_SIZE
#endif /* SER_CONNECTIVITY */


/***********************************************************************************************//**
 * SER_PHY layer configuration.
 **************************************************************************************************/

#define SER_PHY_HEADER_SIZE             2

/** Max transfer unit for SPI MASTER and SPI SLAVE. */
#define SER_PHY_SPI_MTU_SIZE            255

/** UART transmission parameters */
#define SER_PHY_UART_FLOW_CTRL          APP_UART_FLOW_CONTROL_ENABLED
#define SER_PHY_UART_PARITY             true
#define SER_PHY_UART_BAUDRATE           UART_BAUDRATE_BAUDRATE_Baud1M

/** Configuration timeouts of connectivity MCU */
#define CONN_CHIP_RESET_TIME            50      /**< The time to keep the reset line to the nRF51822 low (in milliseconds). */
#define CONN_CHIP_WAKEUP_TIME           500     /**< The time for nRF51822 to reset and become ready to receive serialized commands (in milliseconds). */

#endif /* SER_CONFIG_H__ */
