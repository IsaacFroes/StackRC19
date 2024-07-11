/**
 * \file
 *
 * \brief CONF_HAL: Hardware Abstraction Layer (HAL) configuration.
 *
 * Copyright (c) 2019 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef CONF_HAL_H_INCLUDE
#define CONF_HAL_H_INCLUDE

#include "atpl360_hal_spi.h"

/* Disable Reset handling */
#define DISABLE_RESET_HANDLING

/* Disable PIB handling */
#define DISABLE_PIB_HANDLING

/* Disable all security (only SAMG55) */
#define HAL_NO_SECURITY

/* Enable ATPL360 interface */
#define HAL_ATPL360_INTERFACE

/* Configure PRIME HAL version */
/* define HAL PRIME versions */
#define HAL_PRIME_1_3                  1
#define HAL_PRIME_1_4                  2

#define PRIME_HAL_VERSION              HAL_PRIME_1_3

/* UART  */
/** Timers Configuration */
#define HAL_ID_TC_UART                 ID_TC5
#define HAL_TC_UART                    TC1
#define HAL_TC_UART_CHN                2
#define HAL_TC_UART_Handler            TC5_Handler
#define HAL_TC_UART_IRQn               TC5_IRQn
#define HAL_UART0_Handler              UART0_Handler

/** Configuration Size Buffers */
#define HAL_RX_UART_BUF0_SIZE          1024
#define HAL_TX_UART_BUF0_SIZE          1024

#define HAL_RX_UART_BUF1_SIZE          1024
#define HAL_TX_UART_BUF1_SIZE          1024

/* Select HW timer TC for Timer of 1us service */
#define TIMER_1US_TC                   TC0
#define TIMER_1US_TC_CHN               1
#define TIMER_1US_ID_TC                ID_TC1
#define TIMER_1US_TC_Handler           TC1_Handler

/* USART 0 */
#define ID_USART0                      ID_FLEXCOM0
#define USART0_IRQn                    FLEXCOM0_IRQn
#define HAL_USART0_Handler             FLEXCOM0_Handler

/* USART 1 */
#define ID_USART1                      ID_FLEXCOM1
#define USART1_IRQn                    FLEXCOM1_IRQn
#define HAL_USART1_Handler             FLEXCOM1_Handler

/* USART 2 */
#define ID_USART2                      ID_FLEXCOM2
#define USART2_IRQn                    FLEXCOM2_IRQn
#define HAL_USART2_Handler             FLEXCOM2_Handler

/* USART 3 */
#define ID_USART3                      ID_FLEXCOM3
#define USART3_IRQn                    FLEXCOM3_IRQn
#define HAL_USART3_Handler             FLEXCOM3_Handler

/* USART 4 */
#define ID_USART4                      ID_FLEXCOM4
#define USART4_IRQn                    FLEXCOM4_IRQn
#define HAL_USART4_Handler             FLEXCOM4_Handler

/** Timers Configuration */
#define HAL_ID_TC_USART                ID_TC4
#define HAL_TC_USART                   TC1
#define HAL_TC_USART_CHN               1
#define HAL_TC_USART_IRQn              TC4_IRQn
#define HAL_TC_USART_Handler           TC4_Handler

/** Configuration Size Buffers */
#define HAL_RX_USART_BUF0_SIZE         1024
#define HAL_TX_USART_BUF0_SIZE         1024

/** Configuration Size Buffers */
#define HAL_RX_USART_BUF1_SIZE         1024
#define HAL_TX_USART_BUF1_SIZE         1024

/** Configuration Size Buffers */
#define HAL_RX_USART_BUF2_SIZE         1024
#define HAL_TX_USART_BUF2_SIZE         1024

/** Configuration Size Buffers */
#define HAL_RX_USART_BUF3_SIZE         1024
#define HAL_TX_USART_BUF3_SIZE         1024

/** Configuration Size Buffers */
#define HAL_RX_USART_BUF4_SIZE         1024
#define HAL_TX_USART_BUF4_SIZE         1024

/* PLC */
/* Select the SPI module that PLC is connected to */
#define HAL_PLC_SPI_MODULE               ATPL360_SPI

/* Chip select used by PLC internal peripheral  */
#define HAL_PLC_CS                       ATPL360_SPI_CS

/* SPI polarity by PLC internal peripheral  */
#define HAL_PLC_POL                      0

/* SPI phas used by PLC internal peripheral  */
#define HAL_PLC_PHA                      1

/* Programmable Clock Settings (Hz) */
/* < PLC clock setting (ATPL360 in Half Speed Mode) */
#define HAL_PLC_CLOCK                    8000000

/* Interruption pin used by PLC internal peripheral */
#define HAL_PLC_INT_GPIO                 ATPL360_INT_GPIO 
#define HAL_PLC_INT_FLAGS                ATPL360_INT_FLAGS
#define HAL_PLC_INT_SENSE                ATPL360_INT_SENSE

#define HAL_PLC_INT_MASK                 ATPL360_INT_MASK
#define HAL_PLC_INT_PIO                  ATPL360_INT_PIO 
#define HAL_PLC_INT_ID                   ATPL360_INT_ID  
#define HAL_PLC_INT_TYPE                 ATPL360_INT_TYPE
#define HAL_PLC_INT_ATTR                 ATPL360_INT_ATTR
#define HAL_PLC_INT_IRQn                 ATPL360_INT_IRQn

 /* ATPL360 Reset pin definition */
#define HAL_PLC_RESET_GPIO               ATPL360_RESET_GPIO
#define HAL_PLC_RESET_ACTIVE_LEVEL       ATPL360_RESET_ACTIVE_LEVEL
#define HAL_PLC_RESET_INACTIVE_LEVEL     ATPL360_RESET_INACTIVE_LEVEL

/* ATPL360 LDO Enable pin definition */
#define HAL_PLC_LDO_EN_GPIO              ATPL360_LDO_EN_GPIO          
#define HAL_PLC_LDO_EN_ACTIVE_LEVEL      ATPL360_LDO_EN_ACTIVE_LEVEL  
#define HAL_PLC_LDO_EN_INACTIVE_LEVEL    ATPL360_LDO_EN_INACTIVE_LEVEL

/* Carrier detection PL360 */
#define HAL_PLC_CD_GPIO                     ATPL360_CD_GPIO

#endif  /* CONF_HAL_H_INCLUDE */
