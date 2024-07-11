/**
 * \file
 *
 * \brief CONF_HAL: Hardware Abstraction Layer (HAL) configuration.
 *
 * Copyright (c) 2021 Atmel Corporation. All rights reserved.
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

/* Enable ATPL360 interface */
#define HAL_ATPL360_INTERFACE

/* Configure PRIME HAL version */
/* define HAL PRIME versions */
#define HAL_PRIME_1_3                  1
#define HAL_PRIME_1_4                  2

#define PRIME_HAL_VERSION              HAL_PRIME_1_3

/* UART  */
/** Timers Configuration */
#define HAL_ID_TC_UART                 ID_TC1_CHANNEL2
#define HAL_TC_UART                    TC1
#define HAL_TC_UART_CHN                2
#define HAL_TC_UART_Handler            TC1_CHANNEL2_Handler
#define HAL_TC_UART_IRQn               TC1_CHANNEL2_IRQn
#define ID_UART0                       ID_UART
#define HAL_UART0_Handler              UART_Handler
#define UART0_IRQn                     UART_IRQn

/** Configuration Size Buffers */
#define HAL_RX_UART_BUF0_SIZE          1024
#define HAL_TX_UART_BUF0_SIZE          1024

#define HAL_RX_UART_BUF1_SIZE          1024
#define HAL_TX_UART_BUF1_SIZE          1024

/* USART */
/** Timers Configuration */
#define HAL_ID_TC_USART                ID_TC1_CHANNEL1
#define HAL_TC_USART                   TC1
#define HAL_TC_USART_CHN               1
#define HAL_TC_USART_IRQn              TC1_CHANNEL1_IRQn
#define HAL_TC_USART_Handler           TC1_CHANNEL1_Handler
#define ID_USART0                      ID_FLEXCOM0
#define ID_USART1                      ID_FLEXCOM1
#define ID_USART2                      ID_FLEXCOM2
#define HAL_USART0_Handler             FLEXCOM0_Handler
#define HAL_USART1_Handler             FLEXCOM1_Handler
#define HAL_USART2_Handler             FLEXCOM2_Handler
#define USART0_IRQn                    FLEXCOM0_IRQn
#define USART1_IRQn                    FLEXCOM1_IRQn
#define USART2_IRQn                    FLEXCOM2_IRQn   

/** Configuration Size Buffers */
#define HAL_RX_USART_BUF0_SIZE         1024
#define HAL_TX_USART_BUF0_SIZE         1024

#define HAL_RX_USART_BUF1_SIZE         1024
#define HAL_TX_USART_BUF1_SIZE         1024

#define HAL_RX_USART_BUF2_SIZE         1024
#define HAL_TX_USART_BUF2_SIZE         1024

/* Select HW timer TC for Timer of 1us service */
#define TIMER_1US_TC                   TC2
#define TIMER_1US_TC_CHN               0
#define TIMER_1US_ID_TC                ID_TC2_CHANNEL0
#define TIMER_1US_TC_Handler           TC2_CHANNEL0_Handler

/* PLC */
/* Select the SPI module that PLC is connected to */
#define HAL_PLC_SPI_MODULE               SPI5

/* Chip select used by PLC internal peripheral  */
#define HAL_PLC_CS                       0

/* SPI polarity by PLC internal peripheral  */
#define HAL_PLC_POL                      0

/* SPI phas used by PLC internal peripheral  */
#define HAL_PLC_PHA                      1

/* Programmable Clock Settings (Hz) */
/* < PLC clock setting (ATPL360 in Half Speed Mode) */
#define HAL_PLC_CLOCK                    8000000

/* Interruption pin used by PLC internal peripheral */
#define HAL_PLC_INT_GPIO                 XPLAIN_PRO_PIN_9
#define HAL_PLC_INT_FLAGS                IOPORT_MODE_DEBOUNCE
#define HAL_PLC_INT_SENSE                IOPORT_SENSE_FALLING

#define HAL_PLC_INT_MASK                 PIO_PA2
#define HAL_PLC_INT_PIO                  PIOA
#define HAL_PLC_INT_ID                   ID_PIOA
#define HAL_PLC_INT_TYPE                 PIO_INPUT
#define HAL_PLC_INT_ATTR                 PIO_IT_LOW
#define HAL_PLC_INT_IRQn                 PIOA_IRQn

 /* ATPL360 Reset pin definition */
#define HAL_PLC_RESET_GPIO               XPLAIN_PRO_PIN_7
#define HAL_PLC_RESET_ACTIVE_LEVEL       IOPORT_PIN_LEVEL_LOW
#define HAL_PLC_RESET_INACTIVE_LEVEL     IOPORT_PIN_LEVEL_HIGH

/* ATPL360 LDO Enable pin definition */
#define HAL_PLC_LDO_EN_GPIO              XPLAIN_PRO_PIN_8
#define HAL_PLC_LDO_EN_ACTIVE_LEVEL      IOPORT_PIN_LEVEL_HIGH
#define HAL_PLC_LDO_EN_INACTIVE_LEVEL    IOPORT_PIN_LEVEL_LOW

/* ATPL360 STBY pin definition */
#define HAL_PLC_STBY_GPIO                XPLAIN_PRO_PIN_11
#define HAL_PLC_STBY_ACTIVE_LEVEL        IOPORT_PIN_LEVEL_HIGH
#define HAL_PLC_STBY_INACTIVE_LEVEL      IOPORT_PIN_LEVEL_LOW

/* PL460 TX_ENABLE pin definition (old CD pin) */
#define HAL_PLC_TX_ENABLE_GPIO           XPLAIN_PRO_PIN_12
#define HAL_PLC_TX_ENABLE_ACTIVE_LEVEL   IOPORT_PIN_LEVEL_HIGH
#define HAL_PLC_TX_ENABLE_INACTIVE_LEVEL IOPORT_PIN_LEVEL_LOW

/* ADC channel for PVDD monitor */
#define HAL_PLC_PVDD_MON_ADC_CHN         ADC_CHANNEL_4

/* PL460 NTHW0 pin definition */
#define HAL_PLC_NTHW0_GPIO               XPLAIN_PRO_PIN_10

#endif  /* CONF_HAL_H_INCLUDE */