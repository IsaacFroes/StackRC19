/**
 * \file
 *
 * \brief CONF_HAL: Hardware Abstraction Layer (HAL) configuration.
 *
 * Copyright (c) 2023 Atmel Corporation. All rights reserved.
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

/* Enable ATPL360 interface */
#define HAL_ATPL360_INTERFACE

/* UART  */
/** Timers Configuration */
#define HAL_ID_TC_UART                 ID_TC5
#define HAL_TC_UART                    TC1
#define HAL_TC_UART_CHN                2
#define HAL_TC_UART_Handler            TC5_Handler
#define HAL_TC_UART_IRQn               TC5_IRQn
#define HAL_UART0_Handler              UART0_Handler
#define HAL_UART2_Handler              UART2_Handler
#define HAL_UART4_Handler              UART4_Handler

/** Configuration Size Buffers */
#define HAL_RX_UART_BUF0_SIZE          1024
#define HAL_TX_UART_BUF0_SIZE          1024

#define HAL_RX_UART_BUF2_SIZE          1024
#define HAL_TX_UART_BUF2_SIZE          1024

#define HAL_RX_UART_BUF4_SIZE          1024
#define HAL_TX_UART_BUF4_SIZE          1024

/* USART */
/** Timers Configuration */
#define HAL_ID_TC_USART                ID_TC4
#define HAL_TC_USART                   TC1
#define HAL_TC_USART_CHN               1
#define HAL_TC_USART_IRQn              TC4_IRQn
#define HAL_TC_USART_Handler           TC4_Handler
#define HAL_USART0_Handler             USART0_Handler
#define HAL_USART1_Handler             USART1_Handler
#define HAL_USART2_Handler             USART2_Handler

/** Configuration Size Buffers */
#define HAL_RX_USART_BUF0_SIZE         1024
#define HAL_TX_USART_BUF0_SIZE         1024

#define HAL_RX_USART_BUF1_SIZE         1024
#define HAL_TX_USART_BUF1_SIZE         1024

#define HAL_RX_USART_BUF2_SIZE         1024
#define HAL_TX_USART_BUF2_SIZE         1024

/* Select HW timer TC for Timer of 1us service */
#define TIMER_1US_TC                   TC0
#define TIMER_1US_TC_CHN               1
#define TIMER_1US_ID_TC                ID_TC1
#define TIMER_1US_TC_Handler           TC1_Handler

/* PLC */
/* Select the SPI module that PLC is connected to */
#define HAL_PLC_SPI_MODULE             SPI0

/* Chip select used by PLC internal peripheral  */
#define HAL_PLC_CS                     2

/* < PLC clock setting (ATPL360 in Half Speed Mode) */
#define HAL_PLC_CLOCK            8000000

#define ATPL360_INT_GPIO                 PIO_PD24_IDX
#define ATPL360_INT_FLAGS                IOPORT_MODE_DEBOUNCE
#define ATPL360_INT_SENSE                IOPORT_SENSE_FALLING

#define ATPL360_INT                      {PIO_PD24, PIOD, ID_PIOD, PIO_INPUT, PIO_DEGLITCH | PIO_IT_LOW_LEVEL}
#define ATPL360_INT_MASK                 PIO_PD24
#define ATPL360_INT_PIO                  PIOD
#define ATPL360_INT_ID                   ID_PIOD
#define ATPL360_INT_TYPE                 PIO_INPUT
#define ATPL360_INT_ATTR                 (PIO_DEGLITCH | PIO_IT_LOW_LEVEL)
#define ATPL360_INT_IRQn                 PIOD_IRQn


/* Interruption pin used by PLC internal peripheral */
#define HAL_PLC_INT_GPIO       (ATPL360_INT_GPIO)
#define HAL_PLC_INT_FLAGS      (ATPL360_INT_FLAGS)
#define HAL_PLC_INT_SENSE      (ATPL360_INT_SENSE)

#define HAL_PLC_INT            ATPL360_INT
#define HAL_PLC_INT_MASK       ATPL360_INT_MASK
#define HAL_PLC_INT_PIO        ATPL360_INT_PIO
#define HAL_PLC_INT_ID         ATPL360_INT_ID
#define HAL_PLC_INT_TYPE       ATPL360_INT_TYPE
#define HAL_PLC_INT_ATTR       ATPL360_INT_ATTR
#define HAL_PLC_INT_IRQn       ATPL360_INT_IRQn

/* Asynchronous PLC Reset pin definition */
//#define HAL_PLC_ARST_GPIO              (PIO_PA23_IDX)
//#define HAL_PLC_ARST_ACTIVE_LEVEL      IOPORT_PIN_LEVEL_LOW
//#define HAL_PLC_ARST_INACTIVE_LEVEL    IOPORT_PIN_LEVEL_HIGH
//
///* Wrapper macros to ensure common naming across all boards */
//#define HAL_PLC_ARST                   {PIO_PA23, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}
//#define HAL_PLC_ARST_MASK              PIO_PA23
//#define HAL_PLC_ARST_PIO               PIOA
//#define HAL_PLC_ARST_ID                ID_PIOA
//#define HAL_PLC_ARST_TYPE              PIO_OUTPUT_1
//#define HAL_PLC_ARST_ATTR              PIO_DEFAULT
//
///* Synchronous PLC Reset pin definition */
//#define HAL_PLC_SRST_GPIO              (PIO_PA24_IDX)
//#define HAL_PLC_SRST_ACTIVE_LEVEL      IOPORT_PIN_LEVEL_LOW
//#define HAL_PLC_SRST_INACTIVE_LEVEL    IOPORT_PIN_LEVEL_HIGH
//
///* Wrapper macros to ensure common naming across all boards */
//#define HAL_PLC_SRST                   {PIO_PA24, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}
//#define HAL_PLC_SRST_MASK              PIO_PA24
//#define HAL_PLC_SRST_PIO               PIOA
//#define HAL_PLC_SRST_ID                ID_PIOA
//#define HAL_PLC_SRST_TYPE              PIO_OUTPUT_1
//#define HAL_PLC_SRST_ATTR              PIO_DEFAULT

/* ATPL360 Reset pin definition */
#define HAL_PLC_RESET_GPIO               ATPL360_RESET_GPIO
#define HAL_PLC_RESET_ACTIVE_LEVEL       ATPL360_RESET_ACTIVE_LEVEL
#define HAL_PLC_RESET_INACTIVE_LEVEL     ATPL360_RESET_INACTIVE_LEVEL

/* ATPL360 LDO Enable pin definition */
#define HAL_PLC_LDO_EN_GPIO              ATPL360_LDO_EN_GPIO
#define HAL_PLC_LDO_EN_ACTIVE_LEVEL      ATPL360_LDO_EN_ACTIVE_LEVEL
#define HAL_PLC_LDO_EN_INACTIVE_LEVEL    ATPL360_LDO_EN_INACTIVE_LEVEL

/* GP Timers */
/* GP Timer 0 is reserved for the PLC PHY driver and therefore, its callback */
/* should not be used here. */
#define HAL_GP_TIMER_PLC_Handler       TC0_Handler
#define HAL_GP_TIMER_PLC_IRQn          TC0_IRQn
#define HAL_GP_TIMER_1_Handler         TC1_Handler
#define HAL_GP_TIMER_1_IRQn            TC1_IRQn
#define HAL_GP_TIMER_2_Handler         TC2_Handler
#define HAL_GP_TIMER_2_IRQn            TC2_IRQn
/* #define HAL_GP_TIMER_3_Handler         TC3_Handler */
/* #define HAL_GP_TIMER_3_IRQn            TC3_IRQn */
/* GP Timers 4 and 5 are reserved for the UART and USART drivers */
/* and therefore, their callback should not be used here. */
/* #define HAL_GP_TIMER_4_Handler       TC4_Handler */
/* #define HAL_GP_TIMER_4_IRQn          TC4_IRQn    */
/* #define HAL_GP_TIMER_5_Handler       TC5_Handler */
/* #define HAL_GP_TIMER_5_IRQn          TC5_IRQn    */
#endif  /* CONF_HAL_H_INCLUDE */
