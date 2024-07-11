/**
 *
 * \file
 *
 * \brief CONF_HAL: Hardware Abstraction Layer (HAL) configuration.
 *
 * Copyright (c) 2023 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

#ifndef CONF_HAL_H_INCLUDE
#define CONF_HAL_H_INCLUDE

#include "conf_board.h"
#include "atpl360_hal_spi.h"

/* Enable ATPL360 interface */
#define HAL_ATPL360_INTERFACE

/* Enable RF interface */
#define HAL_ENABLE_PHY_RF

/* Enable this MACRO to perform signature checks during the firmware upgrade */
#define HAL_FU_ENABLE_SIGNATURE

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
#define HAL_PLC_SPI_MODULE               SPI5
#define HAL_PLC_SPI_ID                   ID_FLEXCOM5

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
#define HAL_PLC_INT_GPIO                 EXT1_PIN_9
#define HAL_PLC_INT_FLAGS                IOPORT_MODE_DEBOUNCE
#define HAL_PLC_INT_SENSE                IOPORT_SENSE_FALLING

#define HAL_PLC_INT_MASK                 PIO_PA24
#define HAL_PLC_INT_PIO                  PIOA
#define HAL_PLC_INT_ID                   ID_PIOA
#define HAL_PLC_INT_TYPE                 PIO_INPUT
#define HAL_PLC_INT_ATTR                 (PIO_DEGLITCH | PIO_IT_LOW_LEVEL)
#define HAL_PLC_INT_IRQn                 PIOA_IRQn

 /* ATPL360 Reset pin definition */
#define HAL_PLC_RESET_GPIO               EXT1_PIN_7
#define HAL_PLC_RESET_ACTIVE_LEVEL       IOPORT_PIN_LEVEL_LOW
#define HAL_PLC_RESET_INACTIVE_LEVEL     IOPORT_PIN_LEVEL_HIGH

/* ATPL360 LDO Enable pin definition */
#define HAL_PLC_LDO_EN_GPIO              EXT1_PIN_8
#define HAL_PLC_LDO_EN_ACTIVE_LEVEL      IOPORT_PIN_LEVEL_HIGH
#define HAL_PLC_LDO_EN_INACTIVE_LEVEL    IOPORT_PIN_LEVEL_LOW

/* ATPL360 STBY pin definition */
#define HAL_PLC_STBY_GPIO                EXT1_PIN_11
#define HAL_PLC_STBY_ACTIVE_LEVEL        IOPORT_PIN_LEVEL_HIGH
#define HAL_PLC_STBY_INACTIVE_LEVEL      IOPORT_PIN_LEVEL_LOW

/* PL460 TX_ENABLE pin definition (old CD pin) */
#define HAL_PLC_TX_ENABLE_GPIO           EXT1_PIN_12
#define HAL_PLC_TX_ENABLE_ACTIVE_LEVEL   IOPORT_PIN_LEVEL_HIGH
#define HAL_PLC_TX_ENABLE_INACTIVE_LEVEL IOPORT_PIN_LEVEL_LOW

/* ADC channel for PVDD monitor */
#define HAL_PLC_PVDD_MON_ADC_CHN         ADC_CHANNEL_0

/* PL460 NTHW0 pin definition */
#define HAL_PLC_NTHW0_GPIO               EXT1_PIN_10

#ifdef HAL_ENABLE_PHY_RF
/* Select the SPI module and Chip Select that PRF is connected to */
#define PRF_SPI_MODULE             SPI5
#define PRF_SPI_CS                 1
#define PRF_SPI_ID                 ID_FLEXCOM5
#define PRF_SPI_Handler            FLEXCOM5_Handler

/* Interrupt pin definition */
#define PRF_INT_GPIO               EXT3_PIN_9
#define PRF_INT_ATTR               (PIO_DEGLITCH | PIO_IT_HIGH_LEVEL)

/* Reset pin definition */
#define PRF_RESET_GPIO             EXT3_PIN_7
#define PRF_RESET_ACTIVE_LEVEL     IOPORT_PIN_LEVEL_LOW
#define PRF_RESET_INACTIVE_LEVEL   IOPORT_PIN_LEVEL_HIGH

/* LED pins definition */
#define PRF_LED_TX_GPIO            EXT3_PIN_6
#define PRF_LED_RX_GPIO            EXT3_PIN_5
#define PRF_LED_ACTIVE_LEVEL       IOPORT_PIN_LEVEL_HIGH
#define PRF_LED_INACTIVE_LEVEL     IOPORT_PIN_LEVEL_LOW

/* Maximum message length to be used by upper layers */
/* RF215 buffer size is 2047 bytes */
/* It should be the same as AT86RF215_MAX_PSDU_LEN */
#define PRF_TRX_MAX_MSG_LEN        571

/* Enable auto configuration of SPI clock frequency and delays */
/* The auto configuration finds best choice to comply with RF215 requirements */
#define PRF_ENABLE_SPICLK_AUTO_CONFIGURE

#ifndef PRF_ENABLE_SPICLK_AUTO_CONFIGURE
/* SPI clock frequency and delay if auto configuration is disabled */
# define PRF_SPI_CLOCK             13700000
# define PRF_SPI_DLYBS             5
# define PRF_SPI_DLYBCT            1
#endif
#endif /* HAL_ENABLE_PHY_RF */

#endif  /* CONF_HAL_H_INCLUDE */
