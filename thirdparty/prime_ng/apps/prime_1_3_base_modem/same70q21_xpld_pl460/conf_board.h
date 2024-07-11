/**
 * \file
 *
 * \brief CONF_BOARD : Board configuration.
 *
 * Copyright (c) 2015 Atmel Corporation. All rights reserved.
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

#ifndef CONF_BOARD_H_INCLUDED
#define CONF_BOARD_H_INCLUDED

/* BOARD Rev. */
#define BOARD_REV BOARD_REV_1

/**
 * \name Board oscillator settings
 * @{
 */
/*
#define BOARD_FREQ_SLCK_XTAL        (32768U)
#define BOARD_FREQ_SLCK_BYPASS      (32768U)
#define BOARD_FREQ_MAINCK_BYPASS    (10000000U)
#define BOARD_FREQ_MAINCK_XTAL      (12000000U)
*/

/** Enable watchdog */
#define CONF_BOARD_KEEP_WATCHDOG_AT_INIT

/** Configure Slow Clock as External Crystal */
/* #define CONF_BOARD_32K_XTAL */

/** Enable HAL UART 0 */
//#define CONF_BOARD_UART0
/** Enable HAL UART 2 */
/* #define CONF_BOARD_UART2 */
/** Enable HAL UART 4 */
/* #define CONF_BOARD_UART4 */
/* UART4 pins (UTXD4 and URXD4) definitions PD18,PD19 */
//#define PINS_UART4        (PIO_PD18C_URXD4 | PIO_PD19C_UTXD4)
//#define PINS_UART4_FLAGS  (IOPORT_MODE_MUX_C)
//
//#define PINS_UART4_PORT   IOPORT_PIOD
//#define PINS_UART4_MASK   (PIO_PD18C_URXD4 | PIO_PD19C_UTXD4)
//#define PINS_UART4_PIO    PIOD
//#define PINS_UART4_ID     ID_PIOD
//#define PINS_UART4_TYPE   PIO_PERIPH_C
//#define PINS_UART4_ATTR   PIO_DEFAULT


/* Configure USB */
//#define CONF_BOARD_USB_PORT

/** Enable console port supported ports UART0 ot UART2 **/
//#define CONSOLE_UART      UART2
//#define CONSOLE_UART_ID   ID_UART2
//#define CONF_BOARD_UART_CONSOLE
#define CONF_BOARD_USART1
#define CONF_BOARD_USART1_RXD

/** Enable PLC SPI_0 */
/** Enable SPI0 for PLC */
#define CONF_BOARD_SPI
#define CONF_BOARD_SPI_NPCS1
#define CONF_BOARD_SPI_NPCS3


/* enable SDRAM */
#define CONF_BOARD_SDRAMC

/** Enable Xplain PRO SLP pin */
/* #define CONF_BOARD_XP_SLP */

/** Enable TCM - */
/* #define ENABLE_TCM */

/** Enable MPU */
#define CONF_BOARD_CONFIG_MPU_AT_INIT 

/** Enable Cache */
#define CONF_BOARD_ENABLE_CACHE 

#define MPU_HAS_NOCACHE_REGION

#define NOCACHE_SRAM_REGION_SIZE            0x8000

/** Enable external flash on-board AT45DB321E-SHF2B-T */
/* #define CONF_BOARD_AT45DBX */

/* Configure ATPL360 pins */
#define CONF_BOARD_ATPL360
#define CONF_BOARD_ATPL360_CD_EN
#endif /* CONF_BOARD_H_INCLUDED */
