/**
 * \file
 *
 * \brief Board configuration.
 *
 * Copyright (c) 2024 Microchip Technology Inc. and its subsidiaries.
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

/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef CONF_BOARD_H_INCLUDED
#define CONF_BOARD_H_INCLUDED

/* Enable Watchdog */
#define CONF_BOARD_KEEP_WATCHDOG_AT_INIT

/* Configure console UART */
/*#define CONF_BOARD_UART_MIKROBUS */
#define CONF_BOARD_UART_CONSOLE
//#define CONSOLE_UART                     USART4
//#define CONSOLE_UART_ID                  ID_FLEXCOM4

/* Configure MIKROBUS_SPI */
/* #define CONF_BOARD_SPI_MIKROBUS */

/* Configure MIKROBUS_TWI */
/* #define CONF_BOARD_TWI1_MIKROBUS */

/* Configure USB */
#define CONF_BOARD_USB_PORT

/** Enable SPI for PLC */
#define CONF_BOARD_SPI
#define CONF_BOARD_SPI_NPCS0
#define CONF_BOARD_SPI_NPCS1

/* Carrier detection PL360 */
#define CONF_BOARD_ATPL360_CD_EN


#endif /* CONF_BOARD_H_INCLUDED */
