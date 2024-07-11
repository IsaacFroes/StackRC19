/**
 * \file
 *
 * \brief CONF_APP_EXAMPLE : Example configuration.
 *
 * Copyright (c) 2017 Atmel Corporation. All rights reserved.
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

#ifndef CONF_APP_EXAMPLE_H_INCLUDED
#define CONF_APP_EXAMPLE_H_INCLUDED

/* Application Layer Version
 *          type of application     50 => modem
 *          version number          XX
 */
#define PRIME_APP_VERSION           5001

/* USI PORT */
#define MODEM_USI_PORT              0

/* Enable debug report through modem application */
#define PRIME_DEBUG_REPORT

/* Define PRIME location in flash. Could be movable */
#define PRIME_MAX_SIZE_SN (0x00060000u)
#define PRIME_MAX_SIZE_BN (0x00040000u)

#ifndef IFLASH_ADDR 
#define IFLASH_ADDR IFLASH_CNC_ADDR
#endif
#define PRIME_FLASH_LOCATION_SN  (IFLASH_ADDR + IFLASH_SIZE - PRIME_MAX_SIZE_SN)

#if (PRIME_MAX_SIZE_BN + PRIME_MAX_SIZE_SN > IFLASH_SIZE)
#error 'PRIME_SIZE_SN too big. Cannot allocate enough flash space.'
#endif

#ifdef HAL_NWK_RECOVERY_INTERFACE
/* Define size of network recovery region */
#define PRIME_MAX_SIZE_NWK_RECOVERY   (0x00020000u)

/* Define location for network recovery in flash. Could be movable */
#define PRIME_FLASH_LOCATION_NWK_RECOVERY     (PRIME_FLASH_LOCATION_SN - PRIME_MAX_SIZE_NWK_RECOVERY)
#endif

/* \note Defines the toggle period time for the status LED */
/* @{ */
#define APP_SIGNAL_TIMER_RATE       (500 / portTICK_RATE_MS)
#define APP_SIGNAL_BLOCK_TIME       (100 / portTICK_RATE_MS)
/* @} */

/* \note Defines the ON time for the PLC activity LED */
/* @{ */
#define PLC_ACTIVITY_TIMER          (25 / portTICK_RATE_MS)
#define PLC_ACTIVITY_BLOCK_TIME     (5 / portTICK_RATE_MS)
/* @} */

/* \note Defines the show time for the data activity icon */
/* @{ */
#define DATA_ACTIVITY_TIMER         (250 / portTICK_RATE_MS)
#define DATA_ACTIVITY_BLOCK_TIME    (25 / portTICK_RATE_MS)
/* @} */
#endif /* CONF_APP_EXAMPLE_H_INCLUDED */
