/**
 * \file
 *
 * \brief CONF_APP_EXAMPLE : Example configuration.
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

#ifndef CONF_APP_EXAMPLE_H_INCLUDED
#define CONF_APP_EXAMPLE_H_INCLUDED

/* Application Layer Version
 *          type of application     57 => modem + dual mode
 *          version number          XX
 */
#define PRIME_APP_VERSION           5701

/* USI PORT */
#define MODEM_USI_PORT              0

/* Enable debug report through modem application */
#define PRIME_DEBUG_REPORT

#define PRIME_NUM_REGIONS                 1

/* Define PRIME location in flash. Could be movable */
#define PRIME_APP_SIZE                    0x0001C000
#define PRIME_MAC13_SIZE                  0x00020000
#define PRIME_MAC14_SIZE                  0x00022000
#define PRIME_PHY_SIZE                    0x00018000
#define PRIME_IMAGE_SIZE                  0x00022000
#define PRIME_APP_FLASH_LOCATION          0x00408000
#define PRIME_IMAGE_FLASH_LOCATION        0x00424000
#define PRIME_PHY_FLASH_LOCATION          0x00446000
#define PRIME_MAC14_FLASH_LOCATION        0x0045E000
#define PRIME_MAC13_FLASH_LOCATION        0x0045E000

/* Define app number */
#define PRIME_INVALID_APP                 0
#define PRIME_MAC13_APP                   1
#define PRIME_MAC14_APP                   2
#define PRIME_PHY_APP                     3
#define PRIME_MAIN_APP                    4

/* Define size of metadata = image identifier (12 bytes) + image size (4 bytes) */
#define PRIME_METADATA_SIZE               16

/* Define strings to identify images */
#define PRIME_MAC13_BIN                   "MAC13BIN"
#define PRIME_MAC14_BIN                   "MAC14BIN"
#define PRIME_PHY_BIN                     "PHYBIN"
#define PRIME_APP_BIN                     "APPBIN"

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
