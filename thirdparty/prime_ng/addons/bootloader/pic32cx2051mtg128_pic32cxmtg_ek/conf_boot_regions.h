/**
 * \file
 *
 * \brief Bootloader configuration for PIC32CX.
 *
 * Copyright (c) 2020 Atmel Corporation. All rights reserved.
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

#ifndef CONF_BOOT_REGIONS_H_INCLUDED
#define CONF_BOOT_REGIONS_H_INCLUDED

#include "compiler.h"

/* Bootloader address */
#define BOOT_START_ADDRESS                            IFLASH0_CNC_ADDR

/* Bootloader configuration */
#define BOOT_CONFIG_OFFSET_USER_SIGN                  112
#define BOOT_CONFIG_KEY                               0x55AA55AA
#define BOOT_BUFFER_ADDR                              0x0100E000

/* Firmware configuration */
#define BOOT_FLASH_PAGE_SIZE                          IFLASH0_PAGE_SIZE
#define BOOT_FLASH_PAGES_PER_SECTOR                   128
#define BOOT_FLASH_SECTOR_SIZE                        (BOOT_FLASH_PAGES_PER_SECTOR * BOOT_FLASH_PAGE_SIZE)
#define BOOT_FLASH_16PAGE_SIZE                        (BOOT_FLASH_PAGE_SIZE << 4)

/* Region configuration */
#define BOOT_FIRST_SECTOR_START_ADDRESS               (IFLASH0_CNC_ADDR + 0x00010000)
#define BOOT_FLASH_APP_FIRMWARE_START_ADDRESS         BOOT_FIRST_SECTOR_START_ADDRESS
#define BOOT_FLASH_APP_FIRMWARE_RESET_ADDRESS         (BOOT_FIRST_SECTOR_START_ADDRESS + 4)

/* User signature configuration */
#define BOOT_USER_SIGNATURE_BLOCK                     IFLASH_USR_SIG_BLOCK_0
#define BOOT_USER_SIGNATURE_PAGE                      0
#endif /* CONF_BOOT_REGIONS_H_INCLUDED */
