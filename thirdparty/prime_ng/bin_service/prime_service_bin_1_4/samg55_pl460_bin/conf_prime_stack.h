/**
 * \file
 *
 * \brief CONF_PRIME_STACK : PRIME Stack Version Configuration.
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

#ifndef CONF_PRIME_STACK_H_INCLUDE
#define CONF_PRIME_STACK_H_INCLUDE

/* Defined SSCS layer */
#define SSCS_432   0
#define SSCS_IPV6  1
#define USED_SSCS  SSCS_432

/* Configure PRIME API in case to use separated application to compile PRIME stack(binary file) and User Application */
#define PRIME_API_SEPARATED_APPS

/* Firmware Information */
#define PRIME_FW_VENDOR             "MCHP"
#define PRIME_FW_MODEL              "SAMG55XPL460"
#define PRIME_FW_VERSION            "S14.04.01\0\0\0\0\0\0\0"

/* Prime PIB firmware information. FW Version is used as PIB version */
#define PRIME_PIB_VENDOR            0x0000
#define PRIME_PIB_MODEL             0x3D3E
#endif  /* CONF_PRIME_STACK_H_INCLUDE */
