/**
 * \file
 *
 * \brief CONF_PAL : PAL layer configuration
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

#ifndef CONF_PAL_H_INCLUDED
#define CONF_PAL_H_INCLUDED

/* Select PORT to serialize PHY sniffer */
#define PHY_SNIFFER_USI_PORT                0

/* Definition of available PHY layers */
#define PAL_PLC
#define PAL_SERIAL

#ifdef PAL_PLC

/* If coupling 11 is chosen */
/* #define PAL_ENABLE_C11_CFG */

/* Definition of channel availability */
#define CHANNEL1        0x01
#define CHANNEL2        0x02
#define CHANNEL3        0x04
#define CHANNEL4        0x08
#define CHANNEL5        0x10
#define CHANNEL6        0x20
#define CHANNEL7        0x40
#define CHANNEL8        0x80

/* Define the band plan */
#define USER_BAND_PLAN  (CHANNEL1)

#endif /* PAL_PLC */
#endif  /* CONF_PAL_H_INCLUDED */
