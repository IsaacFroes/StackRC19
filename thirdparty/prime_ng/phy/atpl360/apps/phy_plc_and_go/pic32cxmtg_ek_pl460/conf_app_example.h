/**
 * \file
 *
 * \brief CONF_EXAMPLE : Example configuration for PLC Application
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

#ifndef CONF_EXAMPLE_H
#define CONF_EXAMPLE_H

/* PLCOUP011: Use only Branch 1 (same as PLCOUP006 VLO branch) for Channels 2 - 8 (FCC and Channel 2) ************/
/* If the following line is uncommented, PLCOUP011 is configured *************************************************/
/* #define CONF_APP_ENABLE_C11_CFG */

/* Definition of channel availability */
#define CHANNEL1                    0x01
#define CHANNEL2                    0x02
#define CHANNEL3                    0x04
#define CHANNEL4                    0x08
#define CHANNEL5                    0x10
#define CHANNEL6                    0x20
#define CHANNEL7                    0x40
#define CHANNEL8                    0x80

/* Configure allowed channels and initial channel. It should be coherent with the hardware used (coupling board) */
#ifdef CONF_APP_ENABLE_C11_CFG
/* PLCOUP011: Allow all channels (Branch 0 for Channel 1; Branch 1 for Channels 2 to 8). */
#define USER_BAND_PLAN              (CHANNEL1 | CHANNEL2 | CHANNEL3 | CHANNEL4 | CHANNEL5 | CHANNEL6 | CHANNEL7 | CHANNEL8)
#define CONF_PRIME_CHANNEL_INI      1
#else
/* PLCOUP007: Only allow Channel 1 (CENELEC-A) */
#define USER_BAND_PLAN              (CHANNEL1)
#define CONF_PRIME_CHANNEL_INI      1
/* PLCOUP006: Allow Channels 3-8 (FCC) */
/* #define USER_BAND_PLAN              (CHANNEL3 | CHANNEL4 | CHANNEL5 | CHANNEL6 | CHANNEL7 | CHANNEL8) */
/* #define CONF_PRIME_CHANNEL_INI      3 */
#endif

#endif /* CONF_EXAMPLE_H */
