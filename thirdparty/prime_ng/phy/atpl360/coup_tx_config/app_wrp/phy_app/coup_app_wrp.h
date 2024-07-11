/**
 * \file
 *
 * \brief Coupling configuration PHY app wrapper.
 *
 * Copyright (C) 2020 Atmel Corporation. All rights reserved.
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

#ifndef COUP_APP_WRP_H_INCLUDED
#define COUP_APP_WRP_H_INCLUDED

/* PHY app configuration include */
#include "conf_app_example.h"

/* PL460 + PLCOUP007 (Single Branch) / PL460 CEN-A (Single Branch) configuration */
#ifdef CONF_APP_ENABLE_PL460_CEN_A_CFG
# define COUP_CONF_PL460_CEN_A_ENABLE
#endif

/* PL460 FCC-SB (Single Branch) / FCC-1.5B (1.5 Branch) configuration */
#ifdef CONF_APP_ENABLE_PL460_FCC_1_5B_CFG
# define COUP_CONF_PL460_FCC_1_5B_ENABLE
#endif

/* PLCOUP006 (Double Branch) / PLCOUP011 (Single Branch) */
#ifdef CONF_APP_ENABLE_C11_CFG
# define COUP_CONF_PLCOUP011_ENABLE
#endif

/* CENELEC-A (Channel 1) configuration enabled, depending on band plan */
#if ((!defined(USER_BAND_PLAN)) || (USER_BAND_PLAN & CHANNEL1))
# define COUP_CONF_CHN_1_ENABLE
#endif

/* FCC / CENELEC-BCD (Channel 2 - 8) configuration enabled, depending on band plan */
#define CHANNELS_FCC     (CHANNEL2 | CHANNEL3 | CHANNEL4 | CHANNEL5 | CHANNEL6 | CHANNEL7 | CHANNEL8)
#if ((!defined(USER_BAND_PLAN)) || (USER_BAND_PLAN & CHANNELS_FCC))
# define COUP_CONF_CHN_FCC_ENABLE
#endif

/* Double Channel enabled depending on configuration */
#ifdef CONF_APP_ENABLE_MULTICHANNEL
# define COUP_CONF_2CHN_ENABLE
#endif

#endif /* COUP_APP_WRP_H_INCLUDED */
