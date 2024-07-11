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
#define PAL_RF

#ifdef PAL_PLC

/* Component ATPL360 Binary Information */
#define ATPL360_BINARY_ADDRESS                     0x00446000
#define ATPL360_BINARY_LEN                         0x18000

/* PL460 + PLCOUP007 (Single Branch) / PL460 CEN-A (Single Branch) configuration */
/* #define PAL_ENABLE_PL460_CEN_A_CFG */

/* PL460 FCC-SB (Single Branch) / FCC-1.5B (1.5 Branch) configuration */
/* #define PAL_ENABLE_PL460_FCC_1_5B_CFG */

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
/*#define USER_BAND_PLAN  (CHANNEL1) */

/* Enable Multichannel */
/* #define PAL_ENABLE_MULTICHANNEL */

#ifdef PAL_ENABLE_MULTICHANNEL
/* Maximum number of channels supported */
#define PAL_MAX_NUM_CHANNELS         2 /* Double channel */

/* Definition of channel availability */
#define CHANNEL12       0x01
#define CHANNEL23       0x02
#define CHANNEL34       0x04
#define CHANNEL45       0x08
#define CHANNEL56       0x10
#define CHANNEL67       0x20
#define CHANNEL78       0x40

/* Define the band plan */
/*#define USER_BAND_PLAN_2CH  (CHANNEL34) */

#endif /* PAL_ENABLE_MULTICHANNEL */
#endif /* PAL_PLC */

#ifdef PAL_RF
#include "at86rf.h"

/* RF Operation definition */
#define RF_PHY_OPERATION_MODE AT86RF_SUN_FSK_BAND_863_OPM1
/* #define RF_PHY_OPERATION_MODE AT86RF_SUN_OFDM_BAND_863_OPT4 */

/* RSSI Thresholds */
#define RF_THRESHOLD_FSK_FEC_OFF     (-89)
#define RF_THRESHOLD_FSK_FEC_ON      (-94)

/* RF RM mode */
#define RF_RM_AUTOMATIC    0
#define RF_RM_FORCED_OFF   1
#define RF_RM_FORCED_ON    2

#define RF_RM_MODE         RF_RM_FORCED_OFF

/* RF_FREQUENCY_HOPPING_ENABLED */
//#define RF_FREQUENCY_HOPPING_ENABLED

#ifdef RF_FREQUENCY_HOPPING_ENABLED

/* Channel initial and final for each range, and number of ranges */
#define NUM_RANGES_CHANNELS_INCLUDED_SEQUENCE   1
#define RANGES_CHANNELS_INCLUDED_SEQUENCE       {{ 1, 67}}

/* Excluded channels */
#define NUM_CHANNELS_EXCLUDED                   1
#define CHANNELS_EXLUDED                        {34}

/* Num channels MAC_HOPPING_SEQUENCE_LENGTH */
#define MAC_HOPPING_SEQUENCE_LENGTH             66

/* Channels for BCN_SEQUENCE, maximum 32 channels */
#define NUM_CHANNELS_BCN_SEQUENCE               3
#define CHANNELS_BCN_SEQUENCE                   { 0, 34, 68}

/* Define MAC_HOPPING_BCN_SEQUENCE_LENGTH */
#define MAC_HOPPING_BCN_SEQUENCE_LENGTH         3
#else
#define RF_TX_RX_CHANNEL                        0
#endif

#endif /* PAL_RF */
#endif  /* CONF_PAL_H_INCLUDED */
