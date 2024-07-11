/**
 * \file
 *
 * \brief PLCOUP011 PRIME FCC (Channel 3-8) / CENELEC-BCD (Channel 2)
 * Coupling and TX configuration.
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

#ifndef PLCOUP011_CHN_FCC_H_INCLUDED
#define PLCOUP011_CHN_FCC_H_INCLUDED

/**************************************************************************************************************/
/* Coupling and TX parameters for PRIME FCC (Channel 3 - 8) and CENELEC-BCD (Channel 2). **********************/
/* Each channel needs its own calibration *********************************************************************/
/* Recommended to use PLCOUP011_v1 coupling board (FCC Single Branch). ****************************************/
/* PLCOUP011 designed for CENELEC-A and FCC bands. Channel 2 (CENELEC-BCD) supported to support all channels. */
/* IMPORTANT!!! The given values were obtained from calibration with MCHP EKs. ********************************/
/* For customer HW designs, calibration values should be checked with MCHP PHY Calibration Tool. **************/
/**************************************************************************************************************/

/**************************************************************************************************************/
/* PLCOUP011 configuration based on PLCOUP006 *****************************************************************/
/* Redefine thresholds to never change HI / VLO TX mode *******************************************************/
/* Redefine HI gain to be always the same (MAX_RMS_HI not used) ***********************************************/
/**************************************************************************************************************/
#include "plcoup006_chn_fcc.h"

#undef TH_HI_VALUES_CHN_2
#undef TH_HI_VALUES_CHN_3
#undef TH_HI_VALUES_CHN_4
#undef TH_HI_VALUES_CHN_5
#undef TH_HI_VALUES_CHN_6
#undef TH_HI_VALUES_CHN_7
#undef TH_HI_VALUES_CHN_8

#define TH_HI_VALUES_CHN_2                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define TH_HI_VALUES_CHN_3                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define TH_HI_VALUES_CHN_4                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define TH_HI_VALUES_CHN_5                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define TH_HI_VALUES_CHN_6                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define TH_HI_VALUES_CHN_7                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define TH_HI_VALUES_CHN_8                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

#undef IFFT_GAIN_HI_MIN_CHN_2
#undef IFFT_GAIN_HI_MAX_CHN_2
#undef IFFT_GAIN_HI_MIN_CHN_3
#undef IFFT_GAIN_HI_MAX_CHN_3
#undef IFFT_GAIN_HI_MIN_CHN_4
#undef IFFT_GAIN_HI_MAX_CHN_4
#undef IFFT_GAIN_HI_MIN_CHN_5
#undef IFFT_GAIN_HI_MAX_CHN_5
#undef IFFT_GAIN_HI_MIN_CHN_6
#undef IFFT_GAIN_HI_MAX_CHN_6
#undef IFFT_GAIN_HI_MIN_CHN_7
#undef IFFT_GAIN_HI_MAX_CHN_7
#undef IFFT_GAIN_HI_MIN_CHN_8
#undef IFFT_GAIN_HI_MAX_CHN_8
#define IFFT_GAIN_HI_MIN_CHN_2               IFFT_GAIN_HI_INI_CHN_2
#define IFFT_GAIN_HI_MAX_CHN_2               IFFT_GAIN_HI_INI_CHN_2
#define IFFT_GAIN_HI_MIN_CHN_3               IFFT_GAIN_HI_INI_CHN_3
#define IFFT_GAIN_HI_MAX_CHN_3               IFFT_GAIN_HI_INI_CHN_3
#define IFFT_GAIN_HI_MIN_CHN_4               IFFT_GAIN_HI_INI_CHN_4
#define IFFT_GAIN_HI_MAX_CHN_4               IFFT_GAIN_HI_INI_CHN_4
#define IFFT_GAIN_HI_MIN_CHN_5               IFFT_GAIN_HI_INI_CHN_5
#define IFFT_GAIN_HI_MAX_CHN_5               IFFT_GAIN_HI_INI_CHN_5
#define IFFT_GAIN_HI_MIN_CHN_6               IFFT_GAIN_HI_INI_CHN_6
#define IFFT_GAIN_HI_MAX_CHN_6               IFFT_GAIN_HI_INI_CHN_6
#define IFFT_GAIN_HI_MIN_CHN_7               IFFT_GAIN_HI_INI_CHN_7
#define IFFT_GAIN_HI_MAX_CHN_7               IFFT_GAIN_HI_INI_CHN_7
#define IFFT_GAIN_HI_MIN_CHN_8               IFFT_GAIN_HI_INI_CHN_8
#define IFFT_GAIN_HI_MAX_CHN_8               IFFT_GAIN_HI_INI_CHN_8

/* ATPL360_REG_DACC_TABLE_CFG: Configuration for PRIME Channel 2 - 8 with PLCOUP011 (use Branch 1 for VLO and Branch 0 for HI) */
#undef DACC_CFG_TABLE_CHN_FCC
#define DACC_CFG_TABLE_CHN_FCC               {0x00000000, 0x21202120, 0x073F073F, 0x3F3F3F3F, 0x00000FFF, 0x00000000, 0xFFFF00FF, 0x1B1B1B1B, \
					      0x10101010, 0x00001111, 0x04380006, 0x00000355, 0x0F000000, 0x001020FF, 0x000003AA, 0xF0000000, 0x001020FF}

#endif /* PLCOUP011_CHN_FCC_H_INCLUDED */