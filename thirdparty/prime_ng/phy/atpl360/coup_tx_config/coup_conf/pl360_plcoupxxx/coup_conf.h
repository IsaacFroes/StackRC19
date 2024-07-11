/**
 * \file
 *
 * \brief PRIME Coupling Configuration for PL360 + PLCOUP007/006/011
 * (Single and Double Channel supported).
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

#ifndef COUP_CONF_H_INCLUDED
#define COUP_CONF_H_INCLUDED

/* Include PAL / PHY app wrapper */
#include "coup_app_wrp.h"

/**************************************************************************************************/
/* Coupling and TX parameter configuration. Each PRIME channel needs its own calibration. *********/
/* IMPORTANT!!! The given values were obtained from calibration with MCHP EKs. ********************/
/* For customer HW designs, calibration values should be checked with MCHP PHY Calibration Tool. **/
/**************************************************************************************************/

/* Include PLCOUP007 coupling parameters (Channel 1) */
#include "plcoup007_chn_1.h"

#ifndef COUP_CONF_PLCOUP011_ENABLE
/* Include PLCOUP006 coupling parameters (Channel 2 - 8) */
# include "plcoup006_chn_fcc.h"
/* Include PLCOUP006 coupling parameters (Double Channel) */
# include "plcoup006_2chn.h"
#else
/* Include PLCOUP011 coupling parameters (Channel 2 - 8) */
# include "plcoup011_chn_fcc.h"
/* Include PLCOUP011 coupling parameters (Double Channel) */
# include "plcoup011_2chn.h"
#endif

#endif /* COUP_CONF_H_INCLUDED */
