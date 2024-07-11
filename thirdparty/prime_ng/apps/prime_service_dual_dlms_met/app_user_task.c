/**
 * \file
 *
 * \brief APP_USER_TASK : APP User Task
 *
 * Copyright (c) 2024 Atmel Corporation. All rights reserved.
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

#include <string.h>
#include <stdio.h>

#include "app_emu.h"
#ifdef ENABLE_DIRECT_CON_EXAMPLE_APP
#include "app_dc.h"
#endif
#include "dlms_app.h"
#include "app.h"

/** user aplication mode status */
static uint8_t suc_app_mode;

/**
 * \brief return the application mode
 *
 * \retval application mode status
 */
uint8_t get_app_mode(void)
{
	return suc_app_mode;
}

/**
 * \brief application task initialization. read pin to set the application mode to run
 */
void vUserAppInitTask(uint8_t uc_prime_version)
{
	/* set dlms emu mode */
	suc_app_mode = DLMS_APP_MODE;

	/* Configure APP from user */
#ifndef _PRIME_SIM_
	if (!pio_get(PIN_APPDLMSEMU_PIO, PIN_APPDLMSEMU_TYPE, PIN_APPDLMSEMU_MASK)) {
		suc_app_mode = APP_EMU_MODE;
		/* in APP_EMU MODE are running the DLMS_EMU task and the APP_EMU task */
		app_emu_init();
	}
#endif
	dlms_app_init(uc_prime_version);

#ifdef ENABLE_DIRECT_CON_EXAMPLE_APP
	/* Init APP DC application */
	app_dc_init();
#endif
}

///**
// * \brief reset application task reading aplication mode to reset the tasks
// */
//void vUserAppResetTask(void)
//{
//	if (get_app_mode() == APP_EMU_MODE) {
//		/* in APP_EMU MODE are running the DLMS_EMU task and the APP_EMU task */
//		app_emu_init();
//	}
//
//	dlms_app_init();
//
//#ifdef ENABLE_DIRECT_CON_EXAMPLE_APP
//	/* Init APP DC application */
//	app_dc_init();
//#endif
//}
