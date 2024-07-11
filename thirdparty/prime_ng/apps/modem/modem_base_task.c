/**
 * \file
 *
 * \brief PRIME_TASK : Task to run PRIME Stack
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

#include <string.h>
#include <stdio.h>

/* From module: FreeRTOS - Kernel 7.3.0 */
#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>
#include <StackMacros.h>
#include <croutine.h>
#include <list.h>
#include <mpu_wrappers.h>
#include <portable.h>
#include <projdefs.h>
#include <queue.h>
#include <semphr.h>
#include <task.h>
#include <timers.h>

#include "app.h"
#include "modem.h"
#include "asf.h"
#include "conf_app_example.h"
#if !PIC32CX
#include "wdt.h"
#else
#include "dwdt.h"
#endif

#ifdef __PRIME_GATEWAY__
#include "core/net.h"
#endif

/** PRIME Task Rate */
#define MODEM_TASK_RATE          (5 / portTICK_RATE_MS)

/** PRIME Task Stack priority */
#define TASK_MODEM_PRIO          (tskIDLE_PRIORITY + 2)

/** PRIME Task Stack definition */
#define TASK_MODEM_STACK         (configMINIMAL_STACK_SIZE * 6)

/** Tasks handlers */
xTaskHandle xMODEMHnd;

#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE
#if (BOARD != ATPL360AMB) && (BOARD != ATPL360MB) 
/** Data Signaling Timers */
static xTimerHandle xDataRxActivityTimer = NULL;
static xTimerHandle xDataTxActivityTimer = NULL;

/**
 * \brief Task to data reception activity signalling
 *
 * \param pxTimer    Timer
 */
static void _dataRx_activity(xTimerHandle pxTimer)
{
	UNUSED(pxTimer);
#if (BOARD != PIC32CXMTSH_DB) && (BOARD != PIC32CXMTC_DB)
	c42364a_clear_icon(C42364A_ICON_AUDIO);
#else
	cl010_clear_icon(CL010_ICON_PHASE_3);  
#endif
}

/**
 * \brief Task to data transmission activity signalling
 *
 * \param pxTimer    Timer
 */
static void _dataTx_activity(xTimerHandle pxTimer)
{
	UNUSED(pxTimer);
#if (BOARD != PIC32CXMTSH_DB) && (BOARD != PIC32CXMTC_DB)
	c42364a_clear_icon(C42364A_ICON_WLESS);
#else
	cl010_clear_icon(CL010_ICON_PHASE_2);  
#endif
}

#endif
#endif

/**
 * \brief MODEM task
 *
 * \param pvParameters     Pointer to entry parameters
 */
static void task_modem(void *pvParameters)
{
	static portTickType xLastWakeTime;
	static portTickType xPeriod;

	(void)(pvParameters);

	xPeriod = MODEM_TASK_RATE;
	xLastWakeTime = xTaskGetTickCount();
	for (;;) {
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
#ifndef _PRIME_SIM_
		/* Restart watchdog */
#if !PIC32CX
		wdt_restart(WDT);
#else
		dwdt_restart(DWDT, WDT0_ID);
#endif
#endif

#ifdef __PRIME_GATEWAY__
		systemTicks += MODEM_TASK_RATE;
#endif

		/* Process modem emu application */
		modem_process();

#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE
#if (BOARD != ATPL360AMB) && (BOARD != ATPL360MB) 
		if (modem_txdata_ind()) {
#if (BOARD != PIC32CXMTSH_DB) && (BOARD != PIC32CXMTC_DB)
			c42364a_show_icon(C42364A_ICON_WLESS);
#else
			cl010_show_icon(CL010_ICON_PHASE_2);  
#endif
			xTimerStart(xDataTxActivityTimer, DATA_ACTIVITY_BLOCK_TIME);
		}

		if (modem_rxdata_ind()) {
#if (BOARD != PIC32CXMTSH_DB) && (BOARD != PIC32CXMTC_DB)
			c42364a_show_icon(C42364A_ICON_AUDIO);
#else
			cl010_show_icon(CL010_ICON_PHASE_3);
#endif
			xTimerStart(xDataRxActivityTimer, DATA_ACTIVITY_BLOCK_TIME);
		}
#endif
#endif
	}
}

void vModemInitTask(void)
{
#ifndef _PRIME_SIM_
	/* Restart watchdog */
#if !PIC32CX
	wdt_restart(WDT);
#else
	dwdt_restart(DWDT, WDT0_ID);
#endif
#endif
	/* Init modem application */
	modem_init();

#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE
#if (BOARD != ATPL360AMB) && (BOARD != ATPL360MB) 
	xDataRxActivityTimer = xTimerCreate((const signed char *const)"Data Rx Activity T", DATA_ACTIVITY_TIMER, pdFALSE, NULL, _dataRx_activity);
	configASSERT(xDataRxActivityTimer);

	xDataTxActivityTimer = xTimerCreate((const signed char *const)"Data Tx Activity T", DATA_ACTIVITY_TIMER, pdFALSE, NULL, _dataTx_activity);
	configASSERT(xDataTxActivityTimer);
#endif
#endif

	/* Create new task to run modem application */
	xTaskCreate(task_modem, (const signed char *const)"Modem", TASK_MODEM_STACK, NULL, TASK_MODEM_PRIO, &xMODEMHnd);
}
