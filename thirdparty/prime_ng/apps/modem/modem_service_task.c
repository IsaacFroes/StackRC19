/**
 * \file
 *
 * \brief PRIME_TASK : Task to run PRIME Stack
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

/** PRIME Task Rate */
#define MODEM_TASK_RATE          (5 / portTICK_RATE_MS)

/** PRIME Task Stack priority */
#define TASK_MODEM_PRIO          (tskIDLE_PRIORITY + 2)

/** PRIME Task Stack definition */
#define TASK_MODEM_STACK         (configMINIMAL_STACK_SIZE * 5)

/** Tasks handlers */
xTaskHandle xMODEMHnd;

#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE

/** Data Signaling Timers */
static xTimerHandle xDataRxActivityTimer = NULL;
static xTimerHandle xDataTxActivityTimer = NULL;

/**
 * \brief Function to show node status in the display
 *
 */
static void _show_node_state(void)
{
#if (BOARD == ATPL360AMB) || (BOARD == ATPL360MB)
	c0216CiZ_set_cursor(C0216CiZ_LINE_UP, 0);
#endif

	switch (modem_node_state()) {
	case MODEM_NODE_UNREGISTERED:
#if (BOARD == ATPL360AMB) || (BOARD == ATPL360MB)
		c0216CiZ_show((const char *)"SN UNREGISTERED");
#elif (BOARD == PIC32CXMTSH_DB) || (BOARD == PIC32CXMTC_DB)
		cl010_show_numeric_string(CL010_LINE_UP, (const uint8_t *)"55000000");
		cl010_show_icon(CL010_ICON_COL_2);
		cl010_clear_icon(CL010_ICON_COMM_SIGNAL_LOW);
		cl010_clear_icon(CL010_ICON_COMM_SIGNAL_MED);
		cl010_clear_icon(CL010_ICON_COMM_SIGNAL_HIG);
#else
		c42364a_show_text((const uint8_t *)"UNREG ");
#endif
		break;

	case MODEM_NODE_REGISTERED:
#if (BOARD == ATPL360AMB) || (BOARD == ATPL360MB)
		c0216CiZ_show((const char *)"SN REGISTERED   ");
#elif (BOARD == PIC32CXMTSH_DB) || (BOARD == PIC32CXMTC_DB)
		cl010_show_numeric_string(CL010_LINE_UP, (const uint8_t *)"55000001");
		cl010_show_icon(CL010_ICON_COL_2);
		cl010_show_icon(CL010_ICON_COMM_SIGNAL_LOW);
		cl010_show_icon(CL010_ICON_COMM_SIGNAL_MED);
		cl010_show_icon(CL010_ICON_COMM_SIGNAL_HIG);
#else
		c42364a_show_text((const uint8_t *)"SN REG");
#endif
		break;

	case MODEM_NODE_SWITCH:
#if (BOARD == ATPL360AMB) || (BOARD == ATPL360MB)
		c0216CiZ_show((const char *)"SN SWITCH       ");
#elif (BOARD == PIC32CXMTSH_DB) || (BOARD == PIC32CXMTC_DB)
		cl010_show_numeric_string(CL010_LINE_UP, (const uint8_t *)"55000005");
		cl010_show_icon(CL010_ICON_COL_2);
		cl010_show_icon(CL010_ICON_COMM_SIGNAL_LOW);
		cl010_show_icon(CL010_ICON_COMM_SIGNAL_MED);
		cl010_show_icon(CL010_ICON_COMM_SIGNAL_HIG);
#else
		c42364a_show_text((const uint8_t *)"SN SWI");
#endif
		break;

	default:
		break;
	}
}

/**
 * \brief Task to data reception activity signalling
 *
 * \param pxTimer    Timer
 */
static void _dataRx_activity(xTimerHandle pxTimer)
{
	UNUSED(pxTimer);
#if (BOARD == PIC32CXMTSH_DB) || (BOARD == PIC32CXMTC_DB)
	cl010_show_icon(CL010_ICON_PHASE_3);
#elif (BOARD != ATPL360AMB) && (BOARD != ATPL360MB)
	c42364a_clear_icon(C42364A_ICON_AUDIO);
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
#if (BOARD == PIC32CXMTSH_DB) || (BOARD == PIC32CXMTC_DB)
	cl010_show_icon(CL010_ICON_PHASE_2);
#elif (BOARD != ATPL360AMB) && (BOARD != ATPL360MB)
	c42364a_clear_icon(C42364A_ICON_WLESS);
#endif
}

#endif /* EXAMPLE_LCD_SIGNALLING_ENABLE */

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
		_show_node_state();
#endif
	}
}

/**
 * \brief Initialization of modem task
 *
 */
void vModemInitTask(void)
{
	/* Init modem application */
	modem_init();

#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE
	/* Create timer to control Rx data activity LCD symbol */
	xDataRxActivityTimer = xTimerCreate((const signed char *const)"Data Rx Activity T", DATA_ACTIVITY_TIMER, pdFALSE, NULL, _dataRx_activity);
	configASSERT(xDataRxActivityTimer);

	/* Create timer to control Tx data activity LCD symbol */
	xDataTxActivityTimer = xTimerCreate((const signed char *const)"Data Tx Activity T", DATA_ACTIVITY_TIMER, pdFALSE, NULL, _dataTx_activity);
	configASSERT(xDataTxActivityTimer);
#endif

	/* Create new task to run DLMS Emu application */
	xTaskCreate(task_modem, (const signed char *const)"Modem", TASK_MODEM_STACK, NULL, TASK_MODEM_PRIO, &xMODEMHnd);
}

/**
 * \brief Reset MODEM task
 *
 */
void vModemResetTask(void)
{
	/* Init MODEM emu application */
	modem_init();
}

/**
 * \brief Reset Aplication tasks
 *
 */

void vUserAppResetTask(void)
{
	/* reset application tasks */
	vModemResetTask();
}
