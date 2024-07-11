/**
 * \file
 *
 * \brief PRIME_TASK : Task to run PRIME Stack
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
#include "hal.h"
#include "hal_private.h"
#include "prime_api.h"
#include "conf_prime_stack.h"

/** PRIME Task Rate */
#define PRIME_TASK_RATE          (1 / portTICK_RATE_MS)

/** PRIME Task Stack priority */
#define TASK_PRIME_PRIO          (tskIDLE_PRIORITY + 2)

/** PRIME Task Stack definition */
#define TASK_PRIME_STACK         (configMINIMAL_STACK_SIZE * 5)

/** Tasks handlers */
xTaskHandle xPRIMEHnd;

/** Define HAL API interface */
extern const hal_api_t hal_api;

/**
 * \brief PRIME task
 *
 * \param pvParameters    Pointer to entry parameters
 */
static void task_prime(void *pvParameters)
{
	static portTickType xLastWakeTime;
	static portTickType xPeriod;

	(void)(pvParameters);

	xPeriod = PRIME_TASK_RATE;
	xLastWakeTime = xTaskGetTickCount();
	for (;;) {
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
		/* Process HAL layer */
		hal_process();
		/* Process PRIME stack */
		prime_process();
	}
}

/**
 * \brief Initiate PRIME stack task
 */
void vPrimeStackInitTask(void)
{
	/* Initialize hal layer */
	hal_init();

	/* Init PRIME stack */
	prime_init((hal_api_t *)&hal_api);

	/* Create new task to run PRIME application */
	xTaskCreate(task_prime, (const signed char *const)"PRIMETask", TASK_PRIME_STACK, NULL, TASK_PRIME_PRIO, &xPRIMEHnd);
}
