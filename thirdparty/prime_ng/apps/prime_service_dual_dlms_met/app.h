/**
 * \file
 *
 * \brief APP user App of common function
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

#ifndef APP_H
#define APP_H
/** @cond 0 */
/**INDENT-OFF**/
#include "asf.h"
#include "compiler.h"

#ifdef __cplusplus
extern "C"
{
#endif
/**INDENT-ON**/
/** @endcond */

/** APPDLMSEMU: ON/OFF JUMPER CONFIG */
#if (BOARD == ATPL230CH)
#define PIN_APPDLMSEMU_GPIO      (PIO_PA15_IDX)
#define PIN_APPDLMSEMU_FLAGS     (PIO_INPUT)
#define PIN_APPDLMSEMU           {PIO_PA15, PIOA, ID_PIOA, PIO_INPUT, PIO_DEBOUNCE | PIO_PULLUP}
#define PIN_APPDLMSEMU_MASK      PIO_PA15
#define PIN_APPDLMSEMU_PIO       PIOA
#define PIN_APPDLMSEMU_ID        ID_PIOA
#define PIN_APPDLMSEMU_TYPE      PIO_INPUT
#define PIN_APPDLMSEMU_ATTR      PIO_DEBOUNCE | PIO_PULLUP
#elif (BOARD == ATPL2xxAISN)
#define PIN_APPDLMSEMU_GPIO      (PIO_PB9_IDX)
#define PIN_APPDLMSEMU_FLAGS     (PIO_INPUT)
#define PIN_APPDLMSEMU           {PIO_PB9, PIOB, ID_PIOB, PIO_INPUT, PIO_DEBOUNCE | PIO_PULLUP}
#define PIN_APPDLMSEMU_MASK      PIO_PB9
#define PIN_APPDLMSEMU_PIO       PIOB
#define PIN_APPDLMSEMU_ID        ID_PIOB
#define PIN_APPDLMSEMU_TYPE      PIO_INPUT
#define PIN_APPDLMSEMU_ATTR      PIO_DEBOUNCE | PIO_PULLUP
#elif (BOARD == ATPL360AMB)
#define PIN_APPDLMSEMU_GPIO      (PIO_PA18_IDX)
#define PIN_APPDLMSEMU_FLAGS     (PIO_INPUT)
#define PIN_APPDLMSEMU           {PIO_PA18, PIOA, ID_PIOA, PIO_INPUT, PIO_DEBOUNCE}
#define PIN_APPDLMSEMU_MASK      PIO_PA18
#define PIN_APPDLMSEMU_PIO       PIOA
#define PIN_APPDLMSEMU_ID        ID_PIOA
#define PIN_APPDLMSEMU_TYPE      PIO_INPUT
#define PIN_APPDLMSEMU_ATTR      PIO_DEBOUNCE
#elif (BOARD == ATPL360MB)
#define PIN_APPDLMSEMU_GPIO      (PIO_PA18_IDX)
#define PIN_APPDLMSEMU_FLAGS     (PIO_INPUT)
#define PIN_APPDLMSEMU           {PIO_PA18, PIOA, ID_PIOA, PIO_INPUT, PIO_DEBOUNCE}
#define PIN_APPDLMSEMU_MASK      PIO_PA18
#define PIN_APPDLMSEMU_PIO       PIOA
#define PIN_APPDLMSEMU_ID        ID_PIOA
#define PIN_APPDLMSEMU_TYPE      PIO_INPUT
#define PIN_APPDLMSEMU_ATTR      PIO_DEBOUNCE
#else
#define PIN_APPDLMSEMU_GPIO      (PIO_PC3_IDX)
#define PIN_APPDLMSEMU_FLAGS     (PIO_INPUT)
#define PIN_APPDLMSEMU           {PIO_PC3, PIOC, ID_PIOC, PIO_INPUT, PIO_DEBOUNCE | PIO_PULLUP}
#define PIN_APPDLMSEMU_MASK      PIO_PC3
#define PIN_APPDLMSEMU_PIO       PIOC
#define PIN_APPDLMSEMU_ID        ID_PIOC
#define PIN_APPDLMSEMU_TYPE      PIO_INPUT
#define PIN_APPDLMSEMU_ATTR      PIO_DEBOUNCE | PIO_PULLUP
#endif

/* #define ENABLE_DIRECT_CON_EXAMPLE_APP */

enum {
	DLMS_APP_MODE,
	APP_EMU_MODE,
};

typedef enum {
	NODE_UNREGISTERED     = 0,
	NODE_REGISTERED       = 1,
	NODE_CONNECTED_APPEMU  = 2,
	NODE_CONNECTED_DLMSEMU = 3
} node_state_t;

/** \brief user app tasks */
/* @{ */
void vUserAppInitTask(uint8_t uc_prime_version);
void vUserAppResetTask(void);

/* @} */

/** \brief prime tasks */
/* @{ */
void vPrimeStackInitTask(void);

/* @} */

/** \brief get status of the  application mode */
/* @{ */
uint8_t get_app_mode(void);

/* @} */

/** @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/** @endcond */
#endif /* APP_H */
