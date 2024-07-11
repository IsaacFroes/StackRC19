/**
 * \file
 *
 * \brief PRIME APP_EMU App with echo of appemu
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

#ifndef ECHO_APPEMU_H
#define ECHO_APPEMU_H

#include "app.h"
#include "asf.h"
#include "compiler.h"

/** @cond 0 */
/**INDENT-OFF**/

#ifdef __cplusplus
extern "C"
{
#endif
/**INDENT-ON**/
/** @endcond */
/* debug mode printf app */
/* #define APP_EMU_DEBUG_ENABLE */

/** APP Emu Task Rate */
#define APP_EMU_TASK_RATE          (100 / portTICK_RATE_MS)

/** APP Emu Task Stack priority */
#define TASK_APP_EMU_PRIO          (tskIDLE_PRIORITY + 2)

/** APP Emu Task Stack definition */
#define TASK_APP_EMU_STACK         (configMINIMAL_STACK_SIZE * 5)

/** \brief App EMU interface */
/* @{ */
void app_emu_mlme_register_ind_cb(uint8_t *puc_sna, uint8_t uc_sid);
void app_emu_mlme_unregister_cb(void);
void app_emu_mlme_get_confirm_cb(mlme_result_t x_status, uint16_t us_pib_attrib, void *ul_pib_value, uint8_t len);
void app_emu_establish_confirm_cb(uint16_t us_con_handle, mac_establish_confirm_result_t uc_result, uint8_t *puc_eui48, uint8_t uc_type, uint8_t *puc_data,
		uint16_t us_data_len, uint8_t uc_ae);
void app_emu_release_ind_cb(uint16_t us_con_handle, mac_release_indication_reason_t uc_reason);
void app_emu_data_indication_cb(uint16_t us_con_handle, uint8_t *puc_data, uint16_t us_data_len, uint32_t ul_time_ref);
node_state_t app_emu_node_state(void);
void app_emu_init(void);
void app_emu_process(void);

/* @} */

/** @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/** @endcond */
#endif /* ECHO_APPEMU_H */
