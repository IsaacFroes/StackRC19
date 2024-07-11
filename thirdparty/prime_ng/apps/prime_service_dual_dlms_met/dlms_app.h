/**
 * \file
 *
 * \brief DLMS_APP : DLMS Application for ATMEL PRIME v.1.4 Service Node
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

#ifndef DLMS_APP_H_INCLUDED
#define DLMS_APP_H_INCLUDED

#define DLMS_EMU_CONN_DEBUG_ENABLE
#define DLMS_EMU_DEBUG_ENABLE

#include "app.h"
#include "asf.h"
#include "compiler.h"

/** DLMS Emu Task Rate */
#define DLMS_APP_TASK_RATE          (100 / portTICK_RATE_MS)

/** DLMS Emu Task Stack priority */
#define TASK_DLMS_APP_PRIO          (tskIDLE_PRIORITY + 1)

/** DLMS Emu Task Stack definition */
#define TASK_DLMS_APP_STACK         (configMINIMAL_STACK_SIZE * 5)

/** \brief DLMS APP interface */
/* @{ */
void dlms_app_mlme_register_ind_cb(uint8_t *puc_sna, uint8_t uc_sid);
void dlms_app_mlme_unregister_ind_cb(void);
void dlms_app_cb_mlme_get_cfm(mlme_result_t x_status, uint16_t us_pib_attrib, void *pv_pib_value, uint8_t uc_pib_size);

void dlms_app_cl_432_establish_cfm_cb(uint8_t *puc_device_id, uint8_t uc_device_id_len, uint16_t us_dst_address, uint16_t us_base_address, uint8_t uc_ae);
void dlms_app_cl_432_release_cfm_cb(uint16_t us_dst_address, dl_432_result_t uc_result);
void dlms_app_cl_432_dl_data_ind_cb(uint8_t uc_dst_lsap, uint8_t uc_src_lsap, uint16_t us_dst_address, uint16_t us_src_address, uint8_t *puc_data,
		uint16_t us_lsdu_len, uint8_t uc_link_class);
void dlms_app_cl_432_dl_data_cfm_cb(uint8_t uc_dst_lsap, uint8_t uc_src_lsap, uint16_t us_dst_address, dl_432_tx_status_t uc_tx_status);

void dlms_app_init(uint8_t uc_prime_version);
void dlms_app_process(void);
node_state_t dlms_app_node_state(void);

/* @} */
#endif /* DLMS_APP_H_INCLUDED */
