/**
 * \file
 *
 * \brief APP_EVENT_MANAGER : prime event dispatcher to the application APP_EMU and DLMS_APP
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
#include <stdint.h>

#include "app.h"
#include "app_emu.h"
#include "dlms_app.h"
#include "mac_defs.h"
#include "prime_api.h"

#ifdef ENABLE_DIRECT_CON_EXAMPLE_APP
#include "app_dc.h"
#endif

static uint8_t suc_mode;

void app_event_dispatcher_init(uint8_t mode);

/**
 * \brief MLME Get Confirm
 *
 * \param x_status           Status
 * \param us_pib_attrib      PIB attribute
 * \param pv_pib_value       PIB attribute value
 * \param uc_pib_size        PIB attribute value size
 */
static void _cb_mlme_get_cfm(mlme_result_t x_status, uint16_t us_pib_attrib, void *pv_pib_value, uint8_t uc_pib_size)
{
	if (suc_mode == APP_EMU_MODE) {
		dlms_app_cb_mlme_get_cfm(x_status, us_pib_attrib, pv_pib_value, uc_pib_size);
		app_emu_mlme_get_confirm_cb(x_status, us_pib_attrib, pv_pib_value, uc_pib_size);
	}

	if (suc_mode == DLMS_APP_MODE) {
		dlms_app_cb_mlme_get_cfm(x_status, us_pib_attrib, pv_pib_value, uc_pib_size);
	}

#ifdef ENABLE_DIRECT_CON_EXAMPLE_APP
	app_dc_cb_mlme_get_cfm(x_status, us_pib_attrib, pv_pib_value, uc_pib_size);
#endif
}

/**
 * \brief MLME Register Indication
 *
 * \param puc_sna      Pointer to SNA
 * \param uc_sid       Switch Identifier
 */
static void _mlme_register_ind_cb(uint8_t *puc_sna, uint8_t uc_sid)
{
	if (suc_mode == APP_EMU_MODE) {
		dlms_app_mlme_register_ind_cb(puc_sna, uc_sid);
		app_emu_mlme_register_ind_cb(puc_sna, uc_sid);
	}

	if (suc_mode == DLMS_APP_MODE) {
		dlms_app_mlme_register_ind_cb(puc_sna, uc_sid);
	}

#ifdef ENABLE_DIRECT_CON_EXAMPLE_APP
	app_dc_mlme_register_ind_cb(puc_sna, uc_sid);
#endif
}

/**
 * \brief MLME Unregister Request
 */
static void _mlme_unregister_ind_cb(void)
{
	if (suc_mode == APP_EMU_MODE) {
		dlms_app_mlme_unregister_ind_cb();
		app_emu_mlme_unregister_cb();
	}

	if (suc_mode == DLMS_APP_MODE) {
		dlms_app_mlme_unregister_ind_cb();
	}

#ifdef ENABLE_DIRECT_CON_EXAMPLE_APP
	app_dc_node_unregistered_cb();
#endif
}

/**
 * \brief MLME Unregister Request
 */
static void _data_ind_cb(uint16_t us_con_handle, uint8_t *puc_data, uint16_t us_data_len, uint32_t ul_time_ref)
{
#ifdef ENABLE_DIRECT_CON_EXAMPLE_APP
	if (us_con_handle & SN_DIRECT_CON_HANDLER_MSK) {
		app_dc_data_ind_cb(us_con_handle, puc_data, us_data_len, ul_time_ref);
	} else {
		app_emu_data_indication_cb(us_con_handle, puc_data, us_data_len, ul_time_ref);
	}

#else
	app_emu_data_indication_cb(us_con_handle, puc_data, us_data_len, ul_time_ref);
#endif
}

/**
 * \brief MLME Unregister Request
 */
static void _establish_cfm_cb(uint16_t us_con_handle, mac_establish_confirm_result_t uc_result,
		uint8_t *puc_eui48, uint8_t uc_type, uint8_t *puc_data, uint16_t us_data_len, uint8_t uc_ae)
{
	app_emu_establish_confirm_cb(us_con_handle, uc_result, puc_eui48, uc_type, puc_data, us_data_len, uc_ae);
#ifdef ENABLE_DIRECT_CON_EXAMPLE_APP
	app_dc_establish_cfm_cb(us_con_handle, uc_result, puc_eui48, uc_type, puc_data, us_data_len, uc_ae);
#endif
}

/**
 * \brief MLME Unregister Request
 */
static void _release_ind_cb(uint16_t us_con_handle, mac_release_indication_reason_t uc_reason)
{
	app_emu_release_ind_cb(us_con_handle, uc_reason);
#ifdef ENABLE_DIRECT_CON_EXAMPLE_APP
	app_dc_release_ind_cb(us_con_handle, uc_reason);
#endif
}

/**
 * \brief Initialize DLME EMU Application
 *
 * \param mode       Application mode
 */
void app_event_dispatcher_init(uint8_t mode)
{
	mac_callbacks_t mac_cbs;
	cl_432_callbacks_t cl432_cbs;

	suc_mode = mode;

	memset(&mac_cbs, 0, sizeof(mac_cbs));
	memset(&cl432_cbs, 0, sizeof(cl432_cbs));

	/* Configuration of PRIME callback functions for both applications*/
	mac_cbs.mlme_get_cfm_cb = _cb_mlme_get_cfm;
	mac_cbs.mlme_register_ind_cb = _mlme_register_ind_cb;
	mac_cbs.mlme_unregister_ind_cb = _mlme_unregister_ind_cb;
#ifdef ENABLE_DIRECT_CON_EXAMPLE_APP
	mac_cbs.mac_data_cfm_cb = app_dc_data_cfm_cb;
	mac_cbs.mac_establish_ind_cb = app_dc_establish_ind_cb;
#endif

	if (mode == APP_EMU_MODE) { /* APP_DLMS+APP_EMU */
		/* Configuration of PRIME callback functions for APP_EMU application */
		mac_cbs.mac_data_ind_cb = _data_ind_cb;
		mac_cbs.mac_establish_cfm_cb = _establish_cfm_cb;
		mac_cbs.mac_release_ind_cb = _release_ind_cb;
	}

	if (mode == DLMS_APP_MODE) { /* only APP_DLMS */
		/* Configuration of PRIME callback functions for DLMS_APP application */
#ifdef ENABLE_DIRECT_CON_EXAMPLE_APP
		mac_cbs.mac_data_ind_cb = app_dc_data_ind_cb;
		mac_cbs.mac_establish_cfm_cb = app_dc_establish_cfm_cb;
		mac_cbs.mac_release_ind_cb = app_dc_release_ind_cb;
#endif
	}

	/* Configuration of PRIME callback functions for DLMS_APP and app_emu application */
	cl432_cbs.cl_432_establish_cfm_cb = dlms_app_cl_432_establish_cfm_cb;
	cl432_cbs.cl_432_release_cfm_cb = dlms_app_cl_432_release_cfm_cb;
	cl432_cbs.cl_432_dl_data_ind_cb = dlms_app_cl_432_dl_data_ind_cb;
	cl432_cbs.cl_432_dl_data_cfm_cb = dlms_app_cl_432_dl_data_cfm_cb;

	prime_cl_null_set_callbacks(&mac_cbs);
	prime_cl_432_set_callbacks(&cl432_cbs);
}
