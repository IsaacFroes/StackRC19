/**
 * \file
 *
 * \brief DLMS_APP : DLMS Application for ATMEL PRIME NG v.1.4 Service Node
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

#include "dlms_app.h"

#include <string.h>
#include <stdio.h>

#include "app.h"
#include "app_event_manager.h"
#include "asf.h"
#include "dlms_srv_lib.h"
#include "dlms_srv_data.h"
#include "compiler.h"

static node_state_t uc_node_state;
static uint8_t suc_ae;
static uint8_t suc_prime_version;

/** Meter params */
static meter_params_t sx_meter_params;

/* Tx buffer */
dl_432_buffer_t x_432_buff;

/** Connection status: one 4-32 connection per node */
struct {
	bool b_is_open;
	uint8_t puc_device_id[32];
	uint8_t uc_device_id_len;
	uint16_t us_base_addr;
	uint16_t us_node_addr;
} x_432_con_info;

/** Association configuration: in this example, four possible associations: MGMT(1), READ(2), FW(3) and PUBLIC(16) */
static const assoc_conf_t px_assoc_conf[] = {
	{0x01, 0x01, LLS_ALG_1_PWD, "00000002", COSEM_LOW_LEVEL_SEC},    /* MGMT */
	{0x02, 0x01, LLS_ALG_1_PWD, "00000001", COSEM_LOW_LEVEL_SEC},    /* READ */
	{0x03, 0x01, LLS_FIXED_PWD, "00000003", COSEM_LOW_LEVEL_SEC},    /* FW */
	{0x10, 0x01, LLS_FIXED_PWD, "--------", COSEM_LOWEST_LEVEL_SEC}  /* PUBLIC */
};

/**
 * \brief Convert functions
 *
 * \param uc_num     Hex number
 *
 * \retval string number
 */
static uint8_t _convertHEX2STR(uint8_t uc_num)
{
	if (uc_num > 9) {
		return (uc_num + 0x37);
	} else {
		return (uc_num + 0x30);
	}
}

/**
 * \brief Generate serial
 *
 * \param puc_serial_board    Pointer to serial board
 * \param puc_eui48           EUI-48 address
 */
static void _generate_serial(uint8_t *puc_serial_board, uint8_t *puc_eui48)
{
	uint8_t uc_index;
	uint8_t uc_pos;
	uint8_t uc_num, uc_num1;
	memcpy(puc_serial_board, "ATM", 3);
	/* convert hex to ascii */
	uc_index = 1;
	uc_pos = 3;
	while (uc_pos < 13) {
		uc_num = ((*(puc_eui48 + uc_index) & 0xf0) >> 4);
		uc_num1 = (*(puc_eui48 + uc_index) & 0x0f);
		uc_index++;
		*(puc_serial_board + uc_pos++) = _convertHEX2STR(uc_num);
		*(puc_serial_board + uc_pos++) = _convertHEX2STR(uc_num1);
	}
	*(puc_serial_board + uc_pos) = 0;
}

/**
 * \brief MLME register indication
 *
 * \param puc_sna      Pointer to SNA
 * \param uc_sid       Switch Identifier
 *
 */
void dlms_app_mlme_register_ind_cb(uint8_t *puc_sna, uint8_t uc_sid)
{
	(void)puc_sna;
	(void)uc_sid;

#ifdef DLMS_EMU_CONN_DEBUG_ENABLE
	uint8_t uc_idx;
	printf("DLMS APP Application: _mlme_register_ind_cb...sn: ");
	for (uc_idx = 0; uc_idx < strlen((const char *)sx_meter_params.meter_serial); uc_idx++) {
		printf("%c", sx_meter_params.meter_serial[uc_idx]);
	}
	printf("\r\n");
#endif
	/* Change node state */
	uc_node_state = NODE_REGISTERED;

	if (suc_prime_version == PRIME_1_4) {
		/* Get current security profile */
		prime_cl_null_mlme_get_request(PIB_MAC_SEC_PROFILE_USED);
	} else {
		suc_ae = 0;
	}

	/* Launch 432 connection */
	prime_cl_432_establish_request((uint8_t *)sx_meter_params.meter_serial, strlen((const char *)sx_meter_params.meter_serial), suc_ae);
}

/**
 * \brief MLME unregister indication
 *
 */
void dlms_app_mlme_unregister_ind_cb(void)
{
	/* Reset DLMS connection params */
	x_432_con_info.b_is_open = false;
	x_432_con_info.us_base_addr = CL_432_INVALID_ADDRESS;
	x_432_con_info.us_node_addr = CL_432_INVALID_ADDRESS;
	memset(&x_432_con_info.puc_device_id, 0, sizeof(x_432_con_info.puc_device_id));
	x_432_con_info.uc_device_id_len = 0;

	dlms_srv_432_conn_close();

	/* Change node state */
	uc_node_state = NODE_UNREGISTERED;

#ifdef DLMS_EMU_CONN_DEBUG_ENABLE
	printf("DLMS APP Application: _mlme_unregister_ind_cb...\r\n");
#endif
}

/**
 * \brief MLME get pib confirmm callback
 *
 * \param x_status           Status
 * \param us_pib_attrib      PIB attribute
 * \param pv_pib_value       PIB attribute value
 * \param uc_pib_size        PIB attribute value size
 */
void dlms_app_cb_mlme_get_cfm(mlme_result_t x_status, uint16_t us_pib_attrib, void *pv_pib_value, uint8_t uc_pib_size)
{
	uint16_t ui_temp;
	uint8_t uc_temp;

	/* Check result */
	if (x_status != MLME_RESULT_DONE) {
		return;
	}

	switch (us_pib_attrib) {
	case PIB_MAC_APP_FW_VERSION:
		memcpy(sx_meter_params.pib_fw_version, ((uint32_t *)pv_pib_value), uc_pib_size);
		break;

	case PIB_MAC_APP_VENDOR_ID:
		ui_temp = *((uint16_t *)pv_pib_value);
		sx_meter_params.pib_vendor_id[0] = (uint8_t)(ui_temp >> 8); /* high byte */
		sx_meter_params.pib_vendor_id[1] = (uint8_t)(ui_temp & 0x00FF); /* low byte */
		break;

	case PIB_MAC_APP_PRODUCT_ID:
		ui_temp = *((uint16_t *)pv_pib_value);
		sx_meter_params.pib_product_id[0] = (uint8_t)(ui_temp >> 8); /* high byte */
		sx_meter_params.pib_product_id[1] = (uint8_t)(ui_temp & 0x00FF); /* low byte */
		break;

	case PIB_MTP_MAC_EUI_48:
	case PIB_MAC_EUI_48:
		_generate_serial(sx_meter_params.meter_serial, pv_pib_value);
		break;

	case PIB_MAC_SEC_PROFILE_USED:
		uc_temp =  *((uint8_t *)pv_pib_value);
		/* Set encryption depending on security profile */
		suc_ae = (uc_temp == 0 ? 0 : 1);

	default:
		break;
	}
}

/**
 * \brief CL 432 Establish confirm callback
 *
 * \param puc_device_id          Pointer to the device identifier data
 * \param uc_device_id_len       Length of the device identfier
 * \param us_dst_address         Destination 432 Address
 * \param us_base_address        Base 432 Address
 * \param uc_ae                  Flag to indicate that authentication and encryption is requested
 */
void dlms_app_cl_432_establish_cfm_cb(uint8_t *puc_device_id, uint8_t uc_device_id_len, uint16_t us_dst_address, uint16_t us_base_address, uint8_t uc_ae)
{
	(void)uc_ae;

	/* Change node state */
	uc_node_state = NODE_CONNECTED_DLMSEMU;

	x_432_con_info.b_is_open = true;
	x_432_con_info.us_base_addr = us_base_address;
	x_432_con_info.us_node_addr = us_dst_address;

	if (uc_device_id_len < sizeof(x_432_con_info.puc_device_id)) {
		x_432_con_info.uc_device_id_len = uc_device_id_len;
	} else {
		x_432_con_info.uc_device_id_len = sizeof(x_432_con_info.puc_device_id);
	}

	memcpy(&x_432_con_info.puc_device_id, puc_device_id, x_432_con_info.uc_device_id_len);

#ifdef DLMS_EMU_CONN_DEBUG_ENABLE
	printf("DLMS APP Application: _cl_432_establish_cfm_cb...\r\n");
#endif
}

/**
 * \brief CL 432 Release confirm callback
 *
 * \param us_dst_address   Destination 432 Address
 * \param uc_result        Confirmation result
 */
void dlms_app_cl_432_release_cfm_cb(uint16_t us_dst_address, dl_432_result_t uc_result)
{
	(void)us_dst_address;
	(void)uc_result;

	/* Change node state */
	uc_node_state = NODE_REGISTERED;

#ifdef DLMS_EMU_CONN_DEBUG_ENABLE
	printf("DLMS APP Application: _cl_432_release_cfm_cb...sn: ");
	uint16_t uc_idx;
        for (uc_idx = 0; uc_idx < strlen((const char *)sx_meter_params.meter_serial); uc_idx++) {
		printf("%c", sx_meter_params.meter_serial[uc_idx]);
	}
	printf("\r\n");
#endif

	x_432_con_info.b_is_open = false;
	x_432_con_info.us_base_addr = CL_432_INVALID_ADDRESS;
	x_432_con_info.us_node_addr = CL_432_INVALID_ADDRESS;
	memset(&x_432_con_info.puc_device_id, 0, sizeof(x_432_con_info.puc_device_id));
	x_432_con_info.uc_device_id_len = 0;

	dlms_srv_432_conn_close();

	/* Re-launch 432 connection */
	prime_cl_432_establish_request((uint8_t *)sx_meter_params.meter_serial, strlen((const char *)sx_meter_params.meter_serial), suc_ae);

#ifdef DLMS_EMU_CONN_DEBUG_ENABLE
	printf("DLMS APP Application: _cl_432_release_cfm_cb...\r\n");
#endif
}

/**
 * \brief CL 432 Data indication callback
 *
 * \param uc_dst_lsap      Destination LSAP
 * \param uc_src_lsap      Source LSAP
 * \param us_dst_address   Destination 432 Address
 * \param us_src_address   Source Address
 * \param puc_data         Pointer to received data
 * \param us_lsdu_len      Length of the data
 * \param uc_link_class    Link class (non used)
 */
void dlms_app_cl_432_dl_data_ind_cb(uint8_t uc_dst_lsap, uint8_t uc_src_lsap, uint16_t us_dst_address, uint16_t us_src_address,
		uint8_t *puc_data, uint16_t us_lsdu_len, uint8_t uc_link_class)
{
	(void)uc_link_class;
	(void)us_dst_address;
	(void)us_src_address;

	dlms_srv_data_ind(uc_dst_lsap, uc_src_lsap, puc_data, us_lsdu_len);

#ifdef DLMS_EMU_CONN_DEBUG_ENABLE
	printf("DLMS APP Application: _cl_432_dl_data_ind_cb...(0x%02x)\r\n", us_lsdu_len);
#endif
}

/**
 * \brief CL 432 Data confirmation callback
 *
 * \param uc_dst_lsap      Destination LSAP
 * \param uc_src_lsap      Source LSAP
 * \param us_dst_address   Destination 432 Address
 * \param uc_tx_status     Transmission status
 */
void dlms_app_cl_432_dl_data_cfm_cb(uint8_t uc_dst_lsap, uint8_t uc_src_lsap, uint16_t us_dst_address, dl_432_tx_status_t uc_tx_status)
{
	(void)us_dst_address;

	dlms_srv_data_cfm((uint16_t)uc_dst_lsap, (uint16_t)uc_src_lsap, (uc_tx_status == CL_432_TX_STATUS_SUCCESS) ? true : false);

#ifdef DLMS_EMU_CONN_DEBUG_ENABLE
	printf("DLMS APP Application: _cl_432_dl_data_cfm_cb...(0x%02x)\r\n", uc_tx_status);
#endif
}

/**
 * \brief Sending data from DLMS Server lib to 4-32 connection
 *
 * \param uc_dst_lsap      Destination LSAP
 * \param uc_src_lsap      Source LSAP
 * \param px_buff          Pointer to the data buffer
 * \param us_lsdu_len      Length of the data
 */
static void _dlms_app_data_request(uint16_t us_dst_lsap, uint16_t us_src_lsap, uint8_t *puc_buff, uint16_t us_lsdu_len)
{
	if (us_lsdu_len < MAX_LENGTH_432_DATA) {
		memcpy(x_432_buff.dl.buff, puc_buff, us_lsdu_len);
		prime_cl_432_dl_data_request((uint8_t)us_dst_lsap, (uint8_t)us_src_lsap, x_432_con_info.us_base_addr, &x_432_buff, us_lsdu_len, 0);
	}
}

/**
 * \brief Initialize DLME EMU Application
 *
 */
void dlms_app_init(uint8_t uc_prime_version)
{
	suc_prime_version = uc_prime_version;

	/*                                 OBIS CODE + IC        READ:  MGMT   READ        FW          PUBLIC       WRITE    */
	obis_element_conf_t x_new_obis = {{0, 0, 0, 0, 0, 0}, 0, {0xFF800000, 0xFF800000, 0xFF800000, 0xFF800000}, {0, 0, 0, 0}, NULL};

	/* Configuration of PRIME callback functions */
	if (get_app_mode() == DLMS_APP_MODE) {
		app_event_dispatcher_init(DLMS_APP_MODE);
	}

	/* Reset DLMS connection params */
	x_432_con_info.b_is_open = false;
	x_432_con_info.us_base_addr = CL_432_INVALID_ADDRESS;
	x_432_con_info.us_node_addr = CL_432_INVALID_ADDRESS;
	memset(&x_432_con_info.puc_device_id, 0, sizeof(x_432_con_info.puc_device_id));
	x_432_con_info.uc_device_id_len = 0;

	/* init values needed in dlms emu application */
	if (suc_prime_version == PRIME_1_3) {
		prime_cl_null_mlme_get_request(PIB_MTP_MAC_EUI_48);
	} else {
		prime_cl_null_mlme_get_request(PIB_MAC_EUI_48);
	}

	prime_cl_null_mlme_get_request(PIB_MAC_APP_FW_VERSION);
	prime_cl_null_mlme_get_request(PIB_MAC_APP_VENDOR_ID);
	prime_cl_null_mlme_get_request(PIB_MAC_APP_PRODUCT_ID);

	/* DLMS Server lib init */
	dlms_srv_init(px_assoc_conf, DLMS_MAX_ASSOC, &sx_meter_params, _dlms_app_data_request);

	/* DLMS Server objects configuration */
	dlms_srv_conf_obis(1, 0, 0, 2, 0, 255, 1, obis_1_0_0_2_0_255_cb, &x_new_obis);
	dlms_srv_conf_obis(0, 0, 96, 1, 0, 255, 1, obis_0_0_96_1_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(0, 0, 96, 1, 1, 255, 1, obis_0_0_96_1_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(0, 0, 96, 1, 2, 255, 1, obis_0_0_96_1_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 0, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 10, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 11, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 12, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 13, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 14, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 15, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 16, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 20, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 21, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 22, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 23, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 24, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 25, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 26, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 30, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 31, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 32, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 33, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 34, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 35, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 36, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 1, 8, 255, 255, 3, obis_1_0_1_8_x_255_cb, &x_new_obis);
	dlms_srv_conf_obis(1, 0, 99, 1, 0, 255, 7, obis_1_0_99_1_0_255_cb, &x_new_obis);
	dlms_srv_conf_obis(0, 0, 21, 0, 5, 255, 7, obis_0_0_21_0_5_255_cb, &x_new_obis);
	dlms_srv_conf_obis(0, 0, 21, 0, 6, 255, 7, obis_0_0_21_0_6_255_cb, &x_new_obis);
	dlms_srv_conf_obis(0, 0, 1, 0, 0, 255, 8, obis_0_0_1_0_0_255_cb, &x_new_obis);
	dlms_srv_conf_obis(0, 0, 28, 7, 0, 255, 86, obis_0_0_28_7_0_255_cb, &x_new_obis);

	/* Current association object list */
	dlms_srv_conf_obis(0, 0, 40, 0, 0, 255, 15, obis_0_0_40_0_0_255_cb, &x_new_obis);

	/* Reset node state */
	uc_node_state = NODE_UNREGISTERED;

#ifdef DLMS_EMU_DEBUG_ENABLE
	printf("DLMS APP Application: Service Node...\r\n");
#endif
}

/**
 * \brief Process DLME EMU Application
 *
 */
void dlms_app_process(void)
{
	dlms_srv_process();
	return;
}

/**
 * \brief Return node state
 *
 * \retval Node state
 */
node_state_t dlms_app_node_state(void)
{
	return uc_node_state;
}
