/**
 * \file
 *
 * \brief DLMS_EMU : APP DClator for ATMEL PRIME NG v.1.3 Service Node
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

#include "app_dc.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "app.h"
#include "asf.h"
#include "compiler.h"
#include "prime_api.h"
#include "mac_pib.h"

/* #define USING_SNA */

#define NUM_OPEN_CONNECTIONS    (1)

#define TYPE_DC                 (8)

#define TIME_SEND_DATA          (120000)

#define TIME_RETRY_CONNECTION   (40000)

#define LEN_ID_EUI48            (6)

#define MAX_TABLE_DCON          (1000)

typedef enum {
	ST_IDLE,
	ST_DECIDE_MASTER_OR_SLAVE,
	ST_MASTER,
	ST_SLAVE
} st_app_dc_t;

static st_app_dc_t st_app_dc;

static uint8_t suc_sna[6];

static uint8_t suc_eui48[6];

static int32_t sul_time;

static uint8_t suc_master_connecting_number;

static uint8_t suc_master_sending_data_number;

static uint8_t suc_slave_response_connection;

static uint8_t suc_slave_answer_data;

static uint8_t suc_ae;

static struct {
	int32_t time;
	uint16_t handler;
	uint8_t eui48_dst[6];
} sx_macs_and_handlers[NUM_OPEN_CONNECTIONS];

static const uint8_t null_eui48[LEN_ID_EUI48] = {0, 0, 0, 0, 0, 0};

/* static connection routing table MUST BE THE SAME IN BN dc_base.c */
static const uint8_t x_direct_con_connector[MAX_TABLE_DCON][2][LEN_ID_EUI48] = {
	/*           EUI48 MASTER                      EUI48 SLAVE          */
	{{0x01, 0x40, 0x51, 0x21, 0x00, 0x03}, {0xfc, 0xc2, 0x3d, 0x01, 0x8a, 0x74}},
	{{0xfc, 0xc2, 0x3d, 0x01, 0x8a, 0x78}, {0xfc, 0xc2, 0x3d, 0x01, 0x8a, 0x75}},
};

static st_app_dc_t _search_connection_peer(uint8_t *p_origin, uint8_t *p_destiny)
{
	uint16_t us_index = 0;

	while (us_index < MAX_TABLE_DCON) {
		/* Check if there are more MACs */
		if (!memcmp(x_direct_con_connector[us_index][0], null_eui48, LEN_ID_EUI48)) {
			return ST_IDLE;
		}

		/* Check if eui48 SRC is p_origin */
		if (!memcmp(p_origin, x_direct_con_connector[us_index][0], LEN_ID_EUI48)) {
			memcpy(p_destiny, x_direct_con_connector[us_index][1], LEN_ID_EUI48);
			return ST_MASTER;
		}

		/* Check if eui48 DST is p_origin */
		if (!memcmp(p_origin, x_direct_con_connector[us_index][1], LEN_ID_EUI48)) {
			memcpy(p_destiny, x_direct_con_connector[us_index][0], LEN_ID_EUI48);
			return ST_SLAVE;
		}

		us_index++;
	}

	/* EUI48 NOT FOUND */
	return ST_IDLE;
}

/**
 * \brief Initialize APP DC Application
 *
 */
void app_dc_mlme_register_ind_cb(uint8_t *puc_sna, uint8_t uc_sid)
{
	(void)(uc_sid);
	memcpy(suc_sna, puc_sna, 6);
	prime_cl_null_mlme_get_request(PIB_MAC_EUI_48);
	prime_cl_null_mlme_get_request(PIB_MAC_SEC_PROFILE_USED);
}

void app_dc_node_unregistered_cb(void)
{
	st_app_dc = ST_IDLE;
}

static void _decide_master_or_slave(void)
{
	uint8_t uc_i;
	uint8_t eui48_dst[LEN_ID_EUI48];

	st_app_dc = _search_connection_peer(suc_eui48, eui48_dst);

	if (st_app_dc == ST_MASTER) {
		for (uc_i = 0; uc_i < NUM_OPEN_CONNECTIONS; uc_i++) {
#ifdef USING_SNA
			memcpy(sx_macs_and_handlers[uc_i].eui48_dst, suc_sna, 6);
#else
			memcpy(sx_macs_and_handlers[uc_i].eui48_dst, eui48_dst, 6);
#endif
			sx_macs_and_handlers[uc_i].handler = MAC_INVALID_HANDLER;
			sx_macs_and_handlers[uc_i].time = sul_time;
		}
	}

	if (st_app_dc == ST_SLAVE) {
		for (uc_i = 0; uc_i < NUM_OPEN_CONNECTIONS; uc_i++) {
			sx_macs_and_handlers[uc_i].handler = MAC_INVALID_HANDLER;
		}
	}
}

void app_dc_cb_mlme_get_cfm(mlme_result_t x_status, uint16_t us_pib_attrib, void *pv_pib_value, uint8_t uc_pib_size)
{
	(void)x_status;
	uint8_t uc_temp;

	switch (us_pib_attrib) {
	case PIB_MAC_EUI_48:
		memcpy(suc_eui48, pv_pib_value, uc_pib_size);
		st_app_dc = ST_DECIDE_MASTER_OR_SLAVE;
		break;

	case PIB_MAC_SEC_PROFILE_USED:
		uc_temp =  *((uint8_t *)pv_pib_value);
		/* Set encryption depending on security profile */
		suc_ae = (uc_temp == 0 ? 0 : 1);

	default:
		break;
	}
}

void app_dc_establish_ind_cb(uint16_t us_con_handle, uint8_t *puc_eui48, uint8_t uc_type, uint8_t *puc_data,
		uint16_t us_data_len, uint8_t uc_cfbytes, uint8_t uc_ae)
{
	(void)puc_eui48;
	(void)uc_type;
	(void)puc_data;
	(void)us_data_len;
	(void)uc_cfbytes;
	(void)uc_ae;

	if (st_app_dc == ST_SLAVE) {
		uint8_t uc_i;

		for (uc_i = 0; uc_i < NUM_OPEN_CONNECTIONS; uc_i++) {
			if ((sx_macs_and_handlers[uc_i].handler == MAC_INVALID_HANDLER) ||
					(sx_macs_and_handlers[uc_i].handler == us_con_handle)) {
				sx_macs_and_handlers[uc_i].handler = us_con_handle;
				suc_slave_response_connection = uc_i;
				break;
			}
		}
	}
}

void app_dc_establish_cfm_cb(uint16_t us_con_handle, mac_establish_confirm_result_t uc_result,
		uint8_t *puc_eui48, uint8_t uc_type, uint8_t *puc_data, uint16_t us_data_len, uint8_t uc_ae)
{
	(void)(uc_type);
	(void)(puc_data);
	(void)(us_data_len);
	(void)(uc_ae);

	if (uc_result == MAC_ESTABLISH_CONFIRM_RESULT_SUCCESS) {
		sx_macs_and_handlers[suc_master_connecting_number].handler = us_con_handle;
		memcpy(sx_macs_and_handlers[suc_master_connecting_number].eui48_dst, puc_eui48, 6);
		sx_macs_and_handlers[suc_master_connecting_number].time = sul_time + TIME_SEND_DATA;
	} else {
		sx_macs_and_handlers[suc_master_connecting_number].time = sul_time + TIME_RETRY_CONNECTION;
	}

	suc_master_connecting_number = NUM_OPEN_CONNECTIONS;
}

void app_dc_release_ind_cb(uint16_t us_con_handle, mac_release_indication_reason_t uc_reason)
{
	uint8_t uc_i;

	(void)(uc_reason);

	if (st_app_dc == ST_MASTER) {
		for (uc_i = 0; uc_i < NUM_OPEN_CONNECTIONS; uc_i++) {
			if (sx_macs_and_handlers[uc_i].handler == us_con_handle) {
				sx_macs_and_handlers[uc_i].time = sul_time + TIME_RETRY_CONNECTION;
				sx_macs_and_handlers[uc_i].handler = MAC_INVALID_HANDLER;
				break;
			}
		}
	}

	if (st_app_dc == ST_SLAVE) {
		for (uc_i = 0; uc_i < NUM_OPEN_CONNECTIONS; uc_i++) {
			if (sx_macs_and_handlers[uc_i].handler == us_con_handle) {
				sx_macs_and_handlers[uc_i].handler = MAC_INVALID_HANDLER;
				break;
			}
		}
	}

	prime_cl_null_release_response(us_con_handle, MAC_RELEASE_RESPONSE_ACCEPT);
}

void app_dc_data_cfm_cb(uint16_t us_con_handle, uint8_t *puc_data, mac_data_result_t drt_result)
{
	(void)(puc_data);
	(void)(us_con_handle);

	if (st_app_dc == ST_MASTER) {
		/* Wait for indication or next send data in this connection */
		sx_macs_and_handlers[suc_master_sending_data_number].time = sul_time + TIME_SEND_DATA;

		/* Bad result, continue with next connection */
		if (drt_result != MAC_DATA_SUCCESS) {
			suc_master_sending_data_number = NUM_OPEN_CONNECTIONS;
		}
	}

	/* No actions to be taken in ST_SLAVE */
}

void app_dc_data_ind_cb(uint16_t us_con_handle, uint8_t *puc_data, uint16_t us_data_len, uint32_t ul_time_ref)
{
	(void)(puc_data);
	(void)(us_data_len);
	(void)(ul_time_ref);

	if (st_app_dc == ST_MASTER) {
		/* Wait for next send data in this connection */
		sx_macs_and_handlers[suc_master_sending_data_number].time = sul_time + TIME_SEND_DATA;

		/* Continue with next connection */
		suc_master_sending_data_number = NUM_OPEN_CONNECTIONS;
	}

	if (st_app_dc == ST_SLAVE) {
		uint8_t uc_i;

		for (uc_i = 0; uc_i < NUM_OPEN_CONNECTIONS; uc_i++) {
			if (sx_macs_and_handlers[uc_i].handler == us_con_handle) {
				suc_slave_answer_data = uc_i;
				break;
			}
		}
	}
}

static void app_master_process(void)
{
	uint8_t uc_i;

	/* Check if I have open connection */
	if (suc_master_connecting_number == NUM_OPEN_CONNECTIONS) {
		for (uc_i = 0; uc_i < NUM_OPEN_CONNECTIONS; uc_i++) {
			if (sx_macs_and_handlers[uc_i].handler == MAC_INVALID_HANDLER) {
				int32_t diff;

				diff = sx_macs_and_handlers[uc_i].time - sul_time;
				if (diff < 0) {
					prime_cl_null_establish_request(sx_macs_and_handlers[uc_i].eui48_dst, TYPE_DC, NULL, 0, true, 0, suc_ae);
					suc_master_connecting_number = uc_i;
				}
			}
		}
	}

	/* Check if am cycling */
	if (suc_master_sending_data_number == NUM_OPEN_CONNECTIONS) {
		for (uc_i = 0; uc_i < NUM_OPEN_CONNECTIONS; uc_i++) {
			if (sx_macs_and_handlers[uc_i].handler != MAC_INVALID_HANDLER) {
				int32_t diff;

				diff = sx_macs_and_handlers[uc_i].time - sul_time;
				if (diff < 0) {
					char data_sent[100];
					sprintf(data_sent, "Hello I am %02x:%02x:%02x:%02x:%02x:%02x", suc_eui48[0], suc_eui48[1], suc_eui48[2], suc_eui48[3], suc_eui48[4], suc_eui48[5]);
					prime_cl_null_data_request(sx_macs_and_handlers[uc_i].handler, (uint8_t *)data_sent, strlen(data_sent), 1, 0);
					suc_master_sending_data_number = uc_i;
				}
			}
		}
	}
}

static void app_slave_process(void)
{
	if (suc_slave_response_connection < NUM_OPEN_CONNECTIONS) {
		prime_cl_null_establish_response(sx_macs_and_handlers[suc_slave_response_connection].handler, MAC_ESTABLISH_RESPONSE_ANSWER_ACCEPT, NULL, 0, 1);
		suc_slave_response_connection = NUM_OPEN_CONNECTIONS;
	}

	if (suc_slave_answer_data < NUM_OPEN_CONNECTIONS) {
		char data_sent[100];

		sprintf(data_sent, "How are you? I am %02x:%02x:%02x:%02x:%02x:%02x", suc_eui48[0], suc_eui48[1], suc_eui48[2], suc_eui48[3], suc_eui48[4], suc_eui48[5]);
		prime_cl_null_data_request(sx_macs_and_handlers[suc_slave_answer_data].handler, (uint8_t *)data_sent, strlen(data_sent), 1, 0);
		suc_slave_answer_data = NUM_OPEN_CONNECTIONS;
	}
}

void app_dc_init(void)
{
	st_app_dc = ST_IDLE;

	sul_time = 0;
	suc_master_connecting_number = NUM_OPEN_CONNECTIONS;
	suc_master_sending_data_number = NUM_OPEN_CONNECTIONS;

	suc_slave_response_connection = NUM_OPEN_CONNECTIONS;
	suc_slave_answer_data = NUM_OPEN_CONNECTIONS;
}

/**
 * \brief Process APP DC Application
 *
 */
void app_dc_process(void)
{
	static uint8_t uc_num_times = 1;

	uc_num_times--;
	if (uc_num_times) {
		return;
	}

	uc_num_times = 10;

	switch (st_app_dc) {
	case ST_DECIDE_MASTER_OR_SLAVE:
		_decide_master_or_slave();
		break;

	case ST_MASTER:
		app_master_process();
		break;

	case ST_SLAVE:
		app_slave_process();
		break;

	case ST_IDLE:
	default:
		break;
	}
	sul_time += APP_DC_TASK_RATE;
}
