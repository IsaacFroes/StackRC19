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

#include "app_emu.h"

#include <stdint.h>
#include <string.h>

#include "app.h"
#include "app_event_manager.h"
#include "prime_api.h"
#include "prime_api_defs.h"
#include "mac_pib.h"
#include "hal.h"
#include "hal_private.h"

/* APP include */

#define TASK_RATE_MS               (1)

#define TYPE_CONNECTION_APPEMU     250

#define NUM_TEST_APPEMU            (11)

#define LENGTH_HEADER_UP_MSG       (12)

#define BUFFER_APP_EMU             (512)

#define LEN_ID_SNA                 (6)

#define LEN_ID_SERIAL              (10)

static uint8_t sna[LEN_ID_SNA];
static uint8_t sid;
static uint16_t lnid;
static uint8_t serial_id[LEN_ID_SERIAL];

static uint16_t handler_appemu;
static uint8_t data_rx[BUFFER_APP_EMU];
static uint8_t data_tx[BUFFER_APP_EMU];

/*  timer app_emu connection */
static uint32_t ul_timer_connect;
static bool b_timer_running;
static uint32_t cycles_app_emu_counter;

/* global status node information */
static node_state_t uc_node_state;

static x_mac_cfg_t mac;

/* Definition of the test */
typedef struct {
	/** Step of the test, to fill up the message */
	uint8_t uc_test_number;
	/** Bytes sent by the Base Node */
	uint16_t us_bytes_sent;
	/** Bytes expected in the answer */
	uint16_t us_bytes_expected;
} test_info_t;

/* Test battery to execute */
static test_info_t const conf_test[NUM_TEST_APPEMU] = {
	{2, 24, 36},
	{3, 24, 31},
	{4, 75, 138},
	{4, 16, 52},
	{5, 17, 17},
	{5, 29, 29},
	{5, 65, 65},
	{5, 137, 137},
	{5, 209, 209},
	{5, 281, 281},
	{5, 353, 353}
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
 * \brief Generate serial number using mac value
 *
 * \param puc_serial_board  Pointer to the serial board
 */
static void _generate_serial_app_emu(uint8_t *puc_serial_board)
{
	uint8_t uc_index;
	uint8_t uc_pos;
	uint8_t uc_num, uc_num1;

	hal_get_config_info(CONFIG_TYPE_MAC_INFO, sizeof(mac), &mac);

	memcpy(puc_serial_board, "ATM", 3);
	/* convert hex to ascii */
	uc_index = 3;
	uc_pos = 3;
	while (uc_pos < 9) {
		uc_num = ((mac.uc_mac[uc_index] & 0xf0) >> 4);
		uc_num1 = (mac.uc_mac[uc_index] & 0x0f);
		uc_index++;
		*(puc_serial_board + uc_pos++) = _convertHEX2STR(uc_num);
		*(puc_serial_board + uc_pos++) = _convertHEX2STR(uc_num1);
	}
	*(puc_serial_board + uc_pos) = NULL;
}

/**
 * \brief reset SID and LNID values
 */
static void _appemu_reset_data(void)
{
	sid = 0xff;
	lnid = 0x3fff;
}

/**
 * \brief build app_emu message
 *
 * \param step             Step
 * \param us_data_len_rx   Received data length
 *
 * \retval message length
 */
static uint16_t _fill_data_tx(uint8_t step, uint16_t ui_data_len_rx)
{
	uint16_t i, len_tx;
	uint8_t *ptr_data;
	uint8_t uc_data;

	memcpy(data_tx, "UPATL", 5);
	data_tx[5] = step + '0';
	memcpy(&data_tx[6], &data_rx[6], 6); /* Copy time in tenth of seconds */

	/* Look for in the table for the received message */
	for (i = 0; i < NUM_TEST_APPEMU; i++) {
		if ((step == conf_test[i].uc_test_number) && (ui_data_len_rx == conf_test[i].us_bytes_sent)) {
			break;
		}
	}
	if (i < NUM_TEST_APPEMU) {
		len_tx = conf_test[i].us_bytes_expected;
	} else {
		len_tx = LENGTH_HEADER_UP_MSG;
	}

	ptr_data = &data_tx[LENGTH_HEADER_UP_MSG];
	uc_data = 'A';

	for (i = LENGTH_HEADER_UP_MSG; i < len_tx; i++) {
		*ptr_data++ = uc_data++;
		if (uc_data > 'Z') {
			uc_data = 'A';
		}
	}
	return len_tx;
}

/**
 * \brief send establish connection of the APP_EMU connection type
 *
 * \param pxTimer  Timer handler
 */

static void _connect_appemu(void)
{
	uint8_t puc_eui48[6] = {0, 0, 0, 0, 0, 0};

	prime_cl_null_establish_request(puc_eui48, TYPE_CONNECTION_APPEMU, serial_id, 9, 0, 0, 0);

#ifdef APP_EMU_DEBUG_ENABLE
	printf("APP_EMU:  NODE CONECTION starting...\r\n");
#endif
}

/**
 * \brief callback application confime  for node registration
 *
 * \param puc_sna    Pointer to the SNA
 * \param uc_sid     SID
 */
void app_emu_mlme_register_ind_cb(uint8_t *puc_sna, uint8_t uc_sid)
{
	(void)(uc_sid);

	memcpy(sna, puc_sna, 6);

	prime_cl_null_mlme_get_request(PIB_MAC_LNID);

	prime_cl_null_mlme_get_request(PIB_MAC_SID);

	uc_node_state = NODE_REGISTERED;

#ifdef APP_EMU_DEBUG_ENABLE
	printf("APP_EMU: MLME INDICATION CB NODE REGISTERED...\r\n");
#endif
}

/**
 * \brief callback application confirm for node unregistered
 */
void app_emu_mlme_unregister_cb(void)
{
	_appemu_reset_data();

	uc_node_state = NODE_REGISTERED;

#ifdef APP_EMU_DEBUG_ENABLE
	printf("APP_EMU:  NODE UNREGISTERED...\r\n");
#endif
}

/**
 * \brief  call back function of the connection stablish request
 *
 * \param us_con_handle Unique identifier of the connection
 * \param uc_result     Result of the connection establishment process
 * \param puc_eui48     Address of the node to which this connection is being established
 * \param uc_type       Convergence Layer type of the connection
 * \param puc_data      Data associated with the connection establishment procedure
 * \param us_data_len   Length of the data in bytes
 * \param uc_ae         Flag to indicate that authentication and encryption is requested
 */
void app_emu_establish_confirm_cb(uint16_t us_con_handle, mac_establish_confirm_result_t uc_result, uint8_t *puc_eui48, uint8_t uc_type,
		uint8_t *puc_data, uint16_t us_data_len, uint8_t uc_ae)
{
	(void)(puc_eui48);
	(void)(puc_data);
	(void)(us_data_len);
	(void)(uc_ae);

#ifdef APP_EMU_DEBUG_ENABLE
	printf("APP_EMU:  stablish confirm...\r\n");
#endif

	if (uc_type == TYPE_CONNECTION_APPEMU) {
		if (uc_result == MAC_ESTABLISH_CONFIRM_RESULT_SUCCESS) {
			handler_appemu = us_con_handle;
			uc_node_state = NODE_CONNECTED_APPEMU;
#ifdef APP_EMU_DEBUG_ENABLE
			printf("APP_EMU: APP_Emu connection open success\r\n");
#endif
		} else {
			ul_timer_connect = ((20 + (hal_trng_read() % 20)) * 100) / TASK_RATE_MS;
			b_timer_running = false;

#ifdef APP_EMU_DEBUG_ENABLE
			printf("APP_EMU: connection fail...retry in %u ms\r\n", ul_timer_connect * TASK_RATE_MS);
#endif
		}
	}
}

/**
 * \brief callback application to manage the release indication
 *
 * \param us_con_handle Unique identifier of the connection
 * \param uc_reason     Cause of the connection release
 */
void app_emu_release_ind_cb(uint16_t us_con_handle, mac_release_indication_reason_t uc_reason)
{
	(void)(uc_reason);

	if (us_con_handle == handler_appemu) {
		handler_appemu = 0xffff;
		prime_cl_null_release_response(us_con_handle, MAC_RELEASE_RESPONSE_ACCEPT);
		ul_timer_connect = ((40 + (hal_trng_read() % 20)) * 100) / TASK_RATE_MS;
		b_timer_running = true;
		uc_node_state = NODE_REGISTERED;
		cycles_app_emu_counter++;
#ifdef APP_EMU_DEBUG_ENABLE
		printf("APP_EMU: test cycle %u ms\r\n", cycles_app_emu_counter);
		printf("APP_EMU: Received release indication...retry in %u ms\r\n", ul_timer_connect * TASK_RATE_MS);
#endif
	}
}

/**
 * \brief data send indication function.
 */
void app_emu_data_indication_cb(uint16_t us_con_handle, uint8_t *puc_data, uint16_t us_data_len, uint32_t ul_time_ref)
{
	uint16_t data_len_tx;
	uint8_t step;

	if (us_con_handle == handler_appemu) {
		memcpy(data_rx, puc_data, us_data_len);
		step = data_rx[5] - '0';
		data_len_tx = _fill_data_tx(step, us_data_len);
		prime_cl_null_data_request(handler_appemu, data_tx, data_len_tx, 2, ul_time_ref);

#ifdef APP_EMU_DEBUG_ENABLE
		printf("APP_EMU: Received data indication \r\n");
#endif
	}
}

/**
 * \brief get result of the mlme get queries
 *
 * \param x_status           Status
 * \param us_pib_attrib      PIB attribute
 * \param ul_pib_value       PIB attribute value
 * \param len                PIB attribute value size
 */
void app_emu_mlme_get_confirm_cb(mlme_result_t x_status, uint16_t us_pib_attrib, void *ul_pib_value, uint8_t len)
{
	(void)(len);

	if (x_status != MLME_RESULT_DONE) {
		return;
	}

	if (us_pib_attrib == PIB_MAC_LNID) {
		lnid = *((uint16_t *)ul_pib_value);
	}

	if (us_pib_attrib == PIB_MAC_SID) {
		sid = *((uint8_t *)ul_pib_value);
	}

	if ((sid != 0xff) && (lnid != 0x3fff)) {
		ul_timer_connect = ((50 + (hal_trng_read() % 20)) * 100) / TASK_RATE_MS;
		b_timer_running = true;

#ifdef APP_EMU_DEBUG_ENABLE
		printf("APP_EMU: waiting to open connection app emu %u ms\r\n", ul_timer_connect * TASK_RATE_MS);
#endif
	}
}

/**
 * \brief app emu task initialization.
 */
void app_emu_init(void)
{
	_generate_serial_app_emu(serial_id);

	_appemu_reset_data();

	/* timer task to launch connection. */
	ul_timer_connect = 0;
	b_timer_running = false;

	hal_trng_init();
	app_event_dispatcher_init(get_app_mode());

	uc_node_state = NODE_UNREGISTERED;

#ifdef APP_EMU_DEBUG_ENABLE
	printf("Selected APP_EMU Application: Service Node...\r\n");
#endif
}

/**
 * \brief app_emu main process
 */
void app_emu_process(void)
{
	/* task timer delay connection  */
	if (b_timer_running) {
		if ((--ul_timer_connect) == 0) {
			b_timer_running = false;
			ul_timer_connect = 0;
			_connect_appemu();
		}
	}
}

/**
 * \brief Return node state
 *
 * \retval Node state
 */
node_state_t app_emu_node_state(void)
{
	return uc_node_state;
}
