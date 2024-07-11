/**
 * \file
 *
 * \brief PHY_SERIAL : Physical layer for serial interface
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
/* System includes */
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

#include "hal.h"
#include "prime_hal_wrapper.h"

#include "phy_serial.h"
#include "conf_phy_serial.h"
#ifdef PHY_SERIAL_ADDONS_ENABLE
#include "phy_serial_sniffer_if.h"
#endif

#define MAX_NUM_MSG_RCV    (4)

/** Modulation scheme of the payload: Differential 8PSK */
#define PROTOCOL_DBPSK_C      0x04

/** Internal Data structure to manage rx */
static xPhySerMsgRx_t phy_ser_rx_msg;
/** Internal Tx buf to store last TX message */
static uint8_t spuc_phy_tx_buffer[PHY_MAX_PPDU_SIZE];
/** Internal Tx Confirm to store last confirm */
static xPhySerMsgTxResult_t phy_ser_tx_result;
#ifdef PHY_SERIAL_ADDONS_ENABLE
/** Buffer to report */
static uint8_t spuc_addon_buffer[PHY_MAX_PPDU_SIZE + 32];
#endif

static struct {
	uint16_t us_len;
	uint8_t data_buf[PHY_MAX_PPDU_SIZE];
} sx_msg_rcv[MAX_NUM_MSG_RCV];

static uint8_t suc_output_msg_rcv;

static uint8_t suc_input_msg_rcv;

/** Phy Serial Callbacks pointers */
phy_serial_callbacks_t phy_ser_callbacks;

static bool _phy_serial_rx_frame(uint8_t *puc_rx_msg, uint16_t us_len) {
	if (!sx_msg_rcv[suc_input_msg_rcv].us_len) {
		memcpy(sx_msg_rcv[suc_input_msg_rcv].data_buf, puc_rx_msg, us_len);
		sx_msg_rcv[suc_input_msg_rcv].us_len = us_len;

		if (++suc_input_msg_rcv == MAX_NUM_MSG_RCV) {
			suc_input_msg_rcv = 0;
		}
	} else {
		;//printf("ERROR ,RX queue full\r\n");
	}

	return true;
}

void phy_serial_init(void) {
	uint8_t uc_i;

	phy_ser_callbacks.phy_ser_data_indication = NULL;
	phy_ser_callbacks.phy_ser_data_confirm = NULL;
	phy_ser_callbacks.phy_ser_addon_event = NULL;

	suc_input_msg_rcv = 0;
	suc_output_msg_rcv = 0;
	for (uc_i = 0; uc_i < MAX_NUM_MSG_RCV; uc_i++) {
		sx_msg_rcv[uc_i].us_len = 0;
	}

	prime_hal_usi_set_callback(PROTOCOL_PHY_SERIAL, _phy_serial_rx_frame, PHY_SERIAL_USI_PORT);

#ifdef PHY_SERIAL_ADDONS_ENABLE
	memset(spuc_addon_buffer, 0, sizeof(spuc_addon_buffer));
	phy_serial_addon_sniffer_if_init();
#endif
}

void phy_serial_set_callbacks(phy_serial_callbacks_t *phy_ser_cbs) {
	phy_ser_callbacks.phy_ser_data_indication = phy_ser_cbs->phy_ser_data_indication;
	phy_ser_callbacks.phy_ser_data_confirm = phy_ser_cbs->phy_ser_data_confirm;
#ifdef PHY_SERIAL_ADDONS_ENABLE
	phy_ser_callbacks.phy_ser_addon_event = phy_ser_cbs->phy_ser_addon_event;
#endif
}

uint8_t phy_serial_tx_frame(xPhySerMsgTx_t *px_msg) {
	x_usi_serial_cmd_params_t x_phy_ser_msg;

	memcpy(spuc_phy_tx_buffer, px_msg->data_buf, px_msg->data_len);

#ifdef PHY_SERIAL_ADDONS_ENABLE
	phy_serial_addon_sniffer_if_stringify_tx(px_msg);
#endif

	/* Set data for USI */
	x_phy_ser_msg.uc_protocol_type = PROTOCOL_PHY_SERIAL;
	x_phy_ser_msg.ptr_buf = spuc_phy_tx_buffer;
	x_phy_ser_msg.us_len = px_msg->data_len;

	/* Send packet */
	usi_status_t tx_result = prime_hal_usi_send_cmd(&x_phy_ser_msg);

	/* Generate Phy Data Confirm Callback */
	if (phy_ser_callbacks.phy_ser_data_confirm) {
		uint32_t ul_current_time;

		ul_current_time = prime_hal_timer_1us_get();

		switch (tx_result) {
		case USI_STATUS_OK:
			phy_ser_tx_result.result = PHY_SER_TX_RESULT_SUCCESS;
			break;
		default:
			phy_ser_tx_result.result = PHY_SER_TX_RESULT_BUSY_TX;
			break;
		}

		phy_ser_tx_result.tx_time = ul_current_time;
		phy_ser_tx_result.buff_id = px_msg->buff_id;
		phy_ser_tx_result.mode = PHY_SERIAL_MODE_TYPE_A;
		phy_ser_tx_result.rms_calc = 140;

#ifdef PHY_SERIAL_ADDONS_ENABLE
		uint16_t us_rsp_len = 0;
		us_rsp_len = phy_serial_addon_sniffer_if_stringify_cfm(spuc_addon_buffer, &phy_ser_tx_result);
		if (us_rsp_len) {
			phy_ser_callbacks.phy_ser_addon_event(spuc_addon_buffer, us_rsp_len);
		}
#endif

		phy_ser_callbacks.phy_ser_data_confirm(&phy_ser_tx_result);
	}

	return PHY_SER_TX_RESULT_PROCESS;
}


void phy_serial_process(void)
{
	while(sx_msg_rcv[suc_output_msg_rcv].us_len) {
			/* Generate Phy Data Indication Callback */
		if (phy_ser_callbacks.phy_ser_data_indication) {
			uint32_t ul_current_time;

			ul_current_time = prime_hal_timer_1us_get();

			/* Copy data payload */
			phy_ser_rx_msg.data_buf = sx_msg_rcv[suc_output_msg_rcv].data_buf ;
			phy_ser_rx_msg.data_len = sx_msg_rcv[suc_output_msg_rcv].us_len;
			/* Get header type */
			phy_ser_rx_msg.header_type = PHY_SER_GET_HEADER_TYPE(sx_msg_rcv[suc_output_msg_rcv].data_buf[0]);
			/* Fill other data with values that assure minimum quality */
			phy_ser_rx_msg.rx_time = ul_current_time;
			phy_ser_rx_msg.evm_header_acum = 0;
			phy_ser_rx_msg.evm_payload_acum = 0;
			phy_ser_rx_msg.evm_header = 0;
			phy_ser_rx_msg.evm_payload = 0;
			phy_ser_rx_msg.buff_id = 0;
			phy_ser_rx_msg.scheme = PROTOCOL_DBPSK_C;
			phy_ser_rx_msg.mode = PHY_SERIAL_MODE_TYPE_A;
			phy_ser_rx_msg.noise_result = 0;
			phy_ser_rx_msg.rssi_avg = 0;
			phy_ser_rx_msg.cinr_avg = 100;
			phy_ser_rx_msg.cinr_min = 100;
			phy_ser_rx_msg.bersoft = 0;
			phy_ser_rx_msg.bersoft_max = 0;
			phy_ser_rx_msg.qt = 255;
			phy_ser_rx_msg.snr_ex = 0;

#ifdef PHY_SERIAL_ADDONS_ENABLE
		uint16_t us_rsp_len = 0;
		us_rsp_len = phy_serial_addon_sniffer_if_stringify_ind(spuc_addon_buffer, &phy_ser_rx_msg);
		if (us_rsp_len) {
			phy_ser_callbacks.phy_ser_addon_event(spuc_addon_buffer, us_rsp_len);
		}
#endif

			phy_ser_callbacks.phy_ser_data_indication(&phy_ser_rx_msg);
		}

		sx_msg_rcv[suc_output_msg_rcv].us_len = 0;
		if (++suc_output_msg_rcv == MAX_NUM_MSG_RCV) {
			suc_output_msg_rcv = 0;
		}
	}
}

/* @} */
