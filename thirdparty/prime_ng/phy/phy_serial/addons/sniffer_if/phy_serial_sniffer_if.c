/**
 * \file
 *
 * \brief SNIFFER_IF : Sniffer Interface for Serial Physical layer
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
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "phy_serial_sniffer_if.h"
#include "phy_serial.h"
#include "conf_phy_serial.h"

#ifdef PHY_SERIAL_ADDONS_ENABLE

/** USI protocol */
#define SNIFFER_USI_PROTOCOL               0x13

/** \brief SNIFFER version */
/* @{ */
#define SNIFFER_ATPL230                    0x01
#define SNIFFER_PL360_PRIME                0x11
#define SNIFFER_PL360_PRIME_EXTENDED       0x31

#define SNIFFER_VERSION                    0x14
/* @} */

/** \brief PRIME Time definitions */
/* @{ */
#define TIME_PRIME_1_3_PREAMBLE_US         2048L
#define TIME_PRIME_1_3_HEADER_US           4480L
#define TIME_OFDM_SYMBOL_US                2240L
#define TIME_PRIME_PLUS_PREAMBLE_US        (TIME_PRIME_1_3_PREAMBLE_US * 4)
#define TIME_PRIME_PLUS_HEADER_US          (TIME_OFDM_SYMBOL_US * 4)

/* @} */

/** \brief Sniffer interface commands identifiers */
/* @{ */
#define SNIFFER_IF_PHY_COMMAND_SET_CHANNEL               2     /* SET PLC channel (1 = CENELEC- A) */
#define SNIFFER_IF_PHY_COMMAND_ENABLE_PRIME_PLUS_ROBUST  3     /* Enable robust modes of PRIME */
#define SNIFFER_IF_PHY_COMMAND_MESSAGE                   4     /* Inject message in PLC */
#define SNIFFER_IF_PHY_MESSAGE_TYPE_A                    0x20  /* TYPE A pdu received */
#define SNIFFER_IF_PHY_MESSAGE_TYPE_B                    0x21  /* TYPE B pdu received */
#define SNIFFER_IF_PHY_MESSAGE_TYPE_BC                   0x22  /* TYPE BC pdu received */
/* @} */

/**
 * \weakgroup phy_sniffer_group
 * @{
 */

/** \brief Data buffer used to send the received message
 * \note PHY_MAX_PPDU_SIZE + 32 header bytes
 */
static uint8_t spuc_sniffer_serial_buf[PHY_MAX_PPDU_SIZE + 32];

static uint32_t sul_last_buf_len;

/**
 * \brief Get duration message in microseconds
 *
 * \param mod_type       PRIME mode type
 * \param uc_symbols     Number of symbols in message
 *
 * \return Duration of message in microseconds
 */
static uint32_t _get_msg_duration_time(enum serial_mode_types mod_type, uint8_t uc_symbols)
{
	uint32_t ul_duration;

	if (mod_type == PHY_SERIAL_MODE_TYPE_A) {
		ul_duration = TIME_PRIME_1_3_PREAMBLE_US + TIME_PRIME_1_3_HEADER_US + (uc_symbols *  TIME_OFDM_SYMBOL_US);
	} else if (mod_type == PHY_SERIAL_MODE_TYPE_B) {
		ul_duration = TIME_PRIME_PLUS_PREAMBLE_US + TIME_PRIME_PLUS_HEADER_US + (uc_symbols * TIME_OFDM_SYMBOL_US);
	} else if (mod_type == PHY_SERIAL_MODE_TYPE_BC) {
		ul_duration = TIME_PRIME_1_3_PREAMBLE_US + TIME_PRIME_1_3_HEADER_US + TIME_PRIME_PLUS_PREAMBLE_US + TIME_PRIME_PLUS_HEADER_US +
				(uc_symbols * TIME_OFDM_SYMBOL_US);
	} else {
		ul_duration = 0;
	}

	return ul_duration;
}

uint16_t phy_serial_addon_sniffer_if_stringify_ind(uint8_t *puc_ind_data, xPhySerMsgRx_t *px_ind_msg)
{
	uint32_t ul_time_ini, ul_time_end, ul_duration, ul_len;
	uint8_t *puc_serial_rsp_buf;
	uint8_t uc_symbols;
	int8_t c_tmp;

	if (px_ind_msg->data_len == 0) {
		return 0;
	}

	puc_serial_rsp_buf = puc_ind_data;

	/* build response */
	if (px_ind_msg->mode == PHY_SERIAL_MODE_TYPE_A) {
		*puc_serial_rsp_buf++ = SNIFFER_IF_PHY_MESSAGE_TYPE_A;
	} else if (px_ind_msg->mode == PHY_SERIAL_MODE_TYPE_B) {
		*puc_serial_rsp_buf++ = SNIFFER_IF_PHY_MESSAGE_TYPE_B;
	} else if (px_ind_msg->mode == PHY_SERIAL_MODE_TYPE_BC) {
		*puc_serial_rsp_buf++ = SNIFFER_IF_PHY_MESSAGE_TYPE_BC;
	} else {
		return 0;
	}

	*puc_serial_rsp_buf++ = SNIFFER_VERSION;
	*puc_serial_rsp_buf++ = SNIFFER_PL360_PRIME;

	*puc_serial_rsp_buf++ = 0xF;  /* scheme */

	/* Compute length with CRC, */
	ul_len = px_ind_msg->data_len;

	/* Get Tx symbols */
	uc_symbols = ul_len / 6;  /* DBPSK_CC */
	if (ul_len % 6) {
		uc_symbols++;
	}

	*puc_serial_rsp_buf++ = uc_symbols;

	/* SNR = (QT/(3 * 4)) + 1 */
	/* QT = CINR_MIN + 12 */
	c_tmp = (px_ind_msg->cinr_min + 12 - 40) / 12;
	if (c_tmp > 7) {
		*puc_serial_rsp_buf++ = 7;
	} else if (c_tmp < 0) {
		*puc_serial_rsp_buf++ = 0;
	} else {
		*puc_serial_rsp_buf++ = c_tmp;
	}

	c_tmp = (px_ind_msg->cinr_avg + 12 - 40) / 4; /* Added 2 extra to make round instead trunk */
	if (c_tmp < 0) {
		*puc_serial_rsp_buf++ = 0;
	} else {
		*puc_serial_rsp_buf++ = c_tmp;
	}

	*puc_serial_rsp_buf++ = 0;    /* channel */

	*puc_serial_rsp_buf++ = px_ind_msg->cinr_min;
	*puc_serial_rsp_buf++ = px_ind_msg->bersoft;
	*puc_serial_rsp_buf++ = px_ind_msg->bersoft_max;

	/* padding (reserved bytes) */
	*puc_serial_rsp_buf++ = 0;
	*puc_serial_rsp_buf++ = 0;
	*puc_serial_rsp_buf++ = 0;
	*puc_serial_rsp_buf++ = 0;
	*puc_serial_rsp_buf++ = 0;
	*puc_serial_rsp_buf++ = 0;
	*puc_serial_rsp_buf++ = 0;
	*puc_serial_rsp_buf++ = 0;

	ul_time_ini = px_ind_msg->rx_time;

	ul_duration = _get_msg_duration_time((enum serial_mode_types)px_ind_msg->mode, uc_symbols);
	ul_time_end = ul_time_ini + ul_duration;

	*puc_serial_rsp_buf++ = (ul_time_ini >> 24);
	*puc_serial_rsp_buf++ = (ul_time_ini >> 16) & 0xFF;
	*puc_serial_rsp_buf++ = (ul_time_ini >> 8) & 0xFF;
	*puc_serial_rsp_buf++ = ul_time_ini & 0xFF;

	*puc_serial_rsp_buf++ = (ul_time_end >> 24);
	*puc_serial_rsp_buf++ = (ul_time_end >> 16) & 0xFF;
	*puc_serial_rsp_buf++ = (ul_time_end >> 8) & 0xFF;
	*puc_serial_rsp_buf++ = ul_time_end & 0xFF;

	*puc_serial_rsp_buf++ = 0;
	*puc_serial_rsp_buf++ = px_ind_msg->rssi_avg;

	/* mac_enable not supported */
	*puc_serial_rsp_buf++ = 0;

	*puc_serial_rsp_buf++ = (uint8_t)(ul_len >> 8);
	*puc_serial_rsp_buf++ = (uint8_t)(ul_len);

	memcpy(puc_serial_rsp_buf, px_ind_msg->data_buf, ul_len);
	puc_serial_rsp_buf += ul_len;

	return (puc_serial_rsp_buf - puc_ind_data);
}


void phy_serial_addon_sniffer_if_stringify_tx(xPhySerMsgTx_t *px_msg)
{
	uint32_t ul_len;
	uint8_t *puc_serial_rsp_buf;

	puc_serial_rsp_buf = &spuc_sniffer_serial_buf[0];

	/* build response */
	if (px_msg->data_len) {
		if (px_msg->mode == PHY_SERIAL_MODE_TYPE_A) {
			*puc_serial_rsp_buf++ = SNIFFER_IF_PHY_MESSAGE_TYPE_A;
		} else if (px_msg->mode == PHY_SERIAL_MODE_TYPE_B) {
			*puc_serial_rsp_buf++ = SNIFFER_IF_PHY_MESSAGE_TYPE_B;
		} else if (px_msg->mode == PHY_SERIAL_MODE_TYPE_BC) {
			*puc_serial_rsp_buf++ = SNIFFER_IF_PHY_MESSAGE_TYPE_BC;
		} else {
			return;
		}

		*puc_serial_rsp_buf++ = (uint8_t)SNIFFER_VERSION;
		*puc_serial_rsp_buf++ = (uint8_t)SNIFFER_PL360_PRIME;

		*puc_serial_rsp_buf++ = 0xF;  /* scheme */

		 /* Tx symbols will fill in cfm event */
		*puc_serial_rsp_buf++ = 0;

		*puc_serial_rsp_buf++ = 7;        /* SNR */
		*puc_serial_rsp_buf++ = 60;       /* EX_SNR */

		*puc_serial_rsp_buf++ = 0;        /* channel */

		*puc_serial_rsp_buf++ = 255;
		*puc_serial_rsp_buf++ = 0;
		*puc_serial_rsp_buf++ = 0;

		/* padding (reserved bytes) */
		*puc_serial_rsp_buf++ = 0;
		*puc_serial_rsp_buf++ = 0;
		*puc_serial_rsp_buf++ = 0;
		*puc_serial_rsp_buf++ = 0;
		*puc_serial_rsp_buf++ = 0;
		*puc_serial_rsp_buf++ = 0;
		*puc_serial_rsp_buf++ = 0;
		*puc_serial_rsp_buf++ = 0;

		/* Tx time ini will fill in cfm event */
		*puc_serial_rsp_buf++ = 0;
		*puc_serial_rsp_buf++ = 0;
		*puc_serial_rsp_buf++ = 0;
		*puc_serial_rsp_buf++ = 0;

		/* Tx time end will fill in cfm event */
		*puc_serial_rsp_buf++ = 0;
		*puc_serial_rsp_buf++ = 0;
		*puc_serial_rsp_buf++ = 0;
		*puc_serial_rsp_buf++ = 0;

		*puc_serial_rsp_buf++ = 0;
		/* Fix RSSI */
		*puc_serial_rsp_buf++ = 140;

		/* mac_enable not supported */
		*puc_serial_rsp_buf++ = 0;

		/* Compute length with CRC, */
		ul_len = px_msg->data_len;

		*puc_serial_rsp_buf++ = (uint8_t)(ul_len >> 8);
		*puc_serial_rsp_buf++ = (uint8_t)(ul_len);

		memcpy(puc_serial_rsp_buf, px_msg->data_buf, ul_len);
		puc_serial_rsp_buf += ul_len;

		/* Store buffering len to used in cfm event */
		ul_len = puc_serial_rsp_buf - spuc_sniffer_serial_buf;
		sul_last_buf_len = ul_len;
	}
}

uint16_t phy_serial_addon_sniffer_if_stringify_cfm(uint8_t *puc_cfm_data, xPhySerMsgTxResult_t *px_cfm_msg)
{
	uint32_t ul_time_ini, ul_time_end, ul_duration, ul_len;
	uint8_t *puc_serial_rsp_buf;
	uint8_t uc_symbols;

	if (px_cfm_msg->result != PHY_SER_TX_RESULT_SUCCESS) {
		return 0;
	}

	puc_serial_rsp_buf = &spuc_sniffer_serial_buf[0];

	puc_serial_rsp_buf += 4;

	/* restore buffering len */
	ul_len = sul_last_buf_len;

	/* Get Tx symbols */
	uc_symbols = ul_len / 6;  /* DBPSK_CC */
	if (ul_len % 6) {
		uc_symbols++;
	}

	*puc_serial_rsp_buf++ = uc_symbols;

	puc_serial_rsp_buf += 14;

	ul_time_ini = px_cfm_msg->tx_time;

	/* get Tx time ini */
	*puc_serial_rsp_buf++ = (uint8_t)(ul_time_ini >> 24);
	*puc_serial_rsp_buf++ = (uint8_t)(ul_time_ini >> 16);
	*puc_serial_rsp_buf++ = (uint8_t)(ul_time_ini >> 8);
	*puc_serial_rsp_buf++ = (uint8_t)(ul_time_ini);

	/* Tx time end will fill in cfm event */
	ul_duration = _get_msg_duration_time((enum serial_mode_types)px_cfm_msg->mode, uc_symbols);
	ul_time_end = ul_time_ini + ul_duration;

	*puc_serial_rsp_buf++ = (uint8_t)(ul_time_end >> 24);
	*puc_serial_rsp_buf++ = (uint8_t)(ul_time_end >> 16);
	*puc_serial_rsp_buf++ = (uint8_t)(ul_time_end >> 8);
	*puc_serial_rsp_buf = (uint8_t)(ul_time_end);

	/* copy internal buffering to app buffer */
	memcpy(puc_cfm_data, spuc_sniffer_serial_buf, ul_len);

	/* Return buffering len stored in tx event */
	return ul_len;
}

/**
 * \brief Initialize sniffer interface.
 *
 */
void phy_serial_addon_sniffer_if_init(void)
{

}

#endif /* PHY_SERIAL_ADDONS_ENABLE */
/* @} */
