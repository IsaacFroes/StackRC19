/**
 * \file
 *
 * \brief MODEM : Modem Application for Microchip PRIME NG Base Node
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

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "modem.h"
#include "conf_prime_stack.h"
#if USED_SSCS == SSCS_432
#include "cl_432_defs.h"
#elif USED_SSCS == SSCS_IPV6
#include "cl_ipv6_defs.h"
#ifdef __PRIME_GATEWAY__
#include "gw_ipv6.h"
#endif
#else
#error No SSCS defined
#endif
#include "bmng_defs.h"
#include "mac_defs.h"
#include "prime_api.h"
#include "prime_api_defs.h"
#include "hal.h"
#include "hal_private.h"
#include "hal_regions.h"
#include "board.h"
#include "conf_app_example.h"

#define MAX_NUM_MSG_RCV    (5)

#if USED_SSCS == SSCS_432
#define MAX_LENGTH_BUFF   MAX_LENGTH_432_DATA
#elif USED_SSCS == SSCS_IPV6
#define MAX_LENGTH_BUFF   MAX_LENGTH_IPV6_DATA
#endif

/** PRIME regions configuration. 1 region is defined */
x_fu_region_cfg_t sx_prime_reg;

/** buffer used to tx serialization */
static uint8_t suc_serial_buf[MAX_LENGTH_BUFF];

/** Queue of buffers in rx and pointers*/
static struct {
	uint16_t us_len;
	uint8_t data_buf[MAX_LENGTH_BUFF];
} sx_msg_rcv[MAX_NUM_MSG_RCV];
static uint8_t suc_output_msg_rcv;
static uint8_t suc_input_msg_rcv;
#if USED_SSCS == SSCS_IPV6
#ifndef __PRIME_GATEWAY__
static uint8_t spuc_data_buf[MAX_LENGTH_IPV6_DATA];
#endif
#endif



/** Structure used for USI send request */
static x_usi_serial_cmd_params_t x_usi_msg;

/** Data transmission indication variable */
static uint8_t uc_rxdata_ind;
/** Data reception indication variable */
static uint8_t uc_txdata_ind;

static void _modem_set_callbacks(void);

/** Data transmission indication function */
uint8_t modem_txdata_ind(void)
{
	if (uc_txdata_ind) {
		uc_txdata_ind = false;
		return true;
	} else {
		return false;
	}
}

/** Data reception indication function */
uint8_t modem_rxdata_ind(void)
{
	if (uc_rxdata_ind) {
		uc_rxdata_ind = false;
		return true;
	} else {
		return false;
	}
}

/**
 * MAC Establish Indication
 * - us_con_handle: Unique identifier of the connection
 * - puc_eui48:     Address of the node which initiates the connection establish procedure
 * - uc_type:       Convergence Layer type of the connection
 * - puc_data:      Data associated with the connection establishment procedure
 * - us_data_len:   Length of the data in bytes
 * - uc_cfbytes:    Flag to indicate whether or not the connection should use the contention or contention-free channel access scheme
 * - uc_ae (v1.4):  Flag to indicate that authentication and encryption is requested
 */
static void _establish_ind_cb(uint16_t us_con_handle, uint8_t *puc_eui48, uint8_t uc_type, uint8_t *puc_data, uint16_t us_data_len, uint8_t uc_cfbytes, uint8_t uc_ae)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_null_establish_ind_cmd;
	*puc_buf++ = (uint8_t)(us_con_handle >> 8);
	*puc_buf++ = (uint8_t)(us_con_handle);
	memcpy(puc_buf, puc_eui48, 6);
	puc_buf += 6;
	*puc_buf++ = uc_type;
	*puc_buf++ = (uint8_t)(us_data_len >> 8);
	*puc_buf++ = (uint8_t)(us_data_len);
	memcpy(puc_buf, puc_data, us_data_len);
	puc_buf += us_data_len;
	*puc_buf++ = uc_cfbytes;
	*puc_buf++ = uc_ae;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * MAC Establish Confirm
 * - us_con_handle: Unique identifier of the connection
 * - uc_result:     Result of the connection establishment process
 * - puc_eui48:     Address of the node to which this connection is being established
 * - uc_type:       Convergence Layer type of the connection
 * - puc_data:      Data associated with the connection establishment procedure
 * - us_data_len:   Length of the data in bytes
 * - uc_ae (v1.4):  Flag to indicate that authentication and encryption is requested
 */
static void _establish_cfm_cb(uint16_t us_con_handle, mac_establish_confirm_result_t uc_result,
uint8_t *puc_eui48, uint8_t uc_type, uint8_t *puc_data, uint16_t us_data_len, uint8_t uc_ae)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_null_establish_cfm_cmd;
	*puc_buf++ = (uint8_t)(us_con_handle >> 8);
	*puc_buf++ = (uint8_t)(us_con_handle);
	*puc_buf++ = uc_result;
	memcpy(puc_buf, puc_eui48, 6);
	puc_buf += 6;
	*puc_buf++ = uc_type;
	*puc_buf++ = (uint8_t)(us_data_len >> 8);
	*puc_buf++ = (uint8_t)(us_data_len);
	memcpy(puc_buf, puc_data, us_data_len);
	puc_buf += us_data_len;
	*puc_buf++ = uc_ae;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * MAC Release Indication
 * - us_con_handle: Unique identifier of the connection
 * - uc_reason:     Cause of the connection release
 */
static void _release_ind_cb(uint16_t us_con_handle, mac_release_indication_reason_t uc_reason)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_null_release_ind_cmd;
	*puc_buf++ = (uint8_t)(us_con_handle >> 8);
	*puc_buf++ = (uint8_t)(us_con_handle);
	*puc_buf++ = uc_reason;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * MAC Release Confirm
 * - us_con_handle: Unique identifier of the connection
 * - uc_result:     Result of the connection release process
 */
static void _release_cfm_cb(uint16_t us_con_handle, mac_release_confirm_result_t uc_result)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_null_release_cfm_cmd;
	*puc_buf++ = (uint8_t)(us_con_handle >> 8);
	*puc_buf++ = (uint8_t)(us_con_handle);
	*puc_buf++ = uc_result;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * MAC Join Indication
 * - us_con_handle:        Unique identifier of the connection
 * - puc_eui48:            Address of the node which wishes to join the multicast group
 * - uc_con_type:          Connection type
 * - puc_data:             Data associated with the join request procedure
 * - us_data_len:          Length of the data in bytes
 * - uc_ae (v1.4):         Flag to indicate that authentication and encryption is requested
 */
static void _join_ind_cb(uint16_t us_con_handle, uint8_t *puc_eui48, uint8_t uc_con_type, uint8_t *puc_data, uint16_t us_data_len, uint8_t uc_ae)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_null_join_ind_cmd;
	*puc_buf++ = (uint8_t)(us_con_handle >> 8);
	*puc_buf++ = (uint8_t)(us_con_handle);
	memcpy(puc_buf, puc_eui48, 6);
	puc_buf += 6;
	*puc_buf++ = uc_con_type;
	*puc_buf++ = (uint8_t)(us_data_len >> 8);
	*puc_buf++ = (uint8_t)(us_data_len);
	memcpy(puc_buf, puc_data, us_data_len);
	puc_buf += us_data_len;
	*puc_buf++ = uc_ae;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * MAC Join Confirm
 * - us_con_handle:        Unique identifier of the connection
 * - uc_result:            Result of the join request process
 * - uc_ae (v1.4):         Flag to indicate that authentication and encryption is requested
 */
static void _join_cfm_cb(uint16_t us_con_handle, mac_join_confirm_result_t uc_result, uint8_t uc_ae)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_null_join_cfm_cmd;
	*puc_buf++ = (uint8_t)(us_con_handle >> 8);
	*puc_buf++ = (uint8_t)(us_con_handle);
	*puc_buf++ = uc_result;
	*puc_buf++ = uc_ae;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * MAC Leave Confirm
 * - us_con_handle:        Unique identifier of the connection
 * - uc_result:            Result of the leave request process
 */
static void _leave_cfm_cb(uint16_t us_con_handle, mac_leave_confirm_result_t uc_result)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_null_leave_cfm_cmd;
	*puc_buf++ = (uint8_t)(us_con_handle >> 8);
	*puc_buf++ = (uint8_t)(us_con_handle);
	*puc_buf++ = uc_result;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * MAC Leave Indication
 * - us_con_handle:        Unique identifier of the connection
 * - puc_eui48:            Address of the node to remove from the multicast group
 */
static void _leave_ind_cb(uint16_t us_con_handle, uint8_t *puc_eui48)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_null_leave_ind_cmd;
	*puc_buf++ = (uint8_t)(us_con_handle >> 8);
	*puc_buf++ = (uint8_t)(us_con_handle);
	if (puc_eui48 != NULL) {
		memcpy(puc_buf, puc_eui48, 6);
	} else {
		memset(puc_buf, 0xff, 6);
	}

	puc_buf += 6;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * MAC Data Confirm
 * - us_con_handle:    Unique identifier of the connection
 * - puc_data:         Pointer to data to be transmitted through this connection
 * - drt_result:       Result of the transmission (MAC_DATA_SUCCESS, MAC_DATA_TIMEOUT, MAC_DATA_ERROR_SENDING)
 */
static void _data_cfm_cb(uint16_t us_con_handle, uint8_t *puc_data, mac_data_result_t drt_result)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_null_data_cfm_cmd;
	*puc_buf++ = (uint8_t)(us_con_handle >> 8);
	*puc_buf++ = (uint8_t)(us_con_handle);
	*puc_buf++ = (uint8_t)((uint32_t)puc_data >> 24);
	*puc_buf++ = (uint8_t)((uint32_t)puc_data >> 16);
	*puc_buf++ = (uint8_t)((uint32_t)puc_data >> 8);
	*puc_buf++ = (uint8_t)((uint32_t)puc_data);
	*puc_buf++ = drt_result;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * MAC Data Indication
 * - us_con_handle:           Unique identifier of the connection
 * - puc_data:                Pointer to data to be received through this connection
 * - us_data_len:             Length of the data in bytes
 * - ul_time_ref (v1.4):      Time reference (in 10s of microsec)
 */
static void _data_ind_cb(uint16_t us_con_handle, uint8_t *puc_data, uint16_t us_data_len, uint32_t ul_time_ref)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_null_data_ind_cmd;
	*puc_buf++ = (uint8_t)(us_con_handle >> 8);
	*puc_buf++ = (uint8_t)(us_con_handle);
	*puc_buf++ = (uint8_t)(us_data_len >> 8);
	*puc_buf++ = (uint8_t)(us_data_len);
	memcpy(puc_buf, puc_data, us_data_len);
	puc_buf += us_data_len;
	*puc_buf++ = (uint8_t)(ul_time_ref >> 24);
	*puc_buf++ = (uint8_t)(ul_time_ref >> 16);
	*puc_buf++ = (uint8_t)(ul_time_ref >> 8);
	*puc_buf++ = (uint8_t)(ul_time_ref);

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);

	/* Rx data indication */
	uc_rxdata_ind = true;
}

/**
 * PLME Reset Confirm
 * - x_result:           Result
 * - us_pch (v1.4):      Physical channel
 */
static void _plme_reset_cfm_cb(plme_result_t x_result, uint16_t us_pch)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_null_plme_reset_cfm_cmd;
	*puc_buf++ = x_result;
	*puc_buf++ = (uint8_t)(us_pch >> 8);
	*puc_buf++ = (uint8_t)us_pch;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * PLME Sleep Confirm
 * - x_result:           Result
 * - us_pch (v1.4):      Physical channel
 */
static void _plme_sleep_cfm_cb(plme_result_t x_result, uint16_t us_pch)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_null_plme_sleep_cfm_cmd;
	*puc_buf++ = x_result;
	*puc_buf++ = (uint8_t)(us_pch >> 8);
	*puc_buf++ = (uint8_t)us_pch;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * PLME Resume Confirm
 * - x_result:           Result
 * - us_pch (v1.4):      Physical channel
 */
static void _plme_resume_cfm_cb(plme_result_t x_result, uint16_t us_pch)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_null_plme_resume_cfm_cmd;
	*puc_buf++ = x_result;
	*puc_buf++ = (uint8_t)(us_pch >> 8);
	*puc_buf++ = (uint8_t)us_pch;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * PLME Get Confirm
 * - x_status:           Status
 * - us_pib_attrib:      PIB attribute
 * - pv_pib_value:       PIB attribute value
 * - uc_pib_size:        PIB attribute value size
 * - us_pch (v1.4):      Physical channel
 */
static void _plme_get_cfm_cb(plme_result_t x_status, uint16_t us_pib_attrib, void *pv_pib_value, uint8_t uc_pib_size, uint16_t us_pch)
{
	uint8_t *puc_buf;
	uint16_t us_temp;
	uint32_t ul_temp;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_null_plme_get_cfm_cmd;
	*puc_buf++ = x_status;
	*puc_buf++ = (uint8_t)(us_pib_attrib >> 8);
	*puc_buf++ = (uint8_t)(us_pib_attrib);
	*puc_buf++ = uc_pib_size;

	/* Check size */
	switch (uc_pib_size) {
	case 2:
		/* Extract value */
		us_temp = *((uint16_t *)pv_pib_value);
		/* Copy value into buffer with MSB in MSB */
		*puc_buf++ = (uint8_t)(us_temp >> 8);
		*puc_buf++ = (uint8_t)us_temp;
		break;

	case 4:
		ul_temp = *((uint32_t *)pv_pib_value);
		/* Copy value into buffer with MSB in MSB */
		*puc_buf++ = (uint8_t)(ul_temp >> 24);
		*puc_buf++ = (uint8_t)(ul_temp >> 16);
		*puc_buf++ = (uint8_t)(ul_temp >> 8);
		*puc_buf++ = (uint8_t)ul_temp;
		break;

	default:
		/* Copy value into buffer */
		memcpy(puc_buf, (uint8_t *)pv_pib_value, uc_pib_size);
		/* Increase pointer */
		puc_buf += uc_pib_size;
	}

	*puc_buf++ = (uint8_t)(us_pch >> 8);
	*puc_buf++ = (uint8_t)us_pch;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * PLME Set Confirm
 * - x_result:           Result
 * - us_pch (v1.4):      Physical channel
 */
static void _plme_set_cfm_cb(plme_result_t x_result, uint16_t us_pch)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_null_plme_set_cfm_cmd;
	*puc_buf++ = x_result;
	*puc_buf++ = (uint8_t)(us_pch >> 8);
	*puc_buf++ = (uint8_t)us_pch;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * MLME Promote Confirm
 * - x_result:      Result
 */
static void _mlme_promote_cfm_cb(mlme_result_t x_result)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_null_mlme_promote_cfm_cmd;
	*puc_buf++ = x_result;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * MLME MultiPHY Promote Confirm  (v1.4)
 * - x_result:      Result
 */
static void _mlme_mp_promote_cfm_cb(mlme_result_t x_result)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_null_mlme_mp_promote_cfm_cmd;
	*puc_buf++ = x_result;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * MLME Reset Confirm
 * - x_result:           Result
 */
static void _mlme_reset_cfm_cb(mlme_result_t x_result)
{
	uint8_t *puc_buf;

	/* Check result */
	if (x_result == MLME_RESULT_DONE) {
		/* Set callback functions */
		_modem_set_callbacks();
	}

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_null_mlme_reset_cfm_cmd;
	*puc_buf++ = x_result;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * MLME Get Confirm
 * - x_status:           Status
 * - us_pib_attrib:      PIB attribute
 * - pv_pib_value:       PIB attribute value
 * - uc_pib_size:        PIB attribute value size
 */
static void _mlme_get_cfm_cb(mlme_result_t x_status, uint16_t us_pib_attrib, void *pv_pib_value, uint8_t uc_pib_size)
{
	uint8_t *puc_buf;
	uint16_t us_temp;
	uint32_t ul_temp;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_null_mlme_get_cfm_cmd;
	*puc_buf++ = x_status;
	*puc_buf++ = (uint8_t)(us_pib_attrib >> 8);
	*puc_buf++ = (uint8_t)(us_pib_attrib);
	*puc_buf++ = uc_pib_size;

	/* Check size */
	switch (uc_pib_size) {
	case 2:
		/* Extract value */
		us_temp = *((uint16_t *)pv_pib_value);
		/* Copy value into buffer with MSB in MSB */
		*puc_buf++ = (uint8_t)(us_temp >> 8);
		*puc_buf++ = (uint8_t)us_temp;
		break;

	case 4:
		ul_temp = *((uint32_t *)pv_pib_value);
		/* Copy value into buffer with MSB in MSB */
		*puc_buf++ = (uint8_t)(ul_temp >> 24);
		*puc_buf++ = (uint8_t)(ul_temp >> 16);
		*puc_buf++ = (uint8_t)(ul_temp >> 8);
		*puc_buf++ = (uint8_t)ul_temp;
		break;

	default:
		/* Copy value into buffer */
		memcpy(puc_buf, (uint8_t *)pv_pib_value, uc_pib_size);
		/* Increase pointer */
		puc_buf += uc_pib_size;
	}

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * MLME List Get Confirm
 * - x_status:           Status
 * - us_pib_attrib:      PIB attribute
 * - puc_pib_buff:       Buffer with PIB attribute values
 * - us_pib_len:         Buffer length
 */
static void _mlme_list_get_cfm_cb(mlme_result_t x_status, uint16_t us_pib_attrib, uint8_t *puc_pib_buff, uint16_t us_pib_len)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_null_mlme_list_get_cfm_cmd;
	*puc_buf++ = x_status;
	*puc_buf++ = (uint8_t)(us_pib_attrib >> 8);
	*puc_buf++ = (uint8_t)(us_pib_attrib);
	*puc_buf++ = (uint8_t)(us_pib_len >> 8);
	*puc_buf++ = (uint8_t)(us_pib_len);
	memcpy(puc_buf, (uint8_t *)puc_pib_buff, us_pib_len);
	puc_buf += us_pib_len;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * MLME Set Confirm
 * - x_result:           Result
 */
static void _mlme_set_cfm_cb(mlme_result_t x_result)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_null_mlme_set_cfm_cmd;
	*puc_buf++ = x_result;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

#if USED_SSCS == SSCS_432
/**
 * CL 432 Data indication
 * - uc_dst_lsap:      Destination LSAP
 * - uc_src_lsap:      Source LSAP
 * - us_dst_address:   Destination 432 Address
 * - src_address:      Source Address
 * - puc_data:         Pointer to received data
 * - us_lsdu_len:      Length of the data
 * - uc_link_class:    Link class (non used)
 */
static void _cl_432_dl_data_ind_cb(uint8_t uc_dst_lsap, uint8_t uc_src_lsap, uint16_t us_dst_address, uint16_t us_src_address,
		uint8_t *puc_data, uint16_t us_lsdu_len, uint8_t uc_link_class)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_432_dl_data_ind_cmd;
	*puc_buf++ = uc_dst_lsap;
	*puc_buf++ = uc_src_lsap;
	*puc_buf++ = (uint8_t)(us_dst_address >> 8);
	*puc_buf++ = (uint8_t)(us_dst_address);
	*puc_buf++ = (uint8_t)(us_src_address >> 8);
	*puc_buf++ = (uint8_t)(us_src_address);
	*puc_buf++ = (uint8_t)(us_lsdu_len >> 8);
	*puc_buf++ = (uint8_t)(us_lsdu_len);
	memcpy(puc_buf, puc_data, us_lsdu_len);
	puc_buf += us_lsdu_len;
	*puc_buf++ = uc_link_class;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);

	/* Rx data indication */
	uc_rxdata_ind = true;
}

/**
 * CL 432 Data confirm
 * - uc_dst_lsap:      Destination LSAP
 * - uc_src_lsap:      Source LSAP
 * - us_dst_address:   Destination 432 Address
 * - uc_tx_status:     Tx status
 */
static void _cl_432_dl_data_cfm_cb(uint8_t uc_dst_lsap, uint8_t uc_src_lsap, uint16_t us_dst_address, dl_432_tx_status_t uc_tx_status)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_432_dl_data_cfm_cmd;
	*puc_buf++ = uc_dst_lsap;
	*puc_buf++ = uc_src_lsap;
	*puc_buf++ = (uint8_t)(us_dst_address >> 8);
	*puc_buf++ = (uint8_t)(us_dst_address);
	*puc_buf++ = (uint8_t)uc_tx_status;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * CL 432 Join indication
 * - puc_device_id:          Pointer to the device identifier data
 * - uc_device_id_len:       Length of the device identfier
 * - us_dst_address:         Destination 432 Address
 * - puc_mac:                Pointer to the mac
 * - uc_ae (v1.4):           Flag to indicate that authentication and encryption is requested
 */
static void _cl_432_join_ind_cb(uint8_t *puc_device_id, uint8_t uc_device_id_len, uint16_t us_dst_address, uint8_t *puc_mac, uint8_t uc_ae)
{
	uint8_t *puc_buf;
	uint8_t uc_tmp;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_432_dl_join_ind_cmd;
	*puc_buf++ = uc_device_id_len;
	for (uc_tmp = 0; uc_tmp < uc_device_id_len; uc_tmp++) {
		*puc_buf++ = *puc_device_id++;
	}

	*puc_buf++ = (uint8_t)(us_dst_address >> 8);
	*puc_buf++ = (uint8_t)(us_dst_address);
	for (uc_tmp = 0; uc_tmp < 8; uc_tmp++) {
		*puc_buf++ = *puc_mac++;
	}

	*puc_buf++ = uc_ae;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * CL 432 Leave indication
 * - us_dst_address:         Destination 432 Address
 */
static void _cl_432_leave_ind_cb(uint16_t us_dst_address)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_432_dl_leave_ind_cmd;
	*puc_buf++ = (uint8_t)(us_dst_address >> 8);
	*puc_buf++ = (uint8_t)(us_dst_address);

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}
#endif

#if USED_SSCS == SSCS_IPV6
#ifndef __PRIME_GATEWAY__
/**
 * CL IPv6 Data Indication
 *
 * - puc_ipv6_pdu:		PDU received through IPv6
 * - us_pdu_len:                Length of the data
 */
static void _cl_ipv6_data_ind_cb(uint8_t *puc_ipv6_pdu, uint16_t us_pdu_len)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_ipv6_data_ind_cmd;
	*puc_buf++ = (uint8_t)(us_pdu_len >> 8);
	*puc_buf++ = (uint8_t)(us_pdu_len);
	memcpy(puc_buf, puc_ipv6_pdu, us_pdu_len);
	puc_buf += us_pdu_len;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);

	/* Rx data indication */
	uc_rxdata_ind = true;
}

/**
 * CL IPv6 Data Confirm
 *
 * - puc_ipv6_pdu:		PDU sent
 * - uc_result:			Result of the transmission
 */
static void _cl_ipv6_data_cfm_cb(uint8_t *puc_ipv6_pdu, cl_ipv6_result_t uc_result)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_ipv6_data_cfm_cmd;
	*puc_buf++ = (uint8_t)((uint32_t)puc_ipv6_pdu >> 24);
	*puc_buf++ = (uint8_t)((uint32_t)puc_ipv6_pdu >> 16);
	*puc_buf++ = (uint8_t)((uint32_t)puc_ipv6_pdu >> 8);
	*puc_buf++ = (uint8_t)((uint32_t)puc_ipv6_pdu);
	*puc_buf++ = (uint8_t)uc_result;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * CL IPv6 MUL Join Confirm
 *
 * - puc_ipv6:			IPv6 multicast group joined
 * - uc_result:			Reason of the join
 * - uc_ae:                     Flag to indicate that authentication and encryption is requested
 */
static void _cl_ipv6_mul_join_cfm_cb(uint8_t *puc_ipv6, cl_ipv6_result_t uc_result, uint8_t uc_ae)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_ipv6_mul_join_cfm_cmd;
	memcpy(puc_buf, puc_ipv6, 16);
	puc_buf += 16;
	*puc_buf++ = (uint8_t)uc_result;
	*puc_buf++ = (uint8_t)uc_ae;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * CL IPv6 MUL Leave Confirm
 *
 * - puc_ipv6:			IPv6 multicast group left
 * - uc_result:			Reason of the leave
 */
static void _cl_ipv6_mul_leave_cfm_cb(uint8_t *puc_ipv6, cl_ipv6_result_t uc_result)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_cl_ipv6_mul_leave_cfm_cmd;
	memcpy(puc_buf, puc_ipv6, 16);
	puc_buf += 16;
	*puc_buf++ = (uint8_t)uc_result;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}
#endif
#endif

/**
 * Base Management FUP Acknowledgement
 * - uc_cmd:             Command being acknowledged
 * - x_ack_code:         ACK returned code
 * - us_extra_info:      Extra information
 */
static void _bmng_fup_ack_cb(uint8_t uc_cmd, fup_ack_code_t x_ack_code, uint16_t us_extra_info)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_bmng_fup_ack_cmd;
	*puc_buf++ = uc_cmd;
	*puc_buf++ = x_ack_code;
	*puc_buf++ = (uint8_t)(us_extra_info >> 8);
	*puc_buf++ = (uint8_t)(us_extra_info);

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * Base Management FUP Status Indication
 * - x_fup_node_state:   Node upgrade state
 * - ul_pages:           Pages received by the SN according to the BN
 * - puc_eui48:          MAC address of the node
 */
static void _bmng_fup_status_ind_cb(fup_node_state_t x_fup_node_state, uint16_t ul_pages, uint8_t *puc_eui48)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_bmng_fup_status_ind_cmd;
	*puc_buf++ = x_fup_node_state;
	*puc_buf++ = (uint8_t)(ul_pages >> 8);
	*puc_buf++ = (uint8_t)(ul_pages);
	memcpy(puc_buf, puc_eui48, 6);
	puc_buf += 6;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * Base Management FUP Error Indication
 * - x_error_code:       Error code
 * - puc_eui48:          MAC address of the node
 */
static void _bmng_fup_error_ind_cb(fup_error_code_t x_error_code, uint8_t *puc_eui48)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_bmng_fup_error_ind_cmd;
	*puc_buf++ = x_error_code;
	memcpy(puc_buf, puc_eui48, 6);
	puc_buf += 6;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * Base Management FUP Version Indication
 * - puc_eui48:          MAC address of the node
 * - uc_vendor_len:      Vendor ID length
 * - pch_vendor:         Vendor ID
 * - uc_model_len:       Model length
 * - pch_model:          Model
 * - uc_version_len:     Version length
 * - pch_version:        Version
 */
static void _bmng_fup_version_ind_cb(uint8_t *puc_eui48, uint8_t uc_vendor_len, char *pch_vendor, uint8_t uc_model_len, char *pch_model,
		uint8_t uc_version_len, char *pch_version)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_bmng_fup_version_ind_cmd;
	memcpy(puc_buf, puc_eui48, 6);
	puc_buf += 6;
	*puc_buf++ = uc_vendor_len;
	memcpy(puc_buf, pch_vendor, uc_vendor_len);
	puc_buf += uc_vendor_len;
	*puc_buf++ = uc_model_len;
	memcpy(puc_buf, pch_model, uc_model_len);
	puc_buf += uc_model_len;
	*puc_buf++ = uc_version_len;
	memcpy(puc_buf, pch_version, uc_version_len);
	puc_buf += uc_version_len;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * Base Management FUP Kill Indication
 * - puc_eui48:          MAC address of the node
 */
static void _bmng_fup_kill_ind_cb(uint8_t *puc_eui48)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_bmng_fup_kill_ind_cmd;
	memcpy(puc_buf, puc_eui48, 6);
	puc_buf += 6;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * Base Management Network Event Indication
 * - px_net_event:          Network event information
 */
static void _bmng_net_event_ind_cb(bmng_net_event_t *px_net_event)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_bmng_network_event_cmd;
	*puc_buf++ = px_net_event->net_event;
	memcpy(puc_buf, px_net_event->puc_eui48, 6);
	puc_buf += 6;
	*puc_buf++ = px_net_event->sid;
	*puc_buf++ = (uint8_t)(px_net_event->lnid >> 8);
	*puc_buf++ = (uint8_t)(px_net_event->lnid);
	*puc_buf++ = px_net_event->lsid;
	*puc_buf++ = px_net_event->alv_rx_cnt;
	*puc_buf++ = px_net_event->alv_tx_cnt;
	*puc_buf++ = px_net_event->alv_time;
	*puc_buf++ = (uint8_t)(px_net_event->us_pch >> 8);
	*puc_buf++ = (uint8_t)(px_net_event->us_pch);
	*puc_buf++ = (uint8_t)(px_net_event->us_pch_lsid >> 8);
	*puc_buf++ = (uint8_t)(px_net_event->us_pch_lsid);

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

void _mdm_bmng_net_event_ind_cb(void *px_net_event)
{
	_bmng_net_event_ind_cb((bmng_net_event_t *)px_net_event);
}

/**
 * Base Management PRIME Profile Acknowledgement
 * - uc_cmd:             Command being acknowledged
 * - x_ack_code:         ACK returned code
 */
static void _bmng_pprof_ack_cb(uint8_t uc_cmd, pprof_ack_code_t x_ack_code)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_bmng_pprof_ack_cmd;
	*puc_buf++ = uc_cmd;
	*puc_buf++ = x_ack_code;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * Base Management PRIME Profile Get Response
 * - puc_eui48:            MAC address of the node
 * - us_data_length:       Data length
 * - puc_data:             Data
 */
static void _bmng_pprof_get_response_cb(uint8_t *puc_eui48, uint16_t us_data_len, uint8_t *puc_data)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_bmng_pprof_get_response_cmd;
	memcpy(puc_buf, puc_eui48, 6);
	puc_buf += 6;
	*puc_buf++ = (uint8_t)(us_data_len >> 8);
	*puc_buf++ = (uint8_t)(us_data_len);
	memcpy(puc_buf, puc_data, us_data_len);
	puc_buf += us_data_len;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * Base Management PRIME Profile Get Enhanced Response
 * - puc_eui48:            MAC address of the node
 * - us_data_length:       Data length
 * - puc_data:             Data
 */
static void _bmng_pprof_get_enhanced_response_cb(uint8_t *puc_eui48, uint16_t us_data_len, uint8_t *puc_data)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_bmng_pprof_get_enhanced_response_cmd;
	memcpy(puc_buf, puc_eui48, 6);
	puc_buf += 6;
	*puc_buf++ = (uint8_t)(us_data_len >> 8);
	*puc_buf++ = (uint8_t)(us_data_len);
	memcpy(puc_buf, puc_data, us_data_len);
	puc_buf += us_data_len;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * Base Management PRIME Profile Get Zero Crossing Response
 * - puc_eui48:            MAC address of the node
 * - uc_zc_status:         Zero cross status
 * - ul_zc_time:           Zero crossing time
 */
static void _bmng_pprof_get_zc_response_cb(uint8_t *puc_eui48, uint8_t uc_zc_status, uint32_t ul_zc_time)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_bmng_pprof_get_zc_response_cmd;
	memcpy(puc_buf, puc_eui48, 6);
	puc_buf += 6;
	*puc_buf++ = uc_zc_status;
	*puc_buf++ = (uint8_t)(ul_zc_time >> 24);
	*puc_buf++ = (uint8_t)(ul_zc_time >> 16);
	*puc_buf++ = (uint8_t)(ul_zc_time >> 8);
	*puc_buf++ = (uint8_t)(ul_zc_time);

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * Base Management PRIME Profile Zero Crossing Difference between BN and SN Response
 * - puc_eui48:            MAC address of the node
 * - ul_time_freq:         Frequency
 * - ul_time_diff:         Zero crossing difference (in 10s of microsec)
 */
static void _bmng_pprof_zc_diff_response_cb(uint8_t *puc_eui48, uint32_t ul_time_freq, uint32_t ul_time_diff)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_bmng_pprof_zc_diff_response_cmd;
	memcpy(puc_buf, puc_eui48, 6);
	puc_buf += 6;
	*puc_buf++ = (uint8_t)(ul_time_freq >> 24);
	*puc_buf++ = (uint8_t)(ul_time_freq >> 16);
	*puc_buf++ = (uint8_t)(ul_time_freq >> 8);
	*puc_buf++ = (uint8_t)(ul_time_freq);
	*puc_buf++ = (uint8_t)(ul_time_diff >> 24);
	*puc_buf++ = (uint8_t)(ul_time_diff >> 16);
	*puc_buf++ = (uint8_t)(ul_time_diff >> 8);
	*puc_buf++ = (uint8_t)(ul_time_diff);

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * Base Management Whitelist Acknowledgement
 * - uc_cmd:             Command being acknowledged
 * - x_ack_code:         ACK returned code
 */
static void _bmng_whitelist_ack_cb(uint8_t uc_cmd, whitelist_ack_code_t x_ack_code)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_bmng_whitelist_ack_cmd;
	*puc_buf++ = uc_cmd;
	*puc_buf++ = x_ack_code;

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * USI handler for incoming serial messages
 * - puc_rx_msg:      Pointer to received message
 * - us_len:          Data len of the received message
 */
static bool _usi_prime_api_handler(uint8_t *puc_rx_msg, uint16_t us_len)
{
	if (!sx_msg_rcv[suc_input_msg_rcv].us_len) {
		if (us_len < MAX_LENGTH_BUFF) {
			memcpy(sx_msg_rcv[suc_input_msg_rcv].data_buf, puc_rx_msg, us_len);
			sx_msg_rcv[suc_input_msg_rcv].us_len = us_len;

			if (++suc_input_msg_rcv == MAX_NUM_MSG_RCV) {
				suc_input_msg_rcv = 0;
			}

			return true;
		} else {
			hal_debug_report(MODEM_ERR_MSG_TOO_BIG); //printf("ERROR ,Message too big\r\n");
		}
	} else {
		hal_debug_report(MODEM_ERR_QUEUE_FULL);//printf("ERROR ,RX queue full\r\n");
	}

	return false;
}

/**
 * \brief Set callback functions
 *
 */
static void _modem_set_callbacks(void)
{
	mac_callbacks_t mac_callbacks;
#if USED_SSCS == SSCS_432
	cl_432_callbacks_t cl432_callbacks;
#endif
#if USED_SSCS == SSCS_IPV6
	cl_ipv6_callbacks_t clipv6_callbacks;
#endif
	bmng_callbacks_t bmng_callbacks;

	mac_callbacks.mac_data_cfm_cb = _data_cfm_cb;
	mac_callbacks.mac_data_ind_cb = _data_ind_cb;
	mac_callbacks.mac_establish_cfm_cb = _establish_cfm_cb;
	mac_callbacks.mac_establish_ind_cb = _establish_ind_cb;
	mac_callbacks.mac_join_cfm_cb = _join_cfm_cb;
	mac_callbacks.mac_join_ind_cb = _join_ind_cb;
	mac_callbacks.mac_leave_cfm_cb = _leave_cfm_cb;
	mac_callbacks.mac_leave_ind_cb = _leave_ind_cb;
	mac_callbacks.mac_release_cfm_cb = _release_cfm_cb;
	mac_callbacks.mac_release_ind_cb = _release_ind_cb;

	mac_callbacks.mlme_get_cfm_cb = _mlme_get_cfm_cb;
	mac_callbacks.mlme_list_get_cfm_cb = _mlme_list_get_cfm_cb;
	mac_callbacks.mlme_promote_cfm_cb = _mlme_promote_cfm_cb;
	mac_callbacks.mlme_mp_promote_cfm_cb = _mlme_mp_promote_cfm_cb;
	mac_callbacks.mlme_reset_cfm_cb = _mlme_reset_cfm_cb;
	mac_callbacks.mlme_set_cfm_cb = _mlme_set_cfm_cb;

	mac_callbacks.plme_get_cfm_cb = _plme_get_cfm_cb;
	mac_callbacks.plme_reset_cfm_cb = _plme_reset_cfm_cb;
	mac_callbacks.plme_resume_cfm_cb = _plme_resume_cfm_cb;
	mac_callbacks.plme_set_cfm_cb = _plme_set_cfm_cb;
	mac_callbacks.plme_sleep_cfm_cb = _plme_sleep_cfm_cb;
	mac_callbacks.plme_testmode_cfm_cb = NULL;

	prime_cl_null_set_callbacks(&mac_callbacks);

#if USED_SSCS == SSCS_432
	cl432_callbacks.cl_432_dl_data_cfm_cb = _cl_432_dl_data_cfm_cb;
	cl432_callbacks.cl_432_dl_data_ind_cb = _cl_432_dl_data_ind_cb;
	cl432_callbacks.cl_432_join_ind_cb = _cl_432_join_ind_cb;
	cl432_callbacks.cl_432_leave_ind_cb = _cl_432_leave_ind_cb;

	prime_cl_432_set_callbacks(&cl432_callbacks);
#endif
#if USED_SSCS == SSCS_IPV6
#ifndef __PRIME_GATEWAY__
	clipv6_callbacks.cl_ipv6_data_cfm_cb = _cl_ipv6_data_cfm_cb;
	clipv6_callbacks.cl_ipv6_data_ind_cb = _cl_ipv6_data_ind_cb;
	clipv6_callbacks.cl_ipv6_mul_join_cfm_cb = _cl_ipv6_mul_join_cfm_cb;
	clipv6_callbacks.cl_ipv6_mul_leave_cfm_cb = _cl_ipv6_mul_leave_cfm_cb;
#else
	clipv6_callbacks.cl_ipv6_data_ind_cb = gw_ipv6_data_ind_cb;
#endif

	prime_cl_ipv6_set_callbacks(&clipv6_callbacks);
#endif

	bmng_callbacks.fup_ack_cb = _bmng_fup_ack_cb;
	bmng_callbacks.fup_error_ind_cb = _bmng_fup_error_ind_cb;
	bmng_callbacks.fup_kill_ind_cb = _bmng_fup_kill_ind_cb;
	bmng_callbacks.fup_status_ind_cb = _bmng_fup_status_ind_cb;
	bmng_callbacks.fup_version_ind_cb = _bmng_fup_version_ind_cb;
	bmng_callbacks.network_event_ind_cb = _bmng_net_event_ind_cb;
	bmng_callbacks.pprof_ack_cb = _bmng_pprof_ack_cb;
	bmng_callbacks.pprof_get_response_cb = _bmng_pprof_get_response_cb;
	bmng_callbacks.pprof_get_enhanced_response_cb = _bmng_pprof_get_enhanced_response_cb;
	bmng_callbacks.pprof_get_zc_response_cb = _bmng_pprof_get_zc_response_cb;
	bmng_callbacks.pprof_zc_diff_response_cb = _bmng_pprof_zc_diff_response_cb;
	bmng_callbacks.whitelist_ack_cb = _bmng_whitelist_ack_cb;

	prime_bmng_set_callbacks(&bmng_callbacks);
}

/**
 * \brief Initialize MODEM Application
 *
 */
void modem_init(void)
{
	uint8_t uc_i;

#ifdef __PRIME_GATEWAY__
	gw_ipv6_network_init();
#endif

	/* Initialize the reception queue */
	suc_input_msg_rcv = 0;
	suc_output_msg_rcv = 0;
	for (uc_i = 0; uc_i < MAX_NUM_MSG_RCV; uc_i++) {
		sx_msg_rcv[uc_i].us_len = 0;
	}


	/* Set callback functions */
	_modem_set_callbacks();

	/* Configure USI protocol handler */
	hal_usi_set_callback(PROTOCOL_PRIME_API, _usi_prime_api_handler, MODEM_USI_PORT);
	x_usi_msg.uc_protocol_type = PROTOCOL_PRIME_API;
	x_usi_msg.ptr_buf = suc_serial_buf;

	/* Configure PRIME region to store SN firmware */
	sx_prime_reg.ul_address = PRIME_FLASH_LOCATION_SN;
	sx_prime_reg.ul_size = PRIME_MAX_SIZE_SN;
	sx_prime_reg.in_use = false;
	hal_fu_config_regions(1, &sx_prime_reg);

#ifdef MODEM_DEBUG_ENABLE
	printf("MODEM Application: Base Node...\r\n");
#endif

	/* Initialize TxRx data indicators */
	uc_rxdata_ind = false;
	uc_txdata_ind = false;
}

/**
 * MAC Establish Request
 * - puc_eui48:     Address of the node to which this connection will be addressed
 * - uc_type:       Convergence Layer type of the connection
 * - puc_data:      Data associated with the connection establishment procedure
 * - us_data_len:   Length of the data in bytes
 * - uc_arq:        Flag to indicate whether or not the ARQ mechanism should be used for this connection
 * - uc_cfbytes:    Flag to indicate whether or not the connection should use the contention or contention-free channel access scheme
 * - uc_ae (v1.4):  Flag to indicate that authentication and encryption is requested
 */
static void _establish_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint8_t puc_eui48[6];
	uint8_t uc_type;
	uint16_t us_data_len;
	uint8_t *puc_data;
	uint8_t uc_arq;
	uint8_t uc_cfbytes;
	uint8_t uc_ae;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	memcpy(puc_eui48, puc_msg, 6);
	puc_msg += 6;
	uc_type = *puc_msg++;
	us_data_len = ((uint16_t)(*puc_msg++)) << 8;
	us_data_len += *puc_msg++;
	puc_data = puc_msg;
	puc_msg += us_data_len;
	uc_arq = *puc_msg++;
	uc_cfbytes = *puc_msg++;
	uc_ae = *puc_msg++;

	prime_cl_null_establish_request(puc_eui48, uc_type, puc_data, us_data_len, uc_arq, uc_cfbytes, uc_ae);
}

/**
 * MAC Establish Response
 * - us_con_handle: Unique identifier of the connection
 * - uc_answer:     Action to be taken for this connection establishment
 * - puc_data:      Data associated with the connection establishment procedure
 * - us_data_len:   Length of the data in bytes
 * - uc_ae (v1.4):  Flag to indicate that authentication and encryption is requested
 */
static void _establish_response_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint16_t us_con_handle;
	uint8_t uc_answer;
	uint16_t us_data_len;
	uint8_t *puc_data;
	uint8_t uc_ae;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	us_con_handle = ((uint16_t)(*puc_msg++)) << 8;
	us_con_handle += *puc_msg++;
	uc_answer = *puc_msg++;
	us_data_len = ((uint16_t)(*puc_msg++)) << 8;
	us_data_len += *puc_msg++;
	puc_data = puc_msg;
	puc_msg += us_data_len;
	uc_ae = *puc_msg++;

	prime_cl_null_establish_response(us_con_handle, (mac_establish_response_answer_t)uc_answer, puc_data, us_data_len, uc_ae);
}

/**
 * MAC Redirect Response
 * - puc_eui48:     Address of the node to which this connection will be "redirected"
 * - us_con_handle: Unique identifier of the connection
 * - puc_data:      Data associated with the connection establishment procedure
 * - us_data_len:   Length of the data in bytes
 */
static void _redirect_response_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint16_t us_con_handle;
	uint8_t puc_eui48[6];
	uint8_t *puc_data;
	uint16_t us_data_len;

	puc_msg = puc_rcv_msg;
	us_con_handle = ((uint16_t)(*puc_msg++)) << 8;
	us_con_handle += *puc_msg++;
	memcpy(puc_eui48, puc_msg, 6);
	us_data_len = ((uint16_t)(*puc_msg++)) << 8;
	us_data_len += *puc_msg++;
	puc_data = puc_msg;

	prime_cl_null_redirect_response(us_con_handle, puc_eui48, puc_data, us_data_len);
}

/**
 * MAC Release Request
 * - us_con_handle: Unique identifier of the connection
 */
static void _release_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint16_t us_con_handle;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	us_con_handle = ((uint16_t)(*puc_msg++)) << 8;
	us_con_handle += *puc_msg++;

	prime_cl_null_release_request(us_con_handle);
}

/**
 * MAC Release Response
 * - us_con_handle: Unique identifier of the connection
 * - uc_answer:     Action to be taken for this connection release procedure
 */
static void _release_response_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint16_t us_con_handle;
	uint8_t uc_answer;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	us_con_handle = ((uint16_t)(*puc_msg++)) << 8;
	us_con_handle += *puc_msg++;
	uc_answer = *puc_msg++;

	prime_cl_null_release_response(us_con_handle, (mac_release_response_answer_t)uc_answer);
}

/**
 * MAC Join Request
 * - us_broadcast:      Join type (broadcast or multicast connection)
 * - us_con_handle:     Unique identifier of the connection (only used for base node)
 * - puc_eui48:         Address of the node to which this join is being requested (only used for base node)
 * - uc_con_type:       Connection type
 * - puc_data:          Data associated with the join request procedure
 * - us_data_len:       Length of the data in bytes
 * - uc_ae (v1.4):      Flag to indicate that authentication and encryption is requested
 */
static void _join_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint8_t uc_broadcast;
	uint8_t uc_con_type;
	uint16_t us_data_len;
	uint16_t us_con_handle;
	uint8_t puc_data[256];
	uint8_t puc_eui48[6];
	uint8_t uc_ae;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	uc_broadcast = *puc_msg++;
	us_con_handle = ((uint16_t)(*puc_msg++)) << 8;
	us_con_handle += *puc_msg++;
	memcpy(puc_eui48, puc_msg, 6);
	puc_msg += 6;
	uc_con_type = *puc_msg++;
	us_data_len = ((uint16_t)(*puc_msg++)) << 8;
	us_data_len += *puc_msg++;
	memcpy(puc_data, puc_msg, us_data_len);
	puc_msg += us_data_len;
	uc_ae = *puc_msg++;

	prime_cl_null_join_request((mac_join_mode_t)uc_broadcast, us_con_handle, puc_eui48, uc_con_type, puc_data, us_data_len, uc_ae);
}

/**
 * MAC Join Response
 * - us_con_handle:        Unique identifier of the connection
 * - puc_eui48:            Address of the node which requested the multicast group join (only valid for base node)
 * - uc_answer:            Action to be taken for this join request
 * - uc_ae (v1.4):         Flag to indicate that authentication and encryption is requested
 */
static void _join_response_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint16_t us_con_handle;
	uint8_t uc_answer;
	uint8_t puc_eui48[6];
	uint8_t uc_ae;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	us_con_handle = ((uint16_t)(*puc_msg++)) << 8;
	us_con_handle += *puc_msg++;
	memcpy(puc_eui48, puc_msg, 6);
	puc_msg += 6;
	uc_answer = *puc_msg++;
	uc_ae = *puc_msg++;

	prime_cl_null_join_response(us_con_handle, puc_eui48, (mac_join_response_answer_t)uc_answer, uc_ae);
}

/*
 * MAC Leave Request
 * - us_con_handle:        Unique identifier of the connection
 * - puc_eui48:            Address of the node to be removed from the multicast group (only valid for base node)
 */
static void _leave_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint16_t us_con_handle;
	uint8_t puc_eui48[6];

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	us_con_handle = ((uint16_t)(*puc_msg++)) << 8;
	us_con_handle += *puc_msg++;
	memcpy(puc_eui48, puc_msg, 6);

	prime_cl_null_leave_request(us_con_handle, puc_eui48);
}

/**
 * MAC Data Request
 * - us_con_handle:           Unique identifier of the connection
 * - puc_data:                Pointer to data to be transmitted through this connection
 * - us_data_len:             Length of the data in bytes
 * - uc_prio:                 Priority of the data to be sent when using the CSMA access scheme
 * - ul_time_ref (v1.4):      Time reference (in 10s of microsec)
 */
static void _data_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint16_t us_con_handle;
	uint16_t us_data_len;
	uint8_t *puc_data;
	uint8_t uc_prio;
	uint32_t ul_time_ref;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	us_con_handle = ((uint16_t)(*puc_msg++)) << 8;
	us_con_handle += *puc_msg++;
	us_data_len = ((uint16_t)(*puc_msg++)) << 8;
	us_data_len += *puc_msg++;
	puc_data = puc_msg;
	puc_msg += us_data_len;
	uc_prio = *puc_msg++;
	ul_time_ref = ((uint32_t)(*puc_msg++)) << 24;
	ul_time_ref += ((uint32_t)(*puc_msg++)) << 16;
	ul_time_ref += ((uint32_t)(*puc_msg++)) << 8;
	ul_time_ref += *puc_msg++;

	prime_cl_null_data_request(us_con_handle, puc_data, us_data_len, uc_prio, ul_time_ref);

	/* Tx data indication */
	uc_txdata_ind = true;
}

/**
 * PLME Reset Request
 * - us_pch:             Physical channel
 */
static void _plme_reset_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint16_t us_pch;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	us_pch = ((uint16_t)(*puc_msg++)) << 8;
	us_pch += *puc_msg++;

	prime_cl_null_plme_reset_request(us_pch);
}

/**
 * PLME Sleep Request
 * - us_pch:             Physical channel
 */
static void _plme_sleep_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint16_t us_pch;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	us_pch = ((uint16_t)(*puc_msg++)) << 8;
	us_pch += *puc_msg++;

	prime_cl_null_plme_sleep_request(us_pch);
}

/**
 * PLME Resume Request
 * - us_pch:             Physical channel
 */
static void _plme_resume_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint16_t us_pch;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	us_pch = ((uint16_t)(*puc_msg++)) << 8;
	us_pch += *puc_msg++;

	prime_cl_null_plme_resume_request(us_pch);
}

/**
 * PLME Get Request
 * - us_pib_attrib:      PIB attribute
 * - us_pch (v1.4):      Physical channel
 */
static void _plme_get_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint16_t us_pib_attrib;
	uint16_t us_pch;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	us_pib_attrib = ((uint16_t)(*puc_msg++)) << 8;
	us_pib_attrib += *puc_msg++;
	us_pch = ((uint16_t)(*puc_msg++)) << 8;
	us_pch += *puc_msg++;

	prime_cl_null_plme_get_request(us_pib_attrib, us_pch);
}

/**
 * PLME Set Request
 * - us_pib_attrib:     PIB attribute
 * - pv_pib_value:      PIB attribute value
 * - uc_pib_size:       PIB attribute value size
 * - us_pch (v1.4):     Physical channel
 */
static void _plme_set_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint16_t us_pib_attrib;
	uint8_t puc_pib_value[256];
	uint8_t uc_pib_size;
	void *pv_pib_value;
	uint16_t us_tmp;
	uint32_t ul_tmp;
	uint16_t us_pch;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	us_pib_attrib = ((uint16_t)(*puc_msg++)) << 8;
	us_pib_attrib += *puc_msg++;
	uc_pib_size = *puc_msg++;
	/* Check PIB size */
	switch (uc_pib_size) {
	case 2: /* sizeof(uint16_t) */
		/* Extract PIB value */
		us_tmp = ((uint16_t)(*puc_msg++)) << 8;
		us_tmp += *puc_msg++;
		pv_pib_value = (void *)(&us_tmp);
		break;

	case 4: /* sizeof(uint32_t) */
		/* Extract PIB value */
		ul_tmp = ((uint32_t)(*puc_msg++) << 24);
		ul_tmp += ((uint32_t)(*puc_msg++) << 16);
		ul_tmp += ((uint32_t)(*puc_msg++) << 8);
		ul_tmp += (uint32_t)*puc_msg++;
		pv_pib_value = (void *)(&ul_tmp);
		break;

	case 1: /* sizeof(uint8_t) */
	default: /* arrays */
		memcpy(puc_pib_value, puc_msg, uc_pib_size);
		pv_pib_value = (void *)puc_pib_value;
		break;
	}

	us_pch = ((uint16_t)(*puc_msg++)) << 8;
	us_pch += *puc_msg++;

	prime_cl_null_plme_set_request(us_pib_attrib, pv_pib_value, uc_pib_size, us_pch);
}

/**
 * MLME Promote Request
 * - puc_eui48:            Address of the node to be promoted (NULL in Service Node)
 * - uc_bcn_mode (v1.4):   Beacon mode
 */
static void _mlme_promote_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint8_t puc_eui48[6];
	uint8_t uc_bcn_mode;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	memcpy(puc_eui48, puc_msg, 6);
	puc_msg += 6;
	uc_bcn_mode = *puc_msg++;

	prime_cl_null_mlme_promote_request(puc_eui48, uc_bcn_mode);
}

/**
 * MLME MultiPHY Promote Request  (v1.4)
 * - puc_eui48:            Address of the node to be promoted (NULL in Service Node)
 * - uc_bcn_mode:          Beacon mode
 * - us_pch:               Physical channel
 */
static void _mlme_mp_promote_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint8_t puc_eui48[6];
	uint16_t us_pch;
	uint8_t uc_bcn_mode;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	memcpy(puc_eui48, puc_msg, 6);
	puc_msg += 6;
	uc_bcn_mode = *puc_msg++;
	us_pch = ((uint16_t)(*puc_msg++)) << 8;
	us_pch += *puc_msg++;

	prime_cl_null_mlme_mp_promote_request(puc_eui48, uc_bcn_mode, us_pch);
}

/**
 * MLME Get Request
 * - us_pib_attrib:      PIB attribute
 */
static void _mlme_get_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint16_t us_pib_attrib;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	us_pib_attrib = ((uint16_t)(*puc_msg++)) << 8;
	us_pib_attrib += *puc_msg++;

	prime_cl_null_mlme_get_request(us_pib_attrib);
}

/**
 * MLME List Get Request
 * - us_pib_attrib:      PIB attribute
 */
static void _mlme_list_get_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint16_t us_pib_attrib;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	us_pib_attrib = ((uint16_t)(*puc_msg++)) << 8;
	us_pib_attrib += *puc_msg++;

	prime_cl_null_mlme_list_get_request(us_pib_attrib);
}

/**
 * MLME Set Request
 * - us_pib_attrib      PIB attribute
 * - pv_pib_value       PIB attribute value
 * - uc_pib_size        PIB attribute value size
 */
static void _mlme_set_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	void *pv_pib_value;
	uint16_t us_pib_attrib;
	uint8_t puc_pib_value[256];
	uint8_t uc_pib_size;
	uint16_t us_tmp;
	uint32_t ul_tmp;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	us_pib_attrib = ((uint16_t)(*puc_msg++)) << 8;
	us_pib_attrib += *puc_msg++;
	uc_pib_size = *puc_msg++;
	/* Check PIB size */
	switch (uc_pib_size) {
	case 2: /* sizeof(uint16_t) */
		/* Extract PIB value */
		us_tmp = ((uint16_t)(*puc_msg++)) << 8;
		us_tmp += *puc_msg++;
		pv_pib_value = (void *)(&us_tmp);
		break;

	case 4: /* sizeof(uint32_t) */
		/* Extract PIB value */
		ul_tmp = ((uint32_t)(*puc_msg++)) << 24;
		ul_tmp += ((uint32_t)(*puc_msg++)) << 16;
		ul_tmp += ((uint32_t)(*puc_msg++)) << 8;
		ul_tmp += *puc_msg++;
		pv_pib_value = (void *)(&ul_tmp);
		break;

	case 1: /* sizeof(uint8_t) */
	default: /* arrays */
		memcpy(puc_pib_value, puc_msg, uc_pib_size);
		pv_pib_value = (void *)puc_pib_value;
		break;
	}

	prime_cl_null_mlme_set_request(us_pib_attrib, pv_pib_value, uc_pib_size);
}

#if USED_SSCS == SSCS_432
/**
 * CL 432 Release request
 * - us_dst_address:   Address to disconnect
 */
static void _432_release_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint16_t us_dst_address;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	us_dst_address = ((uint16_t)(*puc_msg++)) << 8;
	us_dst_address += *puc_msg++;

	prime_cl_432_release_request(us_dst_address);
}

/**
 * CL 432 Data request
 * - uc_dst_lsap:      Destination LSAP
 * - uc_src_lsap:      Source LSAP
 * - us_dst_address:   Destination 432 Address
 * - px_buff:          Pointer to the data buffer
 * - us_lsdu_len:      Length of the data
 * - uc_link_class:    Link class (non used)
 *
 */
static void _432_dl_data_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint16_t us_dst_address, us_lsdu_len;
	dl_432_buffer_t x_buff_432;
	uint8_t uc_link_class, uc_dst_lsap, uc_src_lsap;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	uc_dst_lsap = *puc_msg++;
	uc_src_lsap = *puc_msg++;
	us_dst_address = ((uint16_t)(*puc_msg++)) << 8;
	us_dst_address += *puc_msg++;
	us_lsdu_len = ((uint16_t)(*puc_msg++)) << 8;
	us_lsdu_len += *puc_msg++;
	if (us_lsdu_len <= MAX_LENGTH_432_DATA) {
		memcpy(x_buff_432.dl.buff, puc_msg, us_lsdu_len);
		puc_msg += us_lsdu_len;
		uc_link_class = *puc_msg++;

		prime_cl_432_dl_data_request(uc_dst_lsap, uc_src_lsap, us_dst_address, &x_buff_432, us_lsdu_len, uc_link_class);
	}

	/* Tx data indication */
	uc_txdata_ind = true;
}
#endif

#if USED_SSCS == SSCS_IPV6
#ifndef __PRIME_GATEWAY__
/**
 * CL IPv6 Data Request
 *
 * - puc_ipv6_pdu:		PDU to be sent through ipv6
 * - us_pdu_len:                Length of the data
 */
static void _ipv6_data_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint16_t us_pdu_len;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	us_pdu_len = ((uint16_t)(*puc_msg++)) << 8;
	us_pdu_len += *puc_msg++;
	if (us_pdu_len <= MAX_LENGTH_IPV6_DATA) {
		memcpy(spuc_data_buf, puc_msg, us_pdu_len);
		puc_msg += us_pdu_len;

		prime_cl_ipv6_data_request(spuc_data_buf, us_pdu_len);
	}

	/* Tx data indication */
	uc_txdata_ind = true;
}

/**
 * CL IPv6 MUL Join Request
 *
 * - puc_ipv6:			IPv6 multicast group to join
 * - uc_ae:                     Flag to indicate that authentication and encryption is requested
 */
static void _ipv6_mul_join_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint8_t puc_ipv6[16];
	uint8_t uc_ae;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	memcpy(puc_ipv6, puc_msg, 16);
	puc_msg += 16;
	uc_ae = *puc_msg++;

	prime_cl_ipv6_mul_join_request(puc_ipv6, uc_ae);
}

/**
 * CL IPv6 MUL Leave Request
 *
 * - puc_ipv6:			IPv6 multicast group to be left
 */
static void _ipv6_mul_leave_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint8_t puc_ipv6[16];

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	memcpy(puc_ipv6, puc_msg, 16);

	prime_cl_ipv6_mul_leave_request(puc_ipv6);
}
#endif
#endif

/**
 * Base Management FUP Add Target Request
 * - uc_cmd:             Command to acknowledge
 * - puc_eui48:          MAC address of the node to be added
 */
static void _bmng_fup_add_target_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint8_t puc_eui48[6];

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	memcpy(puc_eui48, puc_msg, 6);

	prime_bmng_fup_add_target_request(prime_bmng_fup_add_target_request_cmd, puc_eui48);
}

/**
 * Base Management FUP Set Firmware Data Request
 * - uc_cmd:             Command to acknowledge
 * - uc_vendor_len:      Vendor ID length
 * - pch_vendor:         Vendor ID
 * - uc_model_len:       Model length
 * - pch_model:          Model
 * - uc_version_len:     Version length
 * - pch_version         Version
 */
static void _bmng_fup_set_fw_data_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	char pch_vendor[32], pch_model[32], pch_version[32];
	uint8_t uc_vendor_len, uc_model_len, uc_version_len;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	uc_vendor_len = *puc_msg++;
	memcpy(pch_vendor, puc_msg, uc_vendor_len);
	puc_msg += uc_vendor_len;
	uc_model_len = *puc_msg++;
	memcpy(pch_model, puc_msg, uc_model_len);
	puc_msg += uc_model_len;
	uc_version_len = *puc_msg++;
	memcpy(pch_version, puc_msg, uc_version_len);

	prime_bmng_fup_set_fw_data_request(prime_bmng_fup_set_fw_data_request_cmd, uc_vendor_len, pch_vendor, uc_model_len, pch_model, uc_version_len,
			pch_version);
}

/**
 * Base Management FUP Set Upgrade Options Request
 * - uc_cmd:             Command to acknowledge
 * - uc_arq_en:          Enable/Disable ARQ protocol (1/0)
 * - x_page_size:        Page size
 * - uc_multicast_en:    Enable/Disable multicast (1/0)
 * - ul_delay_restart:   Delay restart time in seconds
 * - ul_safety_timer:    Safety timer in seconds
 */
static void _bmng_fup_set_upg_options_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint32_t ul_delay_restart;
	uint32_t ul_safety_timer;
	fup_page_size_t x_page_size;
	uint8_t uc_arq_en;
	uint8_t uc_multicast_en;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	uc_arq_en = *puc_msg++;
	x_page_size = (fup_page_size_t)*puc_msg++;
	uc_multicast_en = *puc_msg++;
	ul_delay_restart = ((uint32_t)(*puc_msg++)) << 24;
	ul_delay_restart += ((uint32_t)(*puc_msg++)) << 16;
	ul_delay_restart += ((uint32_t)(*puc_msg++)) << 8;
	ul_delay_restart += *puc_msg++;
	ul_safety_timer = ((uint32_t)(*puc_msg++)) << 24;
	ul_safety_timer += ((uint32_t)(*puc_msg++)) << 16;
	ul_safety_timer += ((uint32_t)(*puc_msg++)) << 8;
	ul_safety_timer += *puc_msg++;

	prime_bmng_fup_set_upg_options_request(prime_bmng_fup_set_upg_options_request_cmd, uc_arq_en, x_page_size, uc_multicast_en, ul_delay_restart,
			ul_safety_timer);
}

/**
 * Base Management FUP Init File Transmission Request
 * - uc_cmd:             Command to acknowledge
 * - us_frame_number:    Frame number (0x0000)
 * - ul_file_size:       File size
 * - us_frame_size:      Frame size
 * - ul_crc:             File CRC-32
 */
static void _bmng_fup_init_file_tx_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint32_t ul_file_size, ul_crc;
	uint16_t us_frame_number, us_frame_size;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	us_frame_number = ((uint16_t)(*puc_msg++)) << 8;
	us_frame_number += *puc_msg++;
	ul_file_size = ((uint32_t)(*puc_msg++)) << 24;
	ul_file_size += ((uint32_t)(*puc_msg++)) << 16;
	ul_file_size += ((uint32_t)(*puc_msg++)) << 8;
	ul_file_size += *puc_msg++;
	us_frame_size = ((uint16_t)(*puc_msg++)) << 8;
	us_frame_size += *puc_msg++;
	ul_crc = ((uint32_t)(*puc_msg++)) << 24;
	ul_crc += ((uint32_t)(*puc_msg++)) << 16;
	ul_crc += ((uint32_t)(*puc_msg++)) << 8;
	ul_crc += *puc_msg++;

	prime_bmng_fup_init_file_tx_request(prime_bmng_fup_init_file_tx_request_cmd, us_frame_number, ul_file_size, us_frame_size, ul_crc);
}

/**
 * Base Management FUP Data Frame Request
 * - uc_cmd:             Command to acknowledge
 * - us_frame_number:    Frame number
 * - us_data_len:        Date length
 * - puc_data:           Data frame
 */
static void _bmng_fup_data_frame_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint8_t puc_data[1000];
	uint16_t us_frame_number, us_data_len;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	us_frame_number = ((uint16_t)(*puc_msg++)) << 8;
	us_frame_number += *puc_msg++;
	us_data_len = ((uint16_t)(*puc_msg++)) << 8;
	us_data_len += *puc_msg++;
	memcpy(puc_data, puc_msg, us_data_len);

	prime_bmng_fup_data_frame_request(prime_bmng_fup_data_frame_request_cmd, us_frame_number, us_data_len, puc_data);
}

/**
 * Base Management FUP Abort FU Request
 * - uc_cmd:             Command to acknowledge
 * - puc_eui48:          MAC address of the node to be added
 * NOTE: If the MAC address is FF:FF:FF:FF:FF:FF, the FU is aborted for all nodes.
 */
static void _bmng_fup_abort_fu_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint8_t puc_eui48[6];

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	memcpy(puc_eui48, puc_msg, 6);

	prime_bmng_fup_abort_fu_request(prime_bmng_fup_abort_fu_request_cmd, puc_eui48);
}

/**
 * Base Management FUP Start FU Request
 * - uc_cmd:             Command to acknowledge
 * - uc_enable:          Enable/Disable FU (1/0)
 */
static void _bmng_fup_start_fu_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint8_t uc_enable;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	uc_enable = *puc_msg++;

	prime_bmng_fup_start_fu_request(prime_bmng_fup_start_fu_request_cmd, uc_enable);
}

/**
 * Base Management FUP Set Match Rule Request
 * - uc_cmd:             Command to acknowledge
 * - uc_rules:           Match rules: 0000 0MV0
 * NOTE: If M and/or V are set, only the nodes matching model and/or vendor will be upgraded.
 */
static void _bmng_fup_set_match_rule_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint8_t uc_rules;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	uc_rules = *puc_msg++;

	prime_bmng_fup_set_match_rule_request(prime_bmng_fup_set_match_rule_request_cmd, uc_rules);
}

/**
 * Base Management FUP Version Request
 * - uc_cmd:             Command to acknowledge
 * - puc_eui48:          MAC address of the node to be added
 * NOTE: If the MAC address is FF:FF:FF:FF:FF:FF, version information is asked to all registered nodes.
 */
static void _bmng_fup_get_version_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint8_t puc_eui48[6];

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	memcpy(puc_eui48, puc_msg, 6);

	prime_bmng_fup_get_version_request(prime_bmng_fup_get_version_request_cmd, puc_eui48);
}

/**
 * Base Management FUP State Request
 * - uc_cmd:             Command to acknowledge
 * - puc_eui48:          MAC address of the node
 * NOTE: If the MAC address is FF:FF:FF:FF:FF:FF, state information will be asked to all registered nodes.
 */
static void _bmng_fup_get_state_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint8_t puc_eui48[6];

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	memcpy(puc_eui48, puc_msg, 6);

	prime_bmng_fup_get_state_request(prime_bmng_fup_get_state_request_cmd, puc_eui48);
}

/**
 * Base Management Set Signature Data Request
 * - uc_cmd:               Command to acknowledge
 * - uc_algorithm:         Used algorithm to check signature
 * - us_length:            Signature length
 */
static void _bmng_fup_set_signature_data_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint16_t us_len;
	uint8_t uc_algorithm;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	uc_algorithm = *puc_msg++;
	us_len = ((uint16_t)(*puc_msg++)) << 8;
	us_len += *puc_msg++;

	prime_bmng_fup_set_signature_data_request(prime_bmng_fup_set_signature_data_request_cmd, uc_algorithm, us_len);
}

/**
 * Base Management PRIME Profile Get Request
 * - uc_cmd:               Command to acknowledge
 * - puc_eui48:            MAC address of the node
 * - us_data_length:       Data length
 * - puc_data:             Data
 */
static void _bmng_pprof_get_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint8_t puc_data[1024];
	uint8_t puc_eui48[6];
	uint16_t us_data_len;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	memcpy(puc_eui48, puc_msg, 6);
	puc_msg += 6;
	us_data_len = ((uint16_t)(*puc_msg++)) << 8;
	us_data_len += *puc_msg++;
	memcpy(puc_data, puc_msg, us_data_len);

	prime_bmng_pprof_get_request(prime_bmng_pprof_get_request_cmd, puc_eui48, us_data_len, puc_data);
}

/**
 * Base Management PRIME Profile Set Request
 * - uc_cmd:               Command to acknowledge
 * - puc_eui48:            MAC address of the node
 * - us_data_length:       Data length
 * - puc_data:             Data
 */
static void _bmng_pprof_set_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint8_t puc_data[1024];
	uint8_t puc_eui48[6];
	uint16_t us_data_len;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	memcpy(puc_eui48, puc_msg, 6);
	puc_msg += 6;
	us_data_len = ((uint16_t)(*puc_msg++)) << 8;
	us_data_len += *puc_msg++;
	memcpy(puc_data, puc_msg, us_data_len);

	prime_bmng_pprof_set_request(prime_bmng_pprof_set_request_cmd, puc_eui48, us_data_len, puc_data);
}

/**
 * Base Management PRIME Profile Reset Request
 * - uc_cmd:               Command to acknowledge
 * - puc_eui48:            MAC address of the node
 */
static void _bmng_pprof_reset_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint8_t puc_eui48[6];

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	memcpy(puc_eui48, puc_msg, 6);

	prime_bmng_pprof_reset_request(prime_bmng_pprof_reset_request_cmd, puc_eui48);
}

/**
 * Base Management PRIME Profile Reboot Request
 * - uc_cmd:               Command to acknowledge
 * - puc_eui48:            MAC address of the node
 */
static void _bmng_pprof_reboot_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint8_t puc_eui48[6];

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	memcpy(puc_eui48, puc_msg, 6);

	prime_bmng_pprof_reboot_request(prime_bmng_pprof_reboot_request_cmd, puc_eui48);
}

/**
 * Base Management PRIME Profile Get Enhanced Request
 * - uc_cmd:               Command to acknowledge
 * - puc_eui48:            MAC address of the node
 * - us_data_length:       Data length
 * - puc_data:             Data
 */
static void _bmng_pprof_get_enhanced_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint8_t puc_data[1024];
	uint8_t puc_eui48[6];
	uint16_t us_data_len;

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	memcpy(puc_eui48, puc_msg, 6);
	puc_msg += 6;
	us_data_len = ((uint16_t)(*puc_msg++)) << 8;
	us_data_len += *puc_msg++;
	memcpy(puc_data, puc_msg, us_data_len);

	prime_bmng_pprof_get_enhanced_request(prime_bmng_pprof_get_enhanced_request_cmd, puc_eui48, us_data_len, puc_data);
}

/**
 * Base Management PRIME Profile Zero Crossing Difference between BN and SN Request
 * - uc_cmd:               Command to acknowledge
 * - puc_eui48:            MAC address of the node
 */
static void _bmng_pprof_zc_diff_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint8_t puc_eui48[6];

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	memcpy(puc_eui48, puc_msg, 6);

	prime_bmng_pprof_zc_diff_request(prime_bmng_pprof_zc_diff_request_cmd, puc_eui48);
}

/**
 * Base Management Whitelist Add Request
 * - uc_cmd:             Command to acknowledge
 * - puc_eui48:          MAC address of the node to be added
 */
static void _bmng_whitelist_add_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint8_t puc_eui48[6];

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	memcpy(puc_eui48, puc_msg, 6);

	prime_bmng_whitelist_add_request(prime_bmng_whitelist_add_request_cmd, puc_eui48);
}

/**
 * Base Management Whitelist Remove Request
 * - uc_cmd:             Command to acknowledge
 * - puc_eui48:          MAC address of the node to be removed
 */
static void _bmng_whitelist_remove_request_cmd(uint8_t *puc_rcv_msg)
{
	uint8_t *puc_msg;
	uint8_t puc_eui48[6];

	/* Extract parameters */
	puc_msg = puc_rcv_msg;
	memcpy(puc_eui48, puc_msg, 6);

	prime_bmng_whitelist_remove_request(prime_bmng_whitelist_remove_request_cmd, puc_eui48);
}

/**
 * Report a debug error
 *
 * - ul_err_type:     Type of error
 */
void modem_debug_report(uint32_t ul_err_type)
{
	uint8_t *puc_buf;

	puc_buf = suc_serial_buf;

	*puc_buf++ = prime_debug_report_cmd;
	*puc_buf++ = (uint8_t)((uint32_t)ul_err_type >> 24);
	*puc_buf++ = (uint8_t)((uint32_t)ul_err_type >> 16);
	*puc_buf++ = (uint8_t)((uint32_t)ul_err_type >> 8);
	*puc_buf++ = (uint8_t)((uint32_t)ul_err_type);

	/* Set data for USI */
	x_usi_msg.us_len = puc_buf - suc_serial_buf;

	/* Send packet */
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * \brief Process MODEM Application
 *
 */
void modem_process(void)
{

#ifdef __PRIME_GATEWAY__
	gw_ipv6_process();
#endif

	/* Check data reception */
	while(sx_msg_rcv[suc_output_msg_rcv].us_len) {
		prime_api_cmd_t uc_cmd;
		uint8_t *puc_rcv_buf;

		/* Extract cmd */
		puc_rcv_buf = sx_msg_rcv[suc_output_msg_rcv].data_buf;
		uc_cmd = (prime_api_cmd_t)*puc_rcv_buf++;
		switch (uc_cmd) {
		case prime_cl_null_establish_request_cmd:
			_establish_request_cmd(puc_rcv_buf);
			break;

		case prime_cl_null_establish_response_cmd:
			_establish_response_cmd(puc_rcv_buf);
			break;

		case prime_cl_null_release_request_cmd:
			_release_request_cmd(puc_rcv_buf);
			break;

		case prime_cl_null_release_response_cmd:
			_release_response_cmd(puc_rcv_buf);
			break;

		case prime_cl_null_redirect_response_cmd:
			_redirect_response_cmd(puc_rcv_buf);
			break;

		case prime_cl_null_join_request_cmd:
			_join_request_cmd(puc_rcv_buf);
			break;

		case prime_cl_null_join_response_cmd:
			_join_response_cmd(puc_rcv_buf);
			break;

		case prime_cl_null_leave_request_cmd:
			_leave_request_cmd(puc_rcv_buf);
			break;

		case prime_cl_null_data_request_cmd:
			_data_request_cmd(puc_rcv_buf);
			break;

		case prime_cl_null_plme_reset_request_cmd:
			_plme_reset_request_cmd(puc_rcv_buf);
			break;

		case prime_cl_null_plme_sleep_request_cmd:
			_plme_sleep_request_cmd(puc_rcv_buf);
			break;

		case prime_cl_null_plme_resume_request_cmd:
			_plme_resume_request_cmd(puc_rcv_buf);
			break;

		case prime_cl_null_plme_testmode_request_cmd:
			/* Not implemented */
			break;

		case prime_cl_null_plme_get_request_cmd:
			_plme_get_request_cmd(puc_rcv_buf);
			break;

		case prime_cl_null_plme_set_request_cmd:
			_plme_set_request_cmd(puc_rcv_buf);
			break;

		case prime_cl_null_mlme_promote_request_cmd:
			_mlme_promote_request_cmd(puc_rcv_buf);
			break;

		case prime_cl_null_mlme_mp_promote_request_cmd:
			_mlme_mp_promote_request_cmd(puc_rcv_buf);
			break;

		case prime_cl_null_mlme_reset_request_cmd:
			prime_cl_null_mlme_reset_request();
			break;

		case prime_cl_null_mlme_get_request_cmd:
			_mlme_get_request_cmd(puc_rcv_buf);
			break;

		case prime_cl_null_mlme_list_get_request_cmd:
			_mlme_list_get_request_cmd(puc_rcv_buf);
			break;

		case prime_cl_null_mlme_set_request_cmd:
			_mlme_set_request_cmd(puc_rcv_buf);
			break;

#if USED_SSCS == SSCS_432
		case prime_cl_432_release_request_cmd:
			_432_release_request_cmd(puc_rcv_buf);
			break;

		case prime_cl_432_dl_data_request_cmd:
			_432_dl_data_request_cmd(puc_rcv_buf);
			break;
#endif

		case prime_bmng_fup_clear_target_list_request_cmd:
			bmng_fup_clear_target_list_request(prime_bmng_fup_clear_target_list_request_cmd);
			break;

		case prime_bmng_fup_add_target_request_cmd:
			_bmng_fup_add_target_request_cmd(puc_rcv_buf);
			break;

		case prime_bmng_fup_set_fw_data_request_cmd:
			_bmng_fup_set_fw_data_request_cmd(puc_rcv_buf);
			break;

		case prime_bmng_fup_set_upg_options_request_cmd:
			_bmng_fup_set_upg_options_request_cmd(puc_rcv_buf);
			break;

		case prime_bmng_fup_init_file_tx_request_cmd:
			_bmng_fup_init_file_tx_request_cmd(puc_rcv_buf);
			break;

		case prime_bmng_fup_data_frame_request_cmd:
			_bmng_fup_data_frame_request_cmd(puc_rcv_buf);
			break;

		case prime_bmng_fup_check_crc_request_cmd:
			bmng_fup_check_crc_request(prime_bmng_fup_check_crc_request_cmd);
			break;

		case prime_bmng_fup_abort_fu_request_cmd:
			_bmng_fup_abort_fu_request_cmd(puc_rcv_buf);
			break;

		case prime_bmng_fup_start_fu_request_cmd:
			_bmng_fup_start_fu_request_cmd(puc_rcv_buf);
			break;

		case prime_bmng_fup_set_match_rule_request_cmd:
			_bmng_fup_set_match_rule_request_cmd(puc_rcv_buf);
			break;

		case prime_bmng_fup_get_version_request_cmd:
			_bmng_fup_get_version_request_cmd(puc_rcv_buf);
			break;

		case prime_bmng_fup_get_state_request_cmd:
			_bmng_fup_get_state_request_cmd(puc_rcv_buf);
			break;

		case prime_bmng_fup_set_signature_data_request_cmd:
			_bmng_fup_set_signature_data_request_cmd(puc_rcv_buf);
			break;

		case prime_bmng_pprof_get_request_cmd:
			_bmng_pprof_get_request_cmd(puc_rcv_buf);
			break;

		case prime_bmng_pprof_set_request_cmd:
			_bmng_pprof_set_request_cmd(puc_rcv_buf);
			break;

		case prime_bmng_pprof_reset_request_cmd:
			_bmng_pprof_reset_request_cmd(puc_rcv_buf);
			break;

		case prime_bmng_pprof_reboot_request_cmd:
			_bmng_pprof_reboot_request_cmd(puc_rcv_buf);
			break;

		case prime_bmng_pprof_get_enhanced_request_cmd:
			_bmng_pprof_get_enhanced_request_cmd(puc_rcv_buf);
			break;

		case prime_bmng_pprof_zc_diff_request_cmd:
			_bmng_pprof_zc_diff_request_cmd(puc_rcv_buf);
			break;

		case prime_bmng_whitelist_add_request_cmd:
			_bmng_whitelist_add_request_cmd(puc_rcv_buf);
			break;

		case prime_bmng_whitelist_remove_request_cmd:
			_bmng_whitelist_remove_request_cmd(puc_rcv_buf);
			break;

#if USED_SSCS == SSCS_IPV6
#ifndef __PRIME_GATEWAY__
		case prime_cl_ipv6_data_request_cmd:
			_ipv6_data_request_cmd(puc_rcv_buf);
			break;

		case prime_cl_ipv6_mul_join_request_cmd:
			_ipv6_mul_join_request_cmd(puc_rcv_buf);
			break;

		case prime_cl_ipv6_mul_leave_request_cmd:
			_ipv6_mul_leave_request_cmd(puc_rcv_buf);
			break;
#endif
#endif

		default:
			hal_debug_report(MODEM_ERR_UNKNOWN_CMD);
			break;
		}

		sx_msg_rcv[suc_output_msg_rcv].us_len = 0;
		if (++suc_output_msg_rcv == MAX_NUM_MSG_RCV) {
			suc_output_msg_rcv = 0;
		}
	}
}
