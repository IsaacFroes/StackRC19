/**
 * \file
 *
 * \brief MODEM : Modem Application for ATMEL PRIME NG v.1.4 Base Node
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

#ifndef MODEM_H_INCLUDED
#define MODEM_H_INCLUDED

#include <stdint.h>
#include "conf_prime_stack.h"

/* ENABLE MODEM MESSAGES */
/* #define MODEM_DEBUG_ENABLE */

typedef enum {
	prime_cl_null_establish_request_cmd 	= 0x01,
	prime_cl_null_establish_ind_cmd 	= 0x02,
	prime_cl_null_establish_cfm_cmd 	= 0x03,
	prime_cl_null_establish_response_cmd 	= 0x04,
	prime_cl_null_release_request_cmd 	= 0x05,
	prime_cl_null_release_ind_cmd 		= 0x06,
	prime_cl_null_release_cfm_cmd 		= 0x07,
	prime_cl_null_release_response_cmd 	= 0x08,
	prime_cl_null_join_request_cmd 		= 0x09,
	prime_cl_null_join_ind_cmd 		= 0x0A,
	prime_cl_null_join_response_cmd 	= 0x0B,
	prime_cl_null_join_cfm_cmd 		= 0x0C,
	prime_cl_null_leave_request_cmd 	= 0x0D,
	prime_cl_null_leave_cfm_cmd 		= 0x0E,
	prime_cl_null_leave_ind_cmd 		= 0x0F,
	prime_cl_null_data_request_cmd 		= 0x10,
	prime_cl_null_data_cfm_cmd 		= 0x11,
	prime_cl_null_data_ind_cmd 		= 0x12,
	prime_cl_null_plme_reset_request_cmd 	= 0x13,
	prime_cl_null_plme_reset_cfm_cmd 	= 0x14,
	prime_cl_null_plme_sleep_request_cmd 	= 0x15,
	prime_cl_null_plme_sleep_cfm_cmd 	= 0x16,
	prime_cl_null_plme_resume_request_cmd 	= 0x17,
	prime_cl_null_plme_resume_cfm_cmd 	= 0x18,
	prime_cl_null_plme_testmode_request_cmd = 0x19,
	prime_cl_null_plme_testmode_cfm_cmd 	= 0x1A,
	prime_cl_null_plme_get_request_cmd 	= 0x1B,
	prime_cl_null_plme_get_cfm_cmd 		= 0x1C,
	prime_cl_null_plme_set_request_cmd 	= 0x1D,
	prime_cl_null_plme_set_cfm_cmd 		= 0x1E,
	prime_cl_null_mlme_register_request_cmd = 0x1F,
	prime_cl_null_mlme_register_cfm_cmd 	= 0x20,
	prime_cl_null_mlme_register_ind_cmd 	= 0x21,
	prime_cl_null_mlme_unregister_request_cmd = 0x22,
	prime_cl_null_mlme_unregister_cfm_cmd 	= 0x23,
	prime_cl_null_mlme_unregister_ind_cmd 	= 0x24,
	prime_cl_null_mlme_promote_request_cmd 	= 0x25,
	prime_cl_null_mlme_promote_cfm_cmd 	= 0x26,
	prime_cl_null_mlme_promote_ind_cmd 	= 0x27,
	prime_cl_null_mlme_demote_request_cmd 	= 0x28,
	prime_cl_null_mlme_demote_cfm_cmd 	= 0x29,
	prime_cl_null_mlme_demote_ind_cmd 	= 0x2A,
	prime_cl_null_mlme_reset_request_cmd 	= 0x2B,
	prime_cl_null_mlme_reset_cfm_cmd 	= 0x2C,
	prime_cl_null_mlme_get_request_cmd 	= 0x2D,
	prime_cl_null_mlme_get_cfm_cmd 		= 0x2E,
	prime_cl_null_mlme_list_get_request_cmd = 0x2F,
	prime_cl_null_mlme_list_get_cfm_cmd 	= 0x30,
	prime_cl_null_mlme_set_request_cmd 	= 0x31,
	prime_cl_null_mlme_set_cfm_cmd 		= 0x32,
	prime_cl_432_establish_request_cmd 	= 0x33,
	prime_cl_432_establish_cfm_cmd 		= 0x34,
	prime_cl_432_release_request_cmd 	= 0x35,
	prime_cl_432_release_cfm_cmd 		= 0x36,
	prime_cl_432_dl_data_request_cmd 	= 0x37,
	prime_cl_432_dl_data_ind_cmd 		= 0x38,
	prime_cl_432_dl_data_cfm_cmd 		= 0x39,
	prime_cl_432_dl_join_ind_cmd  		= 0x3A,
	prime_cl_432_dl_leave_ind_cmd 		= 0x3B,
	prime_cl_null_redirect_response_cmd 	= 0x3C,
#ifdef PRIME_API_BN
	prime_bmng_fup_clear_target_list_request_cmd  = 0x3D,
	prime_bmng_fup_add_target_request_cmd         = 0x3E,
	prime_bmng_fup_set_fw_data_request_cmd        = 0x3F,
	prime_bmng_fup_set_upg_options_request_cmd    = 0x40,
	prime_bmng_fup_init_file_tx_request_cmd       = 0x41,
	prime_bmng_fup_data_frame_request_cmd         = 0x42,
	prime_bmng_fup_check_crc_request_cmd          = 0x43,
	prime_bmng_fup_abort_fu_request_cmd           = 0x44,
	prime_bmng_fup_start_fu_request_cmd           = 0x45,
	prime_bmng_fup_set_match_rule_request_cmd     = 0x46,
	prime_bmng_fup_get_version_request_cmd        = 0x47,
	prime_bmng_fup_get_state_request_cmd          = 0x48,
	prime_bmng_fup_ack_cmd                        = 0x49,
	prime_bmng_fup_status_ind_cmd                 = 0x4A,
	prime_bmng_fup_error_ind_cmd                  = 0x4B,
	prime_bmng_fup_version_ind_cmd                = 0x4C,
	prime_bmng_fup_kill_ind_cmd                   = 0x4D,
	prime_bmng_fup_set_signature_data_request_cmd = 0x4E,
	prime_bmng_network_event_cmd                  = 0x4F,
	prime_bmng_pprof_get_request_cmd              = 0x50,
	prime_bmng_pprof_set_request_cmd              = 0x51,
	prime_bmng_pprof_reset_request_cmd            = 0x52,
	prime_bmng_pprof_reboot_request_cmd           = 0x53,
	prime_bmng_pprof_get_enhanced_request_cmd     = 0x54,
	prime_bmng_pprof_ack_cmd                      = 0x55,
	prime_bmng_pprof_get_response_cmd             = 0x56,
	prime_bmng_pprof_get_enhanced_response_cmd    = 0x57,
	prime_bmng_pprof_get_zc_response_cmd          = 0x58,
	prime_bmng_pprof_zc_diff_request_cmd          = 0x59,
	prime_bmng_pprof_zc_diff_response_cmd         = 0x5A,
	prime_bmng_whitelist_add_request_cmd          = 0x5B,
	prime_bmng_whitelist_remove_request_cmd       = 0x5C,
	prime_bmng_whitelist_ack_cmd                  = 0x5D,
#endif
	prime_debug_report_cmd                        = 0x5E,
	prime_cl_ipv6_establish_request_cmd           = 0x5F,
	prime_cl_ipv6_establish_cfm_cmd               = 0x60,
	prime_cl_ipv6_release_request_cmd             = 0x61,
	prime_cl_ipv6_release_cfm_cmd                 = 0x62,
	prime_cl_ipv6_register_request_cmd            = 0x63,
	prime_cl_ipv6_register_cfm_cmd                = 0x64,
	prime_cl_ipv6_unregister_request_cmd          = 0x65,
	prime_cl_ipv6_unregister_cfm_cmd              = 0x66,
	prime_cl_ipv6_data_request_cmd                = 0x67,
	prime_cl_ipv6_data_ind_cmd                    = 0x68,
	prime_cl_ipv6_data_cfm_cmd                    = 0x69,
	prime_cl_ipv6_mul_join_request_cmd            = 0x6A,
	prime_cl_ipv6_mul_join_cfm_cmd                = 0x6B,
	prime_cl_ipv6_mul_leave_request_cmd           = 0x6C,
	prime_cl_ipv6_mul_leave_cfm_cmd               = 0x6D,
	prime_cl_null_mlme_mp_promote_request_cmd 	  = 0x6E,
	prime_cl_null_mlme_mp_promote_cfm_cmd 	      = 0x6F,
	prime_cl_null_mlme_mp_promote_ind_cmd 	      = 0x70,
	prime_cl_null_mlme_mp_demote_request_cmd 	  = 0x71,
	prime_cl_null_mlme_mp_demote_cfm_cmd 	      = 0x72,
	prime_cl_null_mlme_mp_demote_ind_cmd 	      = 0x73,
	prime_api_error_cmd
} prime_api_cmd_t;

typedef enum {
	MODEM_NODE_UNREGISTERED = 0,
	MODEM_NODE_REGISTERED   = 1,
	MODEM_NODE_SWITCH       = 2,
	MODEM_NODE_BASE         = 3
} modem_node_state_t;

#define MODEM_ERR_UNKNOWN_CMD       9500
#define MODEM_ERR_MSG_TOO_BIG       9501
#define MODEM_ERR_QUEUE_FULL        9502

/** \brief Modem interface */
/* @{ */
void modem_init(void);
void modem_process(void);
uint8_t modem_txdata_ind(void);
uint8_t modem_rxdata_ind(void);
modem_node_state_t modem_node_state(void);
void modem_debug_report(uint32_t ul_err_type);
/* @} */

#ifdef PRIME_API_BN
void _mdm_bmng_net_event_ind_cb(void *px_net_event);
#endif

/** \brief FreeRTOS Task definition */
/* @{ */
void vModemInitTask(void);
void vModemResetTask(void);
/* @} */


#endif /* MODEM_H_INCLUDED */
