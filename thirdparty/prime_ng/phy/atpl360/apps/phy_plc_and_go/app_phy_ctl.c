/**
 * \file
 *
 * \brief PHY_CHAT : PRIME-PLC Phy Getting Started Applicationn. Module to manage PL360 PHY Layer
 *
 * Copyright (c) 2020 Atmel Corporation. All rights reserved.
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

/* Atmel library includes. */
#include "asf.h"
#include "conf_app_example.h"
#include "app_phy_ctl.h"
#include "hal_private.h"

#ifdef PHY_CONTROLLER_DEBUG_ENABLE
#define LOG_PHY_CTL_DEBUG(a)   printf a
#else
#define LOG_PHY_CTL_DEBUG(a)   (void)0
#endif

/* Phy controller callbacks */
static phy_ctl_callbacks_t sx_phy_ctl_cbs;

/* ATPL360 descriptor definition */
static atpl360_descriptor_t sx_atpl360_desc;

/* Exception counters */
static uint8_t suc_err_unexpected;
static uint8_t suc_err_critical;
static uint8_t suc_err_reset;
static uint8_t suc_err_none;
static bool sb_exception_pend;
static bool sb_enabling_pl360;

/* PRIME channel */
static uint8_t suc_prime_channel;

/* Transmission management */
static tx_msg_t sx_tx_msg;

/**
 * \brief Set PL360 configuration. Called at initialization (once the binary is loaded) to configure required parameters in PL360 device
 */
static void _set_pl360_configuration(void)
{
	uint8_t uc_value;

	/********* The following lines show how to configure different parameters on PL360 device *********/
	/********* The user can customize it depending on the requirements ********************************/

	/* Force Transmission to VLO mode by default in order to maximize signal level in anycase */
	/* Disable autodetect mode */
	uc_value = 0;
	sx_atpl360_desc.set_config(ATPL360_REG_CFG_AUTODETECT_IMPEDANCE, &uc_value, 1);
	/* Set VLO mode */
	uc_value = 2;
	sx_atpl360_desc.set_config(ATPL360_REG_CFG_IMPEDANCE, &uc_value, 1);

	/* Restore/Initialize PRIME channel and corresponding Tx and Coupling paramters */
	phy_ctl_set_channel(suc_prime_channel);
}

/**
 * \brief Calculates maximun length in bytes allowed depending on configuration
 * The maximum data length depends on:
 *                 - Modulation (BPSK Robust, QPSK Roubust, BPSK, QPSK or 8PSK). See "enum mod_schemes" in "atpl360_comm.h"
 *                 - Frame Type (Type A, Type B or Type BC). See "enum mode_types" in "atpl360_comm.h"
 *                 - Maximum data length allowed by PL360 PHY Layer (511)
 *
 * \return Max PSDU length in bytes
 */
static uint16_t _get_max_psdu_len(void)
{
	uint16_t us_max_mpdu2_len;
	uint16_t us_max_psdu_len;
	uint8_t uc_mdpu1_len;

	/* Maximum MPDU2 length depending on modulation scheme. Length of data in bytes sent in payload */
	switch (sx_tx_msg.uc_scheme) {
	case MOD_SCHEME_DBPSK:
		us_max_mpdu2_len = 756;
		break;

	case MOD_SCHEME_DQPSK:
		us_max_mpdu2_len = 1512;
		break;

	case MOD_SCHEME_D8PSK:
		us_max_mpdu2_len = 2268;
		break;

	case MOD_SCHEME_DBPSK_C:
	case MOD_SCHEME_R_DBPSK:
		us_max_mpdu2_len = 377;
		break;

	case MOD_SCHEME_DQPSK_C:
	case MOD_SCHEME_R_DQPSK:
		us_max_mpdu2_len = 755;
		break;

	case MOD_SCHEME_D8PSK_C:
		us_max_mpdu2_len = 1133;
		break;
	}

	/* MPDU1 length. Length of data in bytes that is sent in header */
	switch (sx_tx_msg.uc_mod_type) {
	case MODE_TYPE_A:
		uc_mdpu1_len = 7;
		break;

	case MODE_TYPE_B:
	case MODE_TYPE_BC:
		uc_mdpu1_len = 0;
		break;
	}

	/* Maximum data length: Maximum MPDU2 length + MPDU1 length */
	us_max_psdu_len = us_max_mpdu2_len + uc_mdpu1_len;

	/* Saturate to maximum data length allowed by PL360 PHY (511) */
	if (us_max_psdu_len > 511) {
		us_max_psdu_len = 511;
	}

	return us_max_psdu_len;
}

/**
 * \brief Setup parameters to use in Tx message
 */
static void _setup_tx_parameters(void)
{
	uint16_t us_max_data_len;

	/* Modulation Scheme. See "enum mod_schemes" in "atpl360_comm.h" file */
	/* Ordered from higher to lower data rate and from higher to lower required SNR (Signal to Noise Ratio): */
	/* D8PSK, DQPSK, D8PSK_CC, DBPSK, DQPSK_C, DBPSK_C, R_DQPSK, R_DBPSK */
	/* Get maximum data length allowed with configured Tx Parameters */
	us_max_data_len = phy_ctl_set_mod_scheme(MOD_SCHEME_DBPSK_C);

	/* Transmission Mode. See TX Mode Bit Mask in "atpl360_comm.h" file */
	/* TX_MODE_RELATIVE: Time (ul_tx_time) in relative mode. The message is sent with a delay from the time of Tx Request */
	/* TX_MODE_ABSOLUTE: Time (ul_tx_time) in absolute mode. The message is sent at the specified time, referred to PL360 internal timer (1 us) */
	sx_tx_msg.uc_tx_mode = TX_MODE_RELATIVE;

	/* Transmission Forced Mode: If there is a reception in progress at the same time of transmission, the message is transmitted and the reception is aborted  */
	sx_tx_msg.x_csma_mode.uc_disable_rx = 1;
	sx_tx_msg.x_csma_mode.uc_sense_count = 0;
	sx_tx_msg.x_csma_mode.uc_sense_delay_ms = 0;

	/* Transmission Time in us. Relative or Absolute time (depending on Transmission Mode) */
	/* TX_MODE_RELATIVE and ul_tx_time = 0: Instantaneous transmission */
	sx_tx_msg.ul_tx_time = 0;

	/* Set transmission attenuation power. It represents 1dB of signal level attenuation per Unit. 0 value means maximum signal level. */
	sx_tx_msg.uc_att_level = 0;

	/* Select buffer. There are two buffers and it is possible to schedule two different transmissions, but both transmissions cannot be overlapped in time. */
	/* If they overlap, the first one will be transmitted and the second one will be cancelled with TX_RESULT_BUSY_TX result. */
	sx_tx_msg.uc_buffer_id = TX_BUFFER_0;

	/* Update configured Modulation and maximum data length in Chat App */
	if (sx_phy_ctl_cbs.phy_ctl_update_tx_configuration) {
		sx_phy_ctl_cbs.phy_ctl_update_tx_configuration(sx_tx_msg.uc_scheme, us_max_data_len);
	}
}

/**
 * \brief Handler to manage confirmation of the last PLC transmission.
 *
 * \param px_msg_cfm Pointer to struct containing Tx Confirm paramters
 *
 */
static void _handler_data_cfm(tx_cfm_t *px_msg_cfm)
{
	if (sx_phy_ctl_cbs.phy_ctl_data_confirm) {
		sx_phy_ctl_cbs.phy_ctl_data_confirm(px_msg_cfm->uc_tx_result);
	}
}

/**
 * \brief Handler to manage new received PLC message.
 *
 * \param px_msg Pointer to struct containing message parameters and data
 *
 */
static void _handler_data_ind(rx_msg_t *px_msg)
{
	uint8_t *puc_data_buf;
	uint32_t ul_crc32_calc;
	uint32_t ul_crc32_received;
	uint16_t us_data_len;
	uint8_t uc_rssi;
	uint8_t uc_cinr_avg;
	enum mod_schemes uc_scheme;

	/* Get data length in bytes */
	us_data_len = px_msg->us_data_len;

	if (us_data_len <= 4) {
		/* Length error. Data length should be at least 5 bytes (1 data byte + 4 CRC bytes) */
		if (sx_phy_ctl_cbs.phy_ctl_rx_msg_discarded) {
			sx_phy_ctl_cbs.phy_ctl_rx_msg_discarded();
		}
	} else {
		/* Discount 4 bytes corresponding to 32-bit CRC */
		us_data_len -= 4;

		/* Get pointer to buffer containing received data */
		puc_data_buf = px_msg->puc_data_buf;

		/* Compute PRIME 32-bit CRC. Use hal_pcrc module */
		ul_crc32_calc = hal_pcrc_calc_fu(puc_data_buf, us_data_len, 0);

		/* Get CRC from last 4 bytes of the message */
		ul_crc32_received = puc_data_buf[us_data_len + 3];
		ul_crc32_received += (uint32_t)puc_data_buf[us_data_len + 2] << 8;
		ul_crc32_received += (uint32_t)puc_data_buf[us_data_len + 1] << 16;
		ul_crc32_received += (uint32_t)puc_data_buf[us_data_len] << 24;

		/* Check integrity of received message comparing the computed CRC with the received CRC in last 4 bytes */
		if (ul_crc32_calc == ul_crc32_received) {
			/* CRC Ok. Get some parameters to show in console */
			/* There are more parameters not used in this example application (see "rx_msg_t" struct in "atpl360_comm.h") */

			/* Get Modulation Scheme used in received message */
			uc_scheme = px_msg->uc_scheme;

			/* Get RSSI (Received Signal Strength Indicator) in dBuV */
			uc_rssi = px_msg->uc_rssi_avg;

			/* Get Averaged CINR (Carrier to Interference-plus-Noise Ratio) in quarters of dB and 10-dB offset (uc_cinr_avg = 0 means -10 dB) */
			uc_cinr_avg = px_msg->uc_cinr_avg;

			if (sx_phy_ctl_cbs.phy_ctl_rx_msg) {
				sx_phy_ctl_cbs.phy_ctl_rx_msg(puc_data_buf, us_data_len, uc_scheme, uc_rssi, uc_cinr_avg);
			}
		} else {
			/* CRC Error */
			if (sx_phy_ctl_cbs.phy_ctl_rx_msg_discarded) {
				sx_phy_ctl_cbs.phy_ctl_rx_msg_discarded();
			}
		}
	}
}

/**
 * \brief Handler to manage PL360 Exceptions. This callback is also called after loading binary at initization
 */
static void _handler_exception_event(atpl360_exception_t exception)
{
	LOG_PHY_CTL_DEBUG(("\r\n"));

	switch (exception) {
	case ATPL360_EXCEPTION_UNEXPECTED_SPI_STATUS:
		/* SPI has detected an unexpected status, reset is recommended */
		suc_err_unexpected++;
		LOG_PHY_CTL_DEBUG(("ATPL360_EXCEPTION_UNEXPECTED_SPI_STATUS\r\n"));
		break;

	case ATPL360_EXCEPTION_SPI_CRITICAL_ERROR:
		/* SPI critical error */
		suc_err_critical++;
		LOG_PHY_CTL_DEBUG(("ATPL360_EXCEPTION_SPI_CRITICAL_ERROR\r\n"));
		break;

	case ATPL360_EXCEPTION_RESET:
		/* Device Reset */
		if (sb_enabling_pl360) {
			/* This callback is also called after loading binary at initization */
			/* This message is shown to indicate that the followitn exception is normal because PL360 binary has just been loaded */
			sb_enabling_pl360 = false;
			LOG_PHY_CTL_DEBUG(("PL360 initialization event: "));
		}

		suc_err_reset++;
		LOG_PHY_CTL_DEBUG(("ATPL360_EXCEPTION_RESET\r\n"));
		break;

	default:
		LOG_PHY_CTL_DEBUG(("ATPL360_EXCEPTION_UNKNOWN\r\n"));
		suc_err_none++;
	}

	/* Set flag to manage exception in phy_ctl_process() */
	sb_exception_pend = true;
}

/**
 * \brief Get PL360 binary addressing.
 *
 * \param pul_address   Pointer to store the initial address of PL360 binary data
 *
 * \return Size of PL360 binary file
 */
static uint32_t _get_pl360_bin_addressing(uint32_t *pul_address)
{
	uint32_t ul_bin_addr;
	uint8_t *puc_bin_start;
	uint8_t *puc_bin_end;

#if defined (__CC_ARM)
	extern uint8_t atpl_bin_start[];
	extern uint8_t atpl_bin_end[];
	ul_bin_addr = (int)(atpl_bin_start - 1);
	puc_bin_start = atpl_bin_start - 1;
	puc_bin_end = atpl_bin_end;
#elif defined (__GNUC__)
	extern uint8_t atpl_bin_start;
	extern uint8_t atpl_bin_end;
	ul_bin_addr = (int)&atpl_bin_start;
	puc_bin_start = (int)&atpl_bin_start;
	puc_bin_end = (int)&atpl_bin_end;
#elif defined (__ICCARM__)
#  pragma section = "P_atpl_bin"
	extern uint8_t atpl_bin;
	ul_bin_addr = (int)&atpl_bin;
	puc_bin_start = __section_begin("P_atpl_bin");
	puc_bin_end = __section_end("P_atpl_bin");
#else
#  error This compiler is not supported for now.
#endif
	*pul_address = ul_bin_addr;
	/* cppcheck-suppress deadpointer */
	return ((uint32_t)puc_bin_end - (uint32_t)puc_bin_start);
}

/**
 * \brief Enables PL360 device. Load PHY binary.
 *
 */
static void _pl360_enable(void)
{
	uint32_t ul_bin_addr;
	uint32_t ul_bin_size;
	uint8_t uc_ret;
	char puc_version_str[11];
	uint8_t puc_version_num[4];

	/* Get PL360 bininary address and size */
	ul_bin_size = _get_pl360_bin_addressing(&ul_bin_addr);

	/* Enable PL360: Load binary */
	LOG_PHY_CTL_DEBUG(("\r\nEnabling PL360 device: Loading PHY binary\r\n"));
	sb_enabling_pl360 = true;
	uc_ret = atpl360_enable(ul_bin_addr, ul_bin_size);
	if (uc_ret == ATPL360_ERROR) {
		LOG_PHY_CTL_DEBUG(("\r\nCRITICAL ERROR: PL360 binary load failed (%d)\r\n", uc_ret));
		while (1) {
#if (!PIC32CX)
			WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;
#else
			DWDT->WDT0_CR = WDT0_CR_KEY_PASSWD | WDT0_CR_WDRSTT;
			DWDT->WDT1_CR = WDT1_CR_KEY_PASSWD | WDT1_CR_WDRSTT;
#endif

#ifdef CONF_BOARD_LCD_EN
#if BOARD == ATPL360MB
			c0216CiZ_set_cursor(C0216CiZ_LINE_UP, 0);
			c0216CiZ_show((const char *)"ATPL360MB ERROR");
#elif (BOARD == PIC32CXMTSH_DB)
			cl010_show_icon(CL010_ICON_PHASE_1);
			cl010_show_icon(CL010_ICON_PHASE_2);
			cl010_show_icon(CL010_ICON_PHASE_3);
			cl010_show_icon(CL010_ICON_PF);
#endif
#else
#if (BOARD == SAM4CMS_DB)
			LED_On(LED4);
#else
			LED_On(LED0);
#endif
#endif
			delay_ms(500);
		}
	}

	LOG_PHY_CTL_DEBUG(("\r\nPL360 binary loaded correctly\r\n"));

	/* Get PHY version (string) */
	sx_atpl360_desc.get_config(ATPL360_REG_VERSION_STR, puc_version_str, 11, true);
	LOG_PHY_CTL_DEBUG(("PHY version: %.*s\r\n", 11, puc_version_str));

	/* Get PHY version (hex) */
	sx_atpl360_desc.get_config(ATPL360_REG_VERSION_NUM, puc_version_num, 4, true);

	/* puc_version_num[2] correspons to protocol [0x05: PRIME Single Channel; 0x06: PRIME Single and Double Channel] */
	if ((puc_version_num[2] != 0x05) && (puc_version_num[2] != 0x06)) {
		LOG_PHY_CTL_DEBUG(("ERROR: PHY band does not match with band configured in application\r\n"));
	}
}

/**
 * \brief Initialization of PL360 PHY Layer.
 *
 * \param uc_channel   PRIME channel configured at initialization
 *
 */
void phy_ctl_pl360_init(uint8_t uc_channel)
{
	atpl360_dev_callbacks_t x_atpl360_cbs;
	atpl360_hal_wrapper_t x_atpl360_hal_wrp;

	/* Initialize PRIME channel static variable */
	suc_prime_channel = uc_channel;

	/* Initialize PL360 controller */
	x_atpl360_hal_wrp.plc_init = hal_plc_init;
	x_atpl360_hal_wrp.plc_reset = hal_plc_reset;
	x_atpl360_hal_wrp.plc_set_stby_mode = hal_plc_set_stby_mode;
	x_atpl360_hal_wrp.plc_set_handler = hal_plc_set_handler;
	x_atpl360_hal_wrp.plc_send_boot_cmd = hal_plc_send_boot_cmd;
	x_atpl360_hal_wrp.plc_write_read_cmd = hal_plc_send_wrrd_cmd;
	x_atpl360_hal_wrp.plc_enable_int = hal_plc_enable_interrupt;
	x_atpl360_hal_wrp.plc_delay = hal_plc_delay;
	x_atpl360_hal_wrp.plc_get_thw = hal_plc_get_thermal_warning;
	atpl360_init(&sx_atpl360_desc, &x_atpl360_hal_wrp);

	/* Callback functions configuration. Set NULL as Not used */
	x_atpl360_cbs.data_confirm = _handler_data_cfm;
	x_atpl360_cbs.data_indication = _handler_data_ind;
	x_atpl360_cbs.exception_event = _handler_exception_event;
	x_atpl360_cbs.addons_event = NULL;
	x_atpl360_cbs.debug_mode_cb = NULL;
	x_atpl360_cbs.sleep_mode_cb = NULL;
	sx_atpl360_desc.set_callbacks(&x_atpl360_cbs);

	/* Enable PL360 device: Load binary */
	_pl360_enable();
}

/**
 * \brief Set the callbacks for PHY controller.
 *
 * \param px_phy_callbacks Pointer to structure containing the callback functions.
 *
 */
void phy_ctl_set_callbacks(phy_ctl_callbacks_t *px_phy_callbacks)
{
	sx_phy_ctl_cbs.phy_ctl_data_confirm = px_phy_callbacks->phy_ctl_data_confirm;
	sx_phy_ctl_cbs.phy_ctl_rx_msg_discarded = px_phy_callbacks->phy_ctl_rx_msg_discarded;
	sx_phy_ctl_cbs.phy_ctl_rx_msg = px_phy_callbacks->phy_ctl_rx_msg;
	sx_phy_ctl_cbs.phy_ctl_update_tx_configuration = px_phy_callbacks->phy_ctl_update_tx_configuration;
}

/**
 * \brief Send PLC message.
 *
 * \param puc_data_buf Pointer to buffer containing the data to send.
 * \param us_data_len Data length in bytes.
 *
 * \return Result of sending the message to the PL360
 */
uint8_t phy_ctl_send_msg(uint8_t *puc_data_buff, uint16_t us_data_len)
{
	uint32_t ul_crc32_calc;
	uint8_t uc_result;

	/* Compute PRIME 32-bit CRC and add to buffer after message. Use hal_pcrc module */
	ul_crc32_calc = hal_pcrc_calc_fu(puc_data_buff, us_data_len, 0);
	puc_data_buff[us_data_len]  = (uint8_t)(ul_crc32_calc >> 24);
	puc_data_buff[us_data_len + 1]  = (uint8_t)(ul_crc32_calc >> 16);
	puc_data_buff[us_data_len + 2]  = (uint8_t)(ul_crc32_calc >> 8);
	puc_data_buff[us_data_len + 3]  = (uint8_t)(ul_crc32_calc);

	/* Set pointer to data buffer in Tx Parameters structure */
	sx_tx_msg.puc_data_buf = puc_data_buff;

	/* Set data length in Tx Parameters structure. Add 4 bytes corresponding to CRC */
	/* It should be equal or less than Maximum Data Length (see _get_max_psdu_len) */
	/* Otherwise TX_RESULT_INV_LENGTH will be reported in Tx Confirm (_handler_data_cfm) */
	sx_tx_msg.us_data_len = us_data_len + 4;

	/* Send PLC message. send_data returns TX_RESULT_PROCESS if transmission was correctly programmed */
	/* The result will be reported in Tx Confirm (_handler_data_cfm) when message is completely sent */
	uc_result = sx_atpl360_desc.send_data(&sx_tx_msg);

	return uc_result;
}

/**
 * \brief Change PRIME channel on PL360 device and reconfigure Tx and Coupling parameters according to channel
 *
 * \param uc_chn PRIME channel.
 *
 * \retval false. Invalid configuration
 * \retval true. Channel reconfigured successfully
 */
bool phy_ctl_set_channel(uint8_t uc_chn)
{
	if ((uc_chn >= SINGLE_CHN_1) && (uc_chn <= SINGLE_CHN_8)) {
		/* Valid channel. Set channel on PL360 */
		sx_atpl360_desc.set_config(ATPL360_REG_CHANNEL_CFG, &uc_chn, 1);

		/* Reconfigure Tx and Coupling parameters, according to new configured channel */
		pl360_prime_coup_tx_config(&sx_atpl360_desc, suc_prime_channel);

		/* Store channel in static variable */
		suc_prime_channel = uc_chn;

		return true;
	} else {
		/* Invalid channel */
		return false;
	}
}

/**
 * \brief Get current PRIME channel configured.
 *
 * \return Current PRIME channel configured.
 */
uint8_t phy_ctl_get_channel(void)
{
	/* Return PRIME channel configured */
	return suc_prime_channel;
}

/**
 * \brief Set Modulation Scheme.
 * They are stored at application level, not in PL360 PHY Layer. They are sent to PL360 PHY Layer in Tx Request (send_data).
 *
 * \param uc_scheme Modulation Scheme to be configured.
 *
 * \return Maximum data length that can be transmitted with new Modulation.
 */
uint16_t phy_ctl_set_mod_scheme(enum mod_schemes uc_scheme)
{
	uint16_t us_max_data_len;

	/* Store Modulation Scheme in Tx Parameters structure */
	sx_tx_msg.uc_scheme = uc_scheme;

	/* Set PRIME mode (Frame type) depending on modulation */
	switch (uc_scheme) {
	case MOD_SCHEME_DBPSK:
	case MOD_SCHEME_DQPSK:
	case MOD_SCHEME_D8PSK:
	case MOD_SCHEME_DBPSK_C:
	case MOD_SCHEME_DQPSK_C:
	case MOD_SCHEME_D8PSK_C:
		/* Not robust modulation: supported in both Type A and Type B. Use Type A because it has less overhead */
		sx_tx_msg.uc_mod_type = MODE_TYPE_A;
		break;

	case MOD_SCHEME_R_DBPSK:
	case MOD_SCHEME_R_DQPSK:
	default:
		/* Robust modulation: only supported in Type B */
		sx_tx_msg.uc_mod_type = MODE_TYPE_B;
		break;
	}

	/* Get maximum data length that can be transmitted with new Modulation */
	us_max_data_len = _get_max_psdu_len();
	return us_max_data_len;
}

/**
 * \brief Get current Modulation Scheme configured.
 * It is stored at application level, not in PL360 PHY Layer. It is sent to PL360 PHY Layer in Tx Request (send_data).
 *
 * \return Current Modulation Scheme configured.
 */
enum mod_schemes phy_ctl_get_mod_scheme(void)
{
	/* Return Modulation Scheme stored in Tx Parameters structure */
	return sx_tx_msg.uc_scheme;
}

/**
 * \brief Phy Controller module process.
 */
void phy_ctl_process(void)
{
	/* Manage PL360 exceptions. At initialization ATPL360_EXCEPTION_RESET is reported */
	if (sb_exception_pend) {
		/* Clear exception flag */
		sb_exception_pend = false;

		/* Set PL360 specific configuration from application */
		/* Called at initialization and if an exception ocurrs */
		/* If an exception occurs, PL360 is reset and some parameters may have to be reconfigured */
		_set_pl360_configuration();

		/* Setup PRIME parameters to use in transmission */
		_setup_tx_parameters();
	}

	/* Check ATPL360 pending events. It must be called from application periodically to handle PHY Layer events */
	atpl360_handle_events();
}
