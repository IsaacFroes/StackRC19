/**
 * \file
 *
 * \brief PHY_CHAT : PRIME Phy Getting Started Application. Module to manage console
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
#include "app_console.h"
#include "app_phy_ctl.h"

/* Time in ms that LED is ON after message reception */
#define COUNT_MS_IND_LED        50

/* Maximum message data length allowed by PRIME Physical Layer. */
/* The maximum data length depends on: */
/*                 - Modulation (BPSK Robust, QPSK Roubust, BPSK, QPSK or 8PSK). See "enum mod_schemes" in "atpl360_comm.h" */
/*                 - Frame Type (Type A, Type B or Type BC). See "enum mode_types" in "atpl360_comm.h" */
/*                 - Maximum data length allowed by PL360 PHY Layer (511) */
/* The absolute maximum corresponds to the maximum data length allowed by PL360 PHY Layer (511) */
#define MAX_DATA_LEN        511

/* Application state machine */
typedef enum app_state {
	APP_STATE_IDLE,
	APP_STATE_TYPING,
	APP_STATE_CONFIG_MENU,
	APP_STATE_CONFIG_MOD,
	APP_STATE_CONFIG_CHANNEL,
	APP_STATE_TRANSMITTING,
} app_state_t;

static app_state_t suc_app_state;

/* Transmission data buffer */
static uint8_t spuc_tx_data_buff[MAX_DATA_LEN];

/* Index for transmission data buffer */
static uint16_t sus_tx_data_index;

/* Maximum length with configured Tx Parameters (see the above explanation) */
static uint16_t sus_max_data_len;

/* LED management variables */
extern uint32_t sul_ind_count_ms;

/**
 * \brief Shows maximum data rate that can be achieved depending on Tx modulation.
 * The data rate is computed for maximum data length.
 * data_rate = (max_data_len * 8) / frame_duration.
 *
 * \param uc_scheme Modulation Scheme.
 *
 */
static void _show_data_rate(enum mod_schemes uc_scheme)
{
	switch (uc_scheme) {
	case MOD_SCHEME_D8PSK:
		printf(" ........................ 107.9 kbit/s\r\n");
		break;

	case MOD_SCHEME_DQPSK:
		printf(" ........................ 76.3 kbit/s\r\n");
		break;

	case MOD_SCHEME_DBPSK:
		printf(" ........................ 40.6 kbit/s\r\n");
		break;

	case MOD_SCHEME_D8PSK_C:
		printf(" ... 58.9 kbit/s\r\n");
		break;

	case MOD_SCHEME_DQPSK_C:
		printf(" ... 40.6 kbit/s\r\n");
		break;

	case MOD_SCHEME_DBPSK_C:
		printf(" ... 20.8 kbit/s\r\n");
		break;

	case MOD_SCHEME_R_DQPSK:
		printf(" ................. 10.2 kbit/s\r\n");
		break;

	case MOD_SCHEME_R_DBPSK:
		printf(" ................. 5.2 kbit/s\r\n");
		break;
	}
}

/**
 * \brief Shows Tx modulation as string.
 *
 * \param uc_scheme Modulation Scheme.
 * \param uc_format Two formats to print modulation (0 or 1).
 *
 */
static void _show_modulation(enum mod_schemes uc_scheme, uint8_t uc_format)
{
	if (uc_format == 0) {
		switch (uc_scheme) {
		case MOD_SCHEME_DBPSK:
			printf("DBPSK");
			break;

		case MOD_SCHEME_DQPSK:
			printf("DQPSK");
			break;

		case MOD_SCHEME_D8PSK:
			printf("D8PSK");
			break;

		case MOD_SCHEME_DBPSK_C:
			printf("DBPSK + Convolutional Code");
			break;

		case MOD_SCHEME_DQPSK_C:
			printf("DQPSK + Convolutional Code");
			break;

		case MOD_SCHEME_D8PSK_C:
			printf("D8PSK + Convolutional Code");
			break;

		case MOD_SCHEME_R_DBPSK:
			printf("Robust DBPSK");
			break;

		case MOD_SCHEME_R_DQPSK:
			printf("Robust DQPSK");
			break;
		}
	} else {
		switch (uc_scheme) {
		case MOD_SCHEME_DBPSK:
			printf("DBPSK");
			break;

		case MOD_SCHEME_DQPSK:
			printf("DQPSK");
			break;

		case MOD_SCHEME_D8PSK:
			printf("D8PSK");
			break;

		case MOD_SCHEME_DBPSK_C:
			/* _CC suffix means Convolutional Code */
			printf("DBPSK_CC");
			break;

		case MOD_SCHEME_DQPSK_C:
			/* _CC suffix means Convolutional Code */
			printf("DQPSK_CC");
			break;

		case MOD_SCHEME_D8PSK_C:
			/* _CC suffix means Convolutional Code */
			printf("D8PSK_CC");
			break;

		case MOD_SCHEME_R_DBPSK:
			/* R_ prefix means Robust (it also has Convolutional Code) */
			printf("R_DBPSK");
			break;

		case MOD_SCHEME_R_DQPSK:
			/* R_ prefix means Robust (it also has Convolutional Code) */
			printf("R_DQPSK");
			break;
		}
	}
}

/**
 * \brief Shows Band as string.
 *
 * \param uc_channel PRIME channel (1-8).
 *
 */
static void _show_band(uint8_t uc_channel)
{
	switch (uc_channel) {
	case 1:
		printf("Channel 1 (42 - 89 kHz, CENELEC-A)\r\n");
		break;

	case 2:
		printf("Channel 2 (97 - 144 kHz, CENELEC-BCD)\r\n");
		break;

	case 3:
		printf("Channel 3 (151 - 198 kHz, FCC)\r\n");
		break;

	case 4:
		printf("Channel 4 (206 - 253 kHz, FCC)\r\n");
		break;

	case 5:
		printf("Channel 5 (261 - 308 kHz, FCC)\r\n");
		break;

	case 6:
		printf("Channel 6 (315 - 362 kHz, FCC)\r\n");
		break;

	case 7:
		printf("Channel 7 (370 - 417 kHz, FCC)\r\n");
		break;

	case 8:
		printf("Channel 8 (425 - 472 kHz, FCC)\r\n");
		break;

	default:
		printf("Unknown channel)\r\n");
		break;
	}
}

/**
 * \brief Shows available options through console depending on app state.
 *
 * \param b_full_menu Show full menu (true). Only used in APP_STATE_IDLE.
 *
 */
static void _show_menu(bool b_full_menu)
{
	enum mod_schemes uc_scheme;
	uint8_t uc_chn;
	uint8_t uc_current_channel;

	switch (suc_app_state) {
	case APP_STATE_IDLE:
		/* Idle state: Show available options */
		if (b_full_menu) {
			printf("\r\nPress 'CTRL+S' to enter configuration menu. ");
			printf("Enter text and press 'ENTER' to trigger transmission");
		}

		printf("\r\n>>> ");
		fflush(stdout);
		break;

	case APP_STATE_TYPING:
		/* Typing message to transmit */
		printf("\r\n>>> %.*s", sus_tx_data_index - 1, spuc_tx_data_buff + 1);
		fflush(stdout);
		break;

	case APP_STATE_CONFIG_MENU:
		/* Configuration Menu */
		printf("\r\n--- Configuration Menu ---\r\n");
		printf("Select parameter to configure: \r\n");
		printf("\t0: Tx Modulation\r\n");
		printf("\t1: Tx/Rx Channel\r\n");
		printf(">>> ");
		fflush(stdout);
		break;

	case APP_STATE_CONFIG_MOD:
		/* Modulation Configuration */
		uc_scheme = phy_ctl_get_mod_scheme();
		printf("\r\n--- Tx Modulation Configuration Menu ---\r\n");
		printf("Select Modulation:\r\n");

		if (uc_scheme == MOD_SCHEME_R_DBPSK) {
			printf("->");
		}

		printf("\t0: ");
		_show_modulation(MOD_SCHEME_R_DBPSK, 0);
		_show_data_rate(MOD_SCHEME_R_DBPSK);

		if (uc_scheme == MOD_SCHEME_R_DQPSK) {
			printf("->");
		}

		printf("\t1: ");
		_show_modulation(MOD_SCHEME_R_DQPSK, 0);
		_show_data_rate(MOD_SCHEME_R_DQPSK);

		if (uc_scheme == MOD_SCHEME_DBPSK_C) {
			printf("->");
		}

		printf("\t2: ");
		_show_modulation(MOD_SCHEME_DBPSK_C, 0);
		_show_data_rate(MOD_SCHEME_DBPSK_C);

		if (uc_scheme == MOD_SCHEME_DQPSK_C) {
			printf("->");
		}

		printf("\t3: ");
		_show_modulation(MOD_SCHEME_DQPSK_C, 0);
		_show_data_rate(MOD_SCHEME_DQPSK_C);

		if (uc_scheme == MOD_SCHEME_D8PSK_C) {
			printf("->");
		}

		printf("\t4: ");
		_show_modulation(MOD_SCHEME_D8PSK_C, 0);
		_show_data_rate(MOD_SCHEME_D8PSK_C);

		if (uc_scheme == MOD_SCHEME_DBPSK) {
			printf("->");
		}

		printf("\t5: ");
		_show_modulation(MOD_SCHEME_DBPSK, 0);
		_show_data_rate(MOD_SCHEME_DBPSK);

		if (uc_scheme == MOD_SCHEME_DQPSK) {
			printf("->");
		}

		printf("\t6: ");
		_show_modulation(MOD_SCHEME_DQPSK, 0);
		_show_data_rate(MOD_SCHEME_DQPSK);

		if (uc_scheme == MOD_SCHEME_D8PSK) {
			printf("->");
		}

		printf("\t7: ");
		_show_modulation(MOD_SCHEME_D8PSK, 0);
		_show_data_rate(MOD_SCHEME_D8PSK);

		printf(">>> ");
		fflush(stdout);
		break;

	case APP_STATE_CONFIG_CHANNEL:
		/* Modulation Configuration */
		printf("\r\n--- Tx/Rx Channel Configuration Menu ---\r\n");
		printf("Select Channel:\r\n");

		uc_current_channel = phy_ctl_get_channel();

		for (uc_chn = 1; uc_chn <= 8; uc_chn++) {
			if (USER_BAND_PLAN & (1 << (uc_chn - 1))) {
				/* Channel allowed */
				if (uc_chn == uc_current_channel) {
					printf("->");
				}

				printf("\t%u: ", (uint32_t)uc_chn);
				_show_band(uc_chn);
			}
		}

		printf(">>> ");
		fflush(stdout);
		break;
	}
}

/**
 * \brief Read one char from Serial port.
 *
 * \param c Pointer read char.
 *
 * \retval 0 Success. One char was read
 * \retval 1 There is no char to read.
 */
static uint32_t _read_char(uint8_t *c)
{
#ifdef CONF_BOARD_UDC_CONSOLE
	/* Read char through USB (SAMG55) */
	uint16_t us_res;
	us_res = hal_usb_udc_read_buf(c, 1) ? 0 : 1;
	return us_res;

#else
#  if SAMG55
	/* Read char through USART (SAMG55) */
	uint32_t ul_char;
	uint32_t ul_res;

	ul_res = usart_read((Usart *)CONF_UART, (uint32_t *)&ul_char);
	*c = (uint8_t)ul_char;

	return ul_res;

#  else
	/* Read char through UART */
	return uart_read((Uart *)CONF_UART, c);
#  endif
#endif
}

/**
 * \brief Shows Tx parameters through console.
 *
 * \param uc_scheme Modulation Scheme.
 *
 */
static void _show_tx_parameters(enum mod_schemes uc_scheme)
{
	/* Modulation Type & Modulation Scheme */
	printf("\r\nTx Modulation: ");
	_show_modulation(uc_scheme, 0);

	/* Maximum data length with configured Tx Parameters */
	printf(" (Max data length = %u bytes)\r\n", (uint32_t)sus_max_data_len);
}

/**
 * \brief Handles received message with CRC ok. Prints message parameters and data.
 *
 * \param puc_data_buf Pointer to buffer containing received data.
 * \param us_data_len Length of received data in bytes.
 * \param uc_scheme Modulation Scheme of received frame.
 * \param uc_rssi Received Signal Strength Indicator in dBuV.
 * \param uc_cinr_avg Averaged CINR (Carrier to Interference-plus-Noise Ratio) in quarters of dB and 10-dB offset (uc_cinr_avg = 0 means -10 dB).
 *
 */
static void app_console_handle_rx_msg(uint8_t *puc_data_buf, uint16_t us_data_len, enum mod_schemes uc_scheme, uint8_t uc_rssi, uint8_t uc_cinr_avg)
{
	printf("\rRx (");
	/* Show Modulation of received frame */
	_show_modulation(uc_scheme, 1);
	/* Show RSSI (Received Signal Strength Indicator) in dBuV */
	printf(", RSSI %udBuV", (uint32_t)uc_rssi);
	/* Show CINR (Carrier to Interference-plus-Noise Ratio). It is in quarters of dB and 10-dB offset: CINR(dB) = (uc_cinr_avg - 40) / 4 */
	printf(", CINR %ddB): ", div_round((int16_t)uc_cinr_avg - 40, 4));
	/* Show received message */
	printf("%.*s", us_data_len - 1, puc_data_buf + 1);

	_show_menu(false);

	/* Turn on LED1 to indicate the reception of PLC message */
	sul_ind_count_ms = COUNT_MS_IND_LED;
#ifdef LED1_GPIO
	LED_On(LED1);
#endif
#if (BOARD == PIC32CXMTSH_DB)
#ifdef CONF_BOARD_LCD_EN
	cl010_show_icon(CL010_ICON_PHASE_2);
#endif
#endif
}

/**
 * \brief Handles received message with bad CRC (corrupted message).
 */
static void app_console_handle_rx_msg_discarded(void)
{
	printf("\rRx ERROR: CRC error\r\n");
	_show_menu(false);

	/* Turn on LED1 to indicate the reception of PLC message */
	sul_ind_count_ms = COUNT_MS_IND_LED;
#ifdef LED1_GPIO
	LED_On(LED1);
#endif
#if (BOARD == PIC32CXMTSH_DB)
#ifdef CONF_BOARD_LCD_EN
	cl010_show_icon(CL010_ICON_PHASE_2);
#endif
#endif
}

/**
 * \brief Handles a modification of the configuration of transmission.
 *
 * \param uc_scheme Configured Tx Modulation Scheme.
 * \param us_max_data_len Maximum length of data that can be sent with the configured Tx parameters.
 *
 */
static void app_console_handle_upd_tx_cfg(enum mod_schemes uc_scheme, uint16_t us_max_data_len)
{
	/* Store maximum data length that can be sent with current configured Tx parameters */
	sus_max_data_len = us_max_data_len;

	_show_tx_parameters(uc_scheme);

	/* Initialize data buffer index to point to second byte */
	/* In PRIME Type A frames, the first byte is not fully available (only 6 LSB bits are available and 2 MSB bits must be 0) */
	/* PRIME stack uses the first byte to encode the header type and 6 bits is enough */
	/* But in this application, it is not acceptable because the first character would change */
	sus_tx_data_index = 1;
	spuc_tx_data_buff[0] = 0;

	/* Reset Chat App state */
	suc_app_state = APP_STATE_IDLE;

	_show_menu(true);
}

/**
 * \brief Handles confirm of transmitted message.
 *
 */
static void app_console_handle_tx_cfm(enum tx_result_values uc_tx_result)
{
	switch (uc_tx_result) {
	case TX_RESULT_SUCCESS:
		/* PLC message was successfully transmitted */
		printf(" TX_RESULT_SUCCESS");
		break;

	case TX_RESULT_INV_LENGTH:
		/* Data length is invalid */
		printf(" TX_RESULT_INV_LENGTH");
		break;

	case TX_RESULT_INV_SCHEME:
		/* Modulation Scheme is invalid */
		printf(" TX_RESULT_INV_SCHEME");
		break;

	case TX_RESULT_BUSY_CH:
		/* Transmission aborted because there is a reception in progress (PLC channel is busy) */
		/* If TX_MODE_FORCED is used in Tx parameters, transmission is never aborted by reception */
		printf(" TX_RESULT_BUSY_CH");
		break;

	case TX_RESULT_BUSY_TX:
		/* There is another transmission that has not been transmitted yet */
		printf(" TX_RESULT_BUSY_TX");
		break;

	case TX_RESULT_TIMEOUT:
		/* Timeout Error */
		printf(" TX_RESULT_TIMEOUT");
		break;

	case TX_RESULT_INV_BUFFER:
		/* Invalid buffer (uc_buffer_id) */
		printf(" TX_RESULT_INV_BUFFER");
		break;

	case TX_RESULT_INV_MODE:
		/* Invalid PRIME mode (or frame type) (uc_mod_type) */
		printf(" TX_RESULT_INV_MODE");
		break;

	case TX_RESULT_INV_TX_MODE:
		/* Invalid TX mode (uc_tx_mode) */
		printf(" TX_RESULT_INV_TX_MODE");
		break;

	case TX_RESULT_CANCELLED:
		/* Transmission cancelled */
		printf(" TX_RESULT_CANCELLED");
		break;

	case TX_RESULT_HIGH_TEMP_120:
		/* Transmission aborted because of high temperature (>120ºC) error (only with PL460/PL480) */
		printf(" TX_RESULT_HIGH_TEMP_120");
		break;

	case TX_RESULT_NO_TX:
		/* No transmission ongoing */
		printf(" TX_RESULT_NO_TX");
		break;
	}

	/* Initialize data buffer index to point to second byte */
	/* In PRIME Type A frames, the first byte is not fully available (only 6 LSB bits are available and 2 MSB bits must be 0) */
	/* PRIME stack uses the first byte to encode the header type and 6 bits is enough */
	/* But in this application, it is not acceptable because the first character would change */
	sus_tx_data_index = 1;

	/* Reset Chat App state */
	suc_app_state = APP_STATE_IDLE;

	_show_menu(false);
}

/**
 * \brief Initialization of Chat module.
 *
 */
void app_console_init(void)
{
	phy_ctl_callbacks_t x_phy_ctl_cbs;

	x_phy_ctl_cbs.phy_ctl_data_confirm = app_console_handle_tx_cfm;
	x_phy_ctl_cbs.phy_ctl_rx_msg_discarded = app_console_handle_rx_msg_discarded;
	x_phy_ctl_cbs.phy_ctl_rx_msg = app_console_handle_rx_msg;
	x_phy_ctl_cbs.phy_ctl_update_tx_configuration = app_console_handle_upd_tx_cfg;

	/* Set PHY controller callbacks */
	phy_ctl_set_callbacks(&x_phy_ctl_cbs);

	/* Initialize Chat App state */
	suc_app_state = APP_STATE_IDLE;
}

/**
 * \brief Chat module process. Process Serial port input.
 */
void app_console_process(void)
{
	/* Process Chat input */
	uint8_t uc_char;
	bool b_send_plc_msg;
	enum mod_schemes uc_scheme;
	bool b_valid_modulation;
	uint8_t uc_result;
	uint8_t uc_channel;
	uint8_t uc_current_channel;
	bool b_channel_changed;

	if (suc_app_state == APP_STATE_TRANSMITTING) {
		/* If transmitting, wait to end transmission to process incoming characters */
		return;
	}

	if (!_read_char((uint8_t *)&uc_char)) {
		/* Show received character if it is printable (ASCII) */
		if (((uc_char >= 32) && (uc_char <= 126)) || ((uc_char >= 128) && (uc_char <= 254)) || (uc_char == '\t') || (uc_char == '\r') || (uc_char == '\n')) {
			printf("%c", uc_char);
			fflush(stdout);
		}

		switch (suc_app_state) {
		case APP_STATE_IDLE:
			switch (uc_char) {
			case 0x13:
				/* Special character: 0x13 (CTRL+S). Enter configuration menu */
				suc_app_state = APP_STATE_CONFIG_MENU;
				_show_menu(true);
				break;

			case '\r':
			case '\n':
				/* Special character: '\r' (Carriage Return) or '\n' (Line Feed). Show menu */
				_show_menu(true);
				break;

			default:
				/* Normal character: Start to type message if it is printable (ASCII) */
				if (((uc_char >= 32) && (uc_char <= 126)) || ((uc_char >= 128) && (uc_char <= 254)) || (uc_char == '\t')) {
					suc_app_state = APP_STATE_TYPING;
				}
			}

			if (suc_app_state != APP_STATE_TYPING) {
				break;
			}

		case APP_STATE_TYPING:
			b_send_plc_msg = false;

			switch (uc_char) {
			case '\r':
				/* Special character: '\r' (carriage return). Send message through PLC */
				b_send_plc_msg = true;
				break;

			case '\b':
			case 0x7F:
				/* Special character: '\b' (backspace) or 0x7F (DEL). Remove character from Tx data buffer */
				if (sus_tx_data_index > 1) {
					sus_tx_data_index--;
					printf("\b \b");
					fflush(stdout);
				}

				if (sus_tx_data_index == 1) {
					/* All text has been removed: Go back to idle state */
					suc_app_state = APP_STATE_IDLE;
				}

				break;

			default:
				/* Normal character: Add to Tx data buffer if it is printable (ASCII) */
				if (((uc_char >= 32) && (uc_char <= 126)) || ((uc_char >= 128) && (uc_char <= 254)) || (uc_char == '\t') || (uc_char == '\n')) {
					spuc_tx_data_buff[sus_tx_data_index++] = uc_char;
					if (sus_tx_data_index == (sus_max_data_len - 4)) {
						/* Maximum data length reached: Send message through PLC. 4 bytes corresponding to CRC discounted */
						printf("\r\nMax data length reached... Message will be sent\r\n");
						b_send_plc_msg = true;
					}
				}
			}

			if (b_send_plc_msg) {
				/* Send PLC message */
				uc_result = phy_ctl_send_msg(spuc_tx_data_buff, sus_tx_data_index);
				switch (uc_result) {
				case TX_RESULT_PROCESS:
					printf("\r\nTx (%u bytes): ", (uint32_t)(sus_tx_data_index - 1));
					break;

				case TX_RESULT_INV_LENGTH:
					printf("\r\nError sending PLC message: TX_RESULT_INV_LENGTH\r\n");
					break;

				case TX_RESULT_NO_TX:
					printf("\r\nError sending PLC message: TX_RESULT_NO_TX\r\n");
					break;

				case TX_RESULT_HIGH_TEMP_110:
					printf("\r\nError sending PLC message: TX_RESULT_HIGH_TEMP_110\r\n");
					break;

				default:
					printf("\r\nError sending PLC message: Unknown Error\r\n");
					break;
				}

				suc_app_state = APP_STATE_TRANSMITTING;
			}

			break;

		case APP_STATE_CONFIG_MENU:
			switch (uc_char) {
			case '0':
				/* Modulation configuration */
				suc_app_state = APP_STATE_CONFIG_MOD;
				_show_menu(true);
				break;

			case '1':
				/* Band configuration */
				suc_app_state = APP_STATE_CONFIG_CHANNEL;
				_show_menu(true);
				break;

			default:
				printf("\r\nUnknown command. Skipping configuration\r\n");
				suc_app_state = APP_STATE_IDLE;
				break;
			}

			break;

		case APP_STATE_CONFIG_MOD:
			b_valid_modulation = true;
			switch (uc_char) {
			case '0':
				/* R_DBPSK */
				uc_scheme = MOD_SCHEME_R_DBPSK;
				break;

			case '1':
				/* R_DQPSK */
				uc_scheme = MOD_SCHEME_R_DQPSK;
				break;

			case '2':
				/* DBPSK_C */
				uc_scheme = MOD_SCHEME_DBPSK_C;
				break;

			case '3':
				/* DQPSK_C */
				uc_scheme = MOD_SCHEME_DQPSK_C;
				break;

			case '4':
				/* D8PSK_C */
				uc_scheme = MOD_SCHEME_D8PSK_C;
				break;

			case '5':
				/* DBPSK */
				uc_scheme = MOD_SCHEME_DBPSK;
				break;

			case '6':
				/* DQPSK */
				uc_scheme = MOD_SCHEME_DQPSK;
				break;

			case '7':
				/* D8PSK */
				uc_scheme = MOD_SCHEME_D8PSK;
				break;

			default:
				printf("\r\nUnknown command. Skipping configuration\r\n");
				b_valid_modulation = false;
				break;
			}

			if (b_valid_modulation) {
				/* Set new Modulation. Get maximum data length allowed with configured Tx Parameters */
				sus_max_data_len = phy_ctl_set_mod_scheme(uc_scheme);
				_show_tx_parameters(uc_scheme);
			}

			suc_app_state = APP_STATE_IDLE;
			_show_menu(false);
			break;

		case APP_STATE_CONFIG_CHANNEL:
			switch (uc_char) {
			case '1':
				/* Channel 1 */
				uc_channel = 1;
				break;

			case '2':
				/* Channel 2 */
				uc_channel = 2;
				break;

			case '3':
				/* Channel 3 */
				uc_channel = 3;
				break;

			case '4':
				/* Channel 4 */
				uc_channel = 4;
				break;

			case '5':
				/* Channel 5 */
				uc_channel = 5;
				break;

			case '6':
				/* Channel 6 */
				uc_channel = 6;
				break;

			case '7':
				/* Channel 7 */
				uc_channel = 7;
				break;

			case '8':
				/* Channel 8 */
				uc_channel = 8;
				break;

			default:
				printf("\r\nUnknown command. Skipping configuration\r\n");
				uc_channel = 0;
				break;
			}

			if (uc_channel != 0) {
				if (USER_BAND_PLAN & (1 << (uc_channel - 1))) {
					/* Channel allowed */
					uc_current_channel = phy_ctl_get_channel();
					if (uc_channel == uc_current_channel) {
						printf("\r\nChannel has not changed: Skipping Channel Reconfiguration\r\n");
					} else {
						/* Set new channel on PL360 PHY */
						printf("\r\n");
						b_channel_changed = phy_ctl_set_channel(uc_channel);
						if (b_channel_changed) {
							switch (uc_channel) {
							case 1:
								printf("Channel 1 (42 - 89 kHz, CENELEC-A)\r\n");
								break;

							case 2:
								printf("Channel 2 (97 - 144 kHz, CENELEC-BCD)\r\n");
								break;

							case 3:
								printf("Channel 3 (151 - 198 kHz, FCC)\r\n");
								break;

							case 4:
								printf("Channel 4 (206 - 253 kHz, FCC)\r\n");
								break;

							case 5:
								printf("Channel 5 (261 - 308 kHz, FCC)\r\n");
								break;

							case 6:
								printf("Channel 6 (315 - 362 kHz, FCC)\r\n");
								break;

							case 7:
								printf("Channel 7 (370 - 417 kHz, FCC)\r\n");
								break;

							case 8:
								printf("Channel 8 (425 - 472 kHz, FCC)\r\n");
								break;

							default:
								printf("Unknown channel)\r\n");
								break;
							}
						} else {
							printf("\r\nInvalid channel: Skipping configuration\r\n");
						}
					}
				} else {
					printf("\r\nChannel not allowed (see USER_BAND_PLAN in 'conf_app_example.h'). Skipping configuration\r\n");
				}
			}

			suc_app_state = APP_STATE_IDLE;

			_show_menu(false);
			break;
		}
	}
}
