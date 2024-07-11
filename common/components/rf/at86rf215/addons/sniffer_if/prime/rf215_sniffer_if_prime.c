/**
 *
 * \file
 *
 * \brief RF215 PRIME sniffer addon.
 *
 * Copyright (c) 2024 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

/* RF215 includes */
#include "at86rf.h"
#include "rf215_addon.h"
#include "rf215_sniffer_if.h"
#include "rf215_phy_defs.h"

/* System includes */
#include "string.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PRIME sniffer identifiers and version */
#define SNIFFER_PL360_PRIME                             0x11
#define SNIFFER_PL360_PRIME_EXTENDED                    0x31
#define SNIFFER_PL360_PRIME_SIMULATOR                   0xD1
#define SNIFFER_RF215_PRIME                             0x13
#define SNIFFER_RF215_PRIME_EXTENDED                    0x33
#define SNIFFER_RF215_PRIME_SIMULATOR                   0xD3

#define SNIFFER_VERSION                                 0x14

/* PRIME sniffer commands identifiers */
#define SNIFFER_IF_PHY_COMMAND_SET_PLC_CHANNEL           2     /* Set PLC channel (1 = CENELEC-A) */
#define SNIFFER_IF_PHY_COMMAND_SET_RF_BAND_OPM_CHANNEL   3     /* Set RF band, operating mode and channel */

/* PRIME sniffer message types */
#define SNIFFER_IF_PHY_MESSAGE_TYPE_A                   0x20
#define SNIFFER_IF_PHY_MESSAGE_TYPE_B                   0x21
#define SNIFFER_IF_PHY_MESSAGE_TYPE_BC                  0x22
#define SNIFFER_IF_PHY_MESSAGE_TYPE_RF_FSK50            0x23
#define SNIFFER_IF_PHY_MESSAGE_TYPE_RF_FSK100           0x24
#define SNIFFER_IF_PHY_MESSAGE_TYPE_RF_FSK150           0x25
#define SNIFFER_IF_PHY_MESSAGE_TYPE_RF_FSK200           0x26
#define SNIFFER_IF_PHY_MESSAGE_TYPE_RF_FSK300           0x27
#define SNIFFER_IF_PHY_MESSAGE_TYPE_RF_FSK400           0x28
#define SNIFFER_IF_PHY_MESSAGE_TYPE_RF_4FSK50           0x29
#define SNIFFER_IF_PHY_MESSAGE_TYPE_RF_4FSK100          0x2A
#define SNIFFER_IF_PHY_MESSAGE_TYPE_RF_4FSK150          0x2B
#define SNIFFER_IF_PHY_MESSAGE_TYPE_RF_4FSK200          0x2C
#define SNIFFER_IF_PHY_MESSAGE_TYPE_RF_4FSK300          0x2D
#define SNIFFER_IF_PHY_MESSAGE_TYPE_RF_4FSK400          0x2E
#define SNIFFER_IF_PHY_MESSAGE_TYPE_RF_OFDM1            0x2F
#define SNIFFER_IF_PHY_MESSAGE_TYPE_RF_OFDM2            0x30
#define SNIFFER_IF_PHY_MESSAGE_TYPE_RF_OFDM3            0x31
#define SNIFFER_IF_PHY_MESSAGE_TYPE_RF_OFDM4            0x32

/* PRIME sniffer PHY modulations */
#define SNIFFER_IF_PHY_MESSAGE_DBPSK                    0x00
#define SNIFFER_IF_PHY_MESSAGE_DQPSK                    0x01
#define SNIFFER_IF_PHY_MESSAGE_D8PSK                    0x02
#define SNIFFER_IF_PHY_MESSAGE_DBPSK_C                  0x04
#define SNIFFER_IF_PHY_MESSAGE_DQPSK_C                  0x05
#define SNIFFER_IF_PHY_MESSAGE_D8PSK_C                  0x06
#define SNIFFER_IF_PHY_MESSAGE_R_DBPSK                  0x0C
#define SNIFFER_IF_PHY_MESSAGE_R_DQPSK                  0x0D
#define SNIFFER_IF_PHY_MESSAGE_R_DQPSK                  0x0D
#define SNIFFER_IF_PHY_MESSAGE_RF_FSK_FEC_OFF           0x10
#define SNIFFER_IF_PHY_MESSAGE_RF_FSK_FEC_ON            0x11
#define SNIFFER_IF_PHY_MESSAGE_RF_OFDM_MCS0             0x12
#define SNIFFER_IF_PHY_MESSAGE_RF_OFDM_MCS1             0x13
#define SNIFFER_IF_PHY_MESSAGE_RF_OFDM_MCS2             0x14
#define SNIFFER_IF_PHY_MESSAGE_RF_OFDM_MCS3             0x15
#define SNIFFER_IF_PHY_MESSAGE_RF_OFDM_MCS4             0x16
#define SNIFFER_IF_PHY_MESSAGE_RF_OFDM_MCS5             0x17
#define SNIFFER_IF_PHY_MESSAGE_RF_OFDM_MCS6             0x18

#ifdef AT86RF_ADDONS_ENABLE

static uint8_t _rf215_sniffer_frame_type(uint8_t uc_trx_id)
{
	at86rf_phy_cfg_t *px_phy_cfg;
	uint8_t uc_type;

	/* Frame type depending on RF PHY configuration */
	px_phy_cfg = &gpx_phy_cfg[uc_trx_id];
	if (px_phy_cfg->uc_phy_mod == AT86RF_PHY_MOD_FSK) {
		at86rf_fsk_cfg_t *px_fsk_cfg = &px_phy_cfg->u_mod_cfg.x_fsk;
		uc_type = (uint8_t)px_fsk_cfg->uc_symrate;
		if (px_fsk_cfg->uc_modord == AT86RF_FSK_MODORD_2FSK) {
			uc_type += SNIFFER_IF_PHY_MESSAGE_TYPE_RF_FSK50;
		} else { /* AT86RF_FSK_MODORD_4FSK */
			uc_type += SNIFFER_IF_PHY_MESSAGE_TYPE_RF_4FSK50;
		}
	} else { /* AT86RF_PHY_MOD_OFDM */
		uc_type = (uint8_t)px_phy_cfg->u_mod_cfg.x_ofdm.uc_opt;
		uc_type += SNIFFER_IF_PHY_MESSAGE_TYPE_RF_OFDM1;
	}

	return uc_type;
}

/**
 * \brief Converts PHY RX indication struct to byte buffer for PRIME sniffer
 *
 * \param uc_trx_id TRX identifier
 * \param puc_dst_buf Pointer to destination buffer
 * \param px_rx_ind Pointer to RX indication struct
 *
 * \return Length of sniffer data buffer
 */
uint16_t rf215_sniffer_if_stringify_ind(uint8_t uc_trx_id, uint8_t *puc_dst_buf, at86rf_rx_ind_t *px_rx_ind)
{
	uint32_t ul_time_ini;
	uint32_t ul_time_end;
	uint16_t us_len;
	uint16_t us_channel;
	uint16_t us_pay_symbols;
	int16_t ss_rssi;

	/* Frame type depending on RF PHY configuration */
	puc_dst_buf[0] = _rf215_sniffer_frame_type(uc_trx_id);

	/* Sniffer version and sniffer type */
	puc_dst_buf[1] = (uint8_t)SNIFFER_VERSION;
	puc_dst_buf[2] = (uint8_t)SNIFFER_RF215_PRIME;

	/* Frame modulation depending on RF PHY configuration */
	puc_dst_buf[3] = (uint8_t)px_rx_ind->x_mod_params.x_ofdm.uc_mcs;
	if (gpx_phy_cfg[uc_trx_id].uc_phy_mod == AT86RF_PHY_MOD_FSK) {
		/* FSK modulation (FEC on/off) */
		puc_dst_buf[3] += SNIFFER_IF_PHY_MESSAGE_RF_FSK_FEC_OFF;
	} else {
		/* OFDM modulation (MCS) */
		puc_dst_buf[3] += SNIFFER_IF_PHY_MESSAGE_RF_OFDM_MCS0;
	}

	/* Number of payload symbols */
	us_pay_symbols = gpx_phy_ctl[uc_trx_id].us_rx_pay_symbols;
	puc_dst_buf[4] = (uint8_t)(us_pay_symbols >> 8);
	puc_dst_buf[5] = (uint8_t)(us_pay_symbols);

	/* Channel */
	us_channel = px_rx_ind->us_chn_num;
	puc_dst_buf[6] = (uint8_t)(us_channel >> 8);
	puc_dst_buf[7] = (uint8_t)(us_channel);

	/* Initial and end time of RX frame */
	ul_time_ini = px_rx_ind->ul_rx_time_ini;
	ul_time_end = ul_time_ini + px_rx_ind->ul_frame_duration;
	puc_dst_buf[19] = (uint8_t)(ul_time_ini >> 24);
	puc_dst_buf[20] = (uint8_t)(ul_time_ini >> 16);
	puc_dst_buf[21] = (uint8_t)(ul_time_ini >> 8);
	puc_dst_buf[22] = (uint8_t)(ul_time_ini);
	puc_dst_buf[23] = (uint8_t)(ul_time_end >> 24);
	puc_dst_buf[24] = (uint8_t)(ul_time_end >> 16);
	puc_dst_buf[25] = (uint8_t)(ul_time_end >> 8);
	puc_dst_buf[26] = (uint8_t)(ul_time_end);

	/* RSSI */
	ss_rssi = (int16_t)px_rx_ind->sc_rssi_dBm;
	puc_dst_buf[27] = (uint8_t)(ss_rssi >> 8);
	puc_dst_buf[28] = (uint8_t)(ss_rssi);

	/* mac_enable not supported */
	puc_dst_buf[29] = 0;

	/* Data PSDU length (including PRIME CRC) */
	us_len = px_rx_ind->us_psdu_len;
	puc_dst_buf[30] = (uint8_t)(us_len >> 8);
	puc_dst_buf[31] = (uint8_t)(us_len);

	/* Copy PHY data message */
	memcpy(puc_dst_buf + RF215_SNIFFER_MSG_HEADER_LEN, px_rx_ind->puc_data, us_len);

	return us_len + RF215_SNIFFER_MSG_HEADER_LEN;
}

/**
 * \brief Converts PHY TX confirm struct to byte buffer for PRIME sniffer
 *
 * \param uc_trx_id TRX identifier
 * \param puc_dst_buf Pointer to destination buffer
 * \param px_tx_cfm Pointer to TX confirm struct
 */
void rf215_sniffer_if_stringify_cfm(uint8_t uc_trx_id, uint8_t *puc_dst_buf, at86rf_tx_cfm_t *px_tx_cfm)
{
	uint32_t ul_time_ini;
	uint32_t ul_time_end;
	uint16_t us_pay_symbols;

	/* Frame type depending on RF PHY configuration */
	puc_dst_buf[0] = _rf215_sniffer_frame_type(uc_trx_id);

	/* Sniffer version and sniffer type */
	puc_dst_buf[1] = (uint8_t)SNIFFER_VERSION;
	puc_dst_buf[2] = (uint8_t)SNIFFER_RF215_PRIME;

	/* Frame modulation depending on RF PHY configuration */
	if (gpx_phy_cfg[uc_trx_id].uc_phy_mod == AT86RF_PHY_MOD_FSK) {
		/* FSK modulation (FEC on/off) */
		puc_dst_buf[3] += SNIFFER_IF_PHY_MESSAGE_RF_FSK_FEC_OFF;
	} else {
		/* OFDM modulation (MCS) */
		puc_dst_buf[3] += SNIFFER_IF_PHY_MESSAGE_RF_OFDM_MCS0;
	}

	/* Number of payload symbols */
	us_pay_symbols = gpx_phy_ctl[uc_trx_id].us_tx_pay_symbols;
	puc_dst_buf[4] = (uint8_t)(us_pay_symbols >> 8);
	puc_dst_buf[5] = (uint8_t)(us_pay_symbols);

	/* Initial and end time of RX frame */
	ul_time_ini = px_tx_cfm->ul_tx_time_ini;
	ul_time_end = ul_time_ini + px_tx_cfm->ul_frame_duration;
	puc_dst_buf[19] = (uint8_t)(ul_time_ini >> 24);
	puc_dst_buf[20] = (uint8_t)(ul_time_ini >> 16);
	puc_dst_buf[21] = (uint8_t)(ul_time_ini >> 8);
	puc_dst_buf[22] = (uint8_t)(ul_time_ini);
	puc_dst_buf[23] = (uint8_t)(ul_time_end >> 24);
	puc_dst_buf[24] = (uint8_t)(ul_time_end >> 16);
	puc_dst_buf[25] = (uint8_t)(ul_time_end >> 8);
	puc_dst_buf[26] = (uint8_t)(ul_time_end);
}

/**
 * \brief Converts PHY TX request struct to byte buffer for PRIME sniffer
 *
 * \param puc_dst_buf Pointer to destination buffer
 * \param px_tx_params Pointer to TX parameters
 *
 * \return Length of sniffer data buffer
 */
uint16_t rf215_sniffer_if_stringify_tx(uint8_t *puc_dst_buf, at86rf_tx_params_t *px_tx_params)
{
	uint16_t us_len;
	uint16_t us_channel;
	int16_t ss_rssi;

	/* Store modulation frame parameter */
	puc_dst_buf[3] = (uint8_t)px_tx_params->x_mod_params.x_ofdm.uc_mcs;

	/* Channel */
	us_channel = px_tx_params->us_chn_num;
	puc_dst_buf[6] = (uint8_t)(us_channel >> 8);
	puc_dst_buf[7] = (uint8_t)(us_channel);

	/* RSSI */
	ss_rssi = 14 - (int16_t)px_tx_params->uc_txpwr_att;
	puc_dst_buf[27] = (uint8_t)(ss_rssi >> 8);
	puc_dst_buf[28] = (uint8_t)(ss_rssi);

	/* mac_enable not supported */
	puc_dst_buf[29] = 0;

	/* Data PSDU length (including PRIME CRC) */
	us_len = px_tx_params->us_psdu_len;
	puc_dst_buf[30] = (uint8_t)(us_len >> 8);
	puc_dst_buf[31] = (uint8_t)(us_len);

	/* Copy PHY data message */
	memcpy(puc_dst_buf + RF215_SNIFFER_MSG_HEADER_LEN, px_tx_params->puc_data, us_len);

	return us_len + RF215_SNIFFER_MSG_HEADER_LEN;
}

/**
 * \brief Function to execute addons commnad
 *
 * \param puc_rx_msg  Pointer to command message
 * \param us_len  Length of the command message
 *
 */
void at86rf_addon_command(uint8_t *puc_rx_msg, uint16_t us_len)
{
	uint16_t us_band_opmode, us_channel;
	uint8_t uc_sniffer_if_cmd;

	/* Protection for invalid length */
	if (!us_len) {
		return;
	}

	/* Process received message. Get command */
	uc_sniffer_if_cmd  = puc_rx_msg[0];

	switch (uc_sniffer_if_cmd) {
	case SNIFFER_IF_PHY_COMMAND_SET_RF_BAND_OPM_CHANNEL:
		/* Set RF band, operating mode and channel */
		us_band_opmode  = (puc_rx_msg[1] << 8) | puc_rx_msg[2];
		us_channel  = (puc_rx_msg[3] << 8) | puc_rx_msg[4];
		at86rf_pib_set(AT86RF_TRX_RF09_ID, AT86RF_PIB_PHY_BAND_OPERATING_MODE, &us_band_opmode);
		at86rf_pib_set(AT86RF_TRX_RF09_ID, AT86RF_PIB_PHY_CHANNEL_NUM, &us_channel);
		break;

	case SNIFFER_IF_PHY_COMMAND_SET_PLC_CHANNEL:
	default:
		/* Not supported */
		break;
	}
}

#endif

#ifdef __cplusplus
}
#endif
