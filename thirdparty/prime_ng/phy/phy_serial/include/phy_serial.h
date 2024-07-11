/**
 * \file
 *
 * \brief PHYSER : Serial Port Physical layer
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

#ifndef PHY_SERIAL_H
#define PHY_SERIAL_H

#include "conf_phy_serial.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

/* Maximum physical pdu size */
#define PHY_MAX_PPDU_SIZE                       512
/** Number of transmission buffers */
#define PHY_NUM_TX_BUFFERS                      4

#define PHY_SER_GET_HEADER_TYPE(val)         ((val >> 4) & 0x03)

/* ! \name PRIME Mode types */
enum serial_mode_types {
	PHY_SERIAL_MODE_TYPE_A = 0,
	PHY_SERIAL_MODE_TYPE_B = 2,
	PHY_SERIAL_MODE_TYPE_BC = 3,
};

/** Serial channel */
#define SERIAL_CHANNEL        0x400

/* @} */
/** \brief TX Result values */
/* @{ */
/** Transmission result: already in process */
#define PHY_SER_TX_RESULT_PROCESS                   0
/** Transmission result: end successfully */
#define PHY_SER_TX_RESULT_SUCCESS                   1
/** Transmission result: invalid length error */
#define PHY_SER_TX_RESULT_INV_LENGTH                2
/** Transmission result: busy channel error */
#define PHY_SER_TX_RESULT_BUSY_CH                   3
/** Transmission result: busy transmission error */
#define PHY_SER_TX_RESULT_BUSY_TX                   4
/** Transmission result: busy reception error */
#define PHY_SER_TX_RESULT_BUSY_RX                   5
/** Transmission result: invalid scheme error */
#define PHY_SER_TX_RESULT_INV_SCHEME                6
/** Transmission result: timeout error */
#define PHY_SER_TX_RESULT_TIMEOUT                   7
/** Transmission result: invalid buffer identifier error */
#define PHY_SER_TX_RESULT_INV_BUFFER                8
/** Transmission result: invalid Prime Mode error */
#define PHY_SER_TX_RESULT_INV_PRIME_MODE            9
/* @} */

/** \brief Data struct used for SERIAL transmission */
typedef struct {
	/** Pointer to data buffer */
	uint8_t *data_buf;
	/** Delay for transmission in us */
	uint32_t time_delay;
	/** Length of the data buffer. */
	uint16_t data_len;
	/** Physical channel to transmit the message */
	uint16_t us_pch;
	/** Buffer identifier */
	uint8_t buff_id;
	/** Attenuation level with which the message must be transmitted */
	uint8_t att_level; /* NOT USED: keep for backwards compatibility */
	/** Modulation scheme of last transmitted message */
	uint8_t scheme; /* NOT USED: keep for backwards compatibility */
	/** TX Forced */
	uint8_t disable_rx; /* NOT USED: keep for backwards compatibility */
	/** Type A, Type B, Type BC, Type Radio */
	uint8_t mode; /* NOT USED: keep for backwards compatibility */
	/** Time mode: 0: Absolute mode, 1: Differential mode, 2: Cancel TX */
	uint8_t time_mode; /* NOT USED: keep for backwards compatibility */
	/** Number of channel senses */
	uint8_t num_senses; /* NOT USED: keep for backwards compatibility */
	/** Delay between channel senses in ms */
	uint8_t sense_delay_ms; /* NOT USED: keep for backwards compatibility */
} xPhySerMsgTx_t;

/** Data struct used for PLC confirm transmission */
typedef struct {
	/** Transmission time in 10us. */
	uint32_t tx_time;
	/** RMS value emitted (valid only when txQRMode is enable) */
	uint16_t rms_calc;
	/** Type mode: Type A, Type B or Type BC  */
	uint8_t mode;
	/** Number of the buffer used to tx */
	uint8_t buff_id;
	/** Result (see atpl230reg.h "Values of TXRXBUF_RESULT_TX register" ) */
	uint8_t result;
} xPhySerMsgTxResult_t;

/** Data struct used for PLC reception */
typedef struct {
	/** Pointer to local data buffer */
	uint8_t *data_buf;
	/** Accumulated Error Vector Magnitude for header (valid only when rxQRMode is enable) */
	uint32_t evm_header_acum;
	/** Accumulated Error Vector Magnitude for payload (valid only when rxQRMode is enable) */
	uint32_t evm_payload_acum;
	/** Reception time in 10us */
	uint32_t rx_time;
	/** Error Vector Magnitude for header (valid only when rxQRMode is enable) */
	uint16_t evm_header;
	/** Error Vector Magnitude for payload (valid only when rxQRMode is enable) */
	uint16_t evm_payload;
	/** Length of the data buffer. */
	uint16_t data_len;
	/** Buffer identifier */
	uint8_t buff_id;
	/** Modulation scheme of the last received message */
	uint8_t scheme;
	/** Type A, Type B or Type BC  */
	uint8_t mode;
	/** Header Type of the last received message */
	uint8_t header_type;
	/** Noise result in case of noise capture mode */
	uint8_t noise_result;
	/** Average RSSI (Received Signal Strength Indication) (valid only when rxQRMode is enable) */
	uint8_t rssi_avg;
	/** Average CNIR (Carrier to Interference + Noise ratio) (valid only when rxQRMode is enable) */
	uint8_t cinr_avg;
	/** Minimum CNIR (Carrier to Interference + Noise ratio) (valid only when rxQRMode is enable) */
	uint8_t cinr_min;
	/** Viterbi soft Bit Error Rate value (valid only when rxQRMode is enable) */
	uint8_t bersoft;
	/** Viterbi soft Bit Error Rate Maximum value (valid only when rxQRMode is enable) */
	uint8_t bersoft_max;
	/** QT value */
	uint8_t qt;
	/** Extended signal noise ratio value */
	uint8_t snr_ex;
} xPhySerMsgRx_t;

/** PHY confirm transmission callback */
typedef void (*phy_ser_tx_result_cb_t)(xPhySerMsgTxResult_t *px_tx_result);

/** PHY reception callback */
typedef void (*phy_ser_rx_message_cb_t)(xPhySerMsgRx_t *px_msg);

/** PHY addon callback */
typedef void (*phy_ser_addon_cb)(uint8_t *puc_msg, uint16_t us_len);

/** Data struct used for Phy callbacks */
typedef struct TPhySerCallbacks {
	phy_ser_tx_result_cb_t phy_ser_data_confirm;
	phy_ser_rx_message_cb_t phy_ser_data_indication;
	phy_ser_addon_cb phy_ser_addon_event;
} phy_serial_callbacks_t;

/** \brief Serial Physical Layer Interface */
/* @{ */
void phy_serial_init(void);
void phy_serial_process(void);
void phy_serial_reset(uint8_t uc_reset_type);
void phy_serial_set_callbacks(phy_serial_callbacks_t *phy_ser_cbs);
uint8_t phy_serial_tx_frame(xPhySerMsgTx_t *px_msg);

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */
#endif /* PHY_SERIAL_H */
