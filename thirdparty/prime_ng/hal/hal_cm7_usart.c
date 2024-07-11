/**
 * \file
 *
 * \brief HAL_USART
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
#include <string.h>
#include "asf.h"
#include "conf_board.h"
#include "hal_private.h"

#if !(SAMV71 || SAMV70 || SAME70 || SAMS70)
#  error No vaild platform.
#endif

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

/**
 * \weakgroup busart_plc_group
 * @{
 */

/* XDMA channel used for BUART */
#define BUSART_XDMAC_USART0_CH_TX 6
#define BUSART_XDMAC_USART0_CH_RX 7
#define BUSART_XDMAC_USART1_CH_TX 8
#define BUSART_XDMAC_USART1_CH_RX 9
#define BUSART_XDMAC_USART2_CH_TX 10
#define BUSART_XDMAC_USART2_CH_RX 11
/* XDMAC channel status  */
#define BUSART_XDMAC_USART0_TX_STATUS (1 << BUSART_XDMAC_USART0_CH_TX)
#define BUSART_XDMAC_USART0_RX_STATUS (1 << BUSART_XDMAC_USART0_CH_RX)
#define BUSART_XDMAC_USART1_TX_STATUS (1 << BUSART_XDMAC_USART1_CH_TX)
#define BUSART_XDMAC_USART1_RX_STATUS (1 << BUSART_XDMAC_USART1_CH_RX)
#define BUSART_XDMAC_USART2_TX_STATUS (1 << BUSART_XDMAC_USART2_CH_TX)
#define BUSART_XDMAC_USART2_RX_STATUS (1 << BUSART_XDMAC_USART2_CH_RX)


#ifdef CONF_BOARD_USART0_RXD
/* Reception Buffer 0 */
static uint8_t rx_usart_buf0[HAL_RX_USART_BUF0_SIZE];
/* Transmission Buffer 0 */
static uint8_t tx_usart_buf0[HAL_TX_USART_BUF0_SIZE];
/* Pointers to Reception Buffer 0 */
COMPILER_ALIGNED(4)
uint8_t *const ptr_rx_usart_buf0 = &rx_usart_buf0[0];
/* Pointers to Transmission Buffer 0 */
COMPILER_ALIGNED(4)
uint8_t *const ptr_tx_usart_buf0 = &tx_usart_buf0[0];

/* USART0 XDMAC peripheral IDs */
#define BUSART_XDMAC_USART0_TX_PERID  7
#define BUSART_XDMAC_USART0_RX_PERID  8
/** XDMA channel configuration. */
static xdmac_channel_config_t busart0_xdmac_tx_channel_cfg;
static xdmac_channel_config_t busart0_xdmac_rx_channel_cfg;

#endif

#ifdef CONF_BOARD_USART1_RXD
/* Reception Buffer 1 */
static uint8_t rx_usart_buf1[HAL_RX_USART_BUF1_SIZE];
/* Transmission Buffer 1 */
static uint8_t tx_usart_buf1[HAL_TX_USART_BUF1_SIZE];
/* Pointers to Reception Buffer 1 */
COMPILER_ALIGNED(4)
uint8_t *const ptr_rx_usart_buf1 = &rx_usart_buf1[0];
/* Pointers to Transmission Buffer 1 */
COMPILER_ALIGNED(4)
uint8_t *const ptr_tx_usart_buf1 = &tx_usart_buf1[0];
/* USART1 XDMAC peripheral IDs */
#define BUSART_XDMAC_USART1_TX_PERID 9
#define BUSART_XDMAC_USART1_RX_PERID 10
/** XDMA channel configuration. */
static xdmac_channel_config_t busart1_xdmac_tx_channel_cfg;
static xdmac_channel_config_t busart1_xdmac_rx_channel_cfg;
#endif

#ifdef CONF_BOARD_USART2_RXD
/* Reception Buffer 1 */
static uint8_t rx_usart_buf2[HAL_RX_USART_BUF2_SIZE];
/* Transmission Buffer 1 */
static uint8_t tx_usart_buf2[HAL_TX_USART_BUF2_SIZE];
/* Pointers to Reception Buffer 1 */
COMPILER_ALIGNED(4)
uint8_t *const ptr_rx_usart_buf2 = &rx_usart_buf2[0];
/* Pointers to Transmission Buffer 1 */
COMPILER_ALIGNED(4)
uint8_t *const ptr_tx_usart_buf2 = &tx_usart_buf2[0];
/* USART2 XDMAC peripheral IDs */
#define BUSART_XDMAC_USART2_TX_PERID 11
#define BUSART_XDMAC_USART2_RX_PERID 12
/** XDMA channel configuration. */
static xdmac_channel_config_t busart2_xdmac_tx_channel_cfg;
static xdmac_channel_config_t busart2_xdmac_rx_channel_cfg;
#endif

/*! \brief Communications Queue Info */
typedef struct {
	/** Pointer to transmission queue. Buffer */
	uint8_t *puc_tq_buf;
	/** Pointer to reception queue. Buffer */
	uint8_t *puc_rq_buf;
	/** Reception queue. Read index */
	uint16_t us_rq_idx;
	/** Reception queue. Write index */
	uint16_t us_wq_idx;
	/** Reception queue. Occupation count */
	uint16_t us_rq_count;
} busart_comm_data_t;

/* \name Size of the receive buffer used by the PDC, in bytes */
/* @{ */
#define USART_BUFFER_SIZE                       1024
/* @} */


#ifdef CONF_BOARD_USART0_RXD
/* Data struct to use with USART0 */
static busart_comm_data_t busart_comm_data_0;
/* Receive buffer use 2 fast buffers */
COMPILER_ALIGNED(4)
static uint8_t gs_puc_usart_buf0[USART_BUFFER_SIZE];
/* Current bytes in buffer. */
static uint32_t gs_ul_size_usart_buf0 = USART_BUFFER_SIZE;
/* Number of bytes received in USART0 */
static uint16_t num_bytes_rx_usart0;
#endif

#ifdef CONF_BOARD_USART1_RXD
/* Data struct to use with USART0 */
static busart_comm_data_t busart_comm_data_1;
/* Receive buffer use 2 fast buffers */
COMPILER_ALIGNED(4)
static uint8_t gs_puc_usart_buf1[USART_BUFFER_SIZE];
/* Current bytes in buffer. */
static uint32_t gs_ul_size_usart_buf1 = USART_BUFFER_SIZE;
/* Number of bytes received in USART1 */
static uint16_t num_bytes_rx_usart1;
#endif

#ifdef CONF_BOARD_USART2_RXD
/* Data struct to use with USART0 */
static busart_comm_data_t busart_comm_data_2;
/* Receive buffer use 2 fast buffers */
COMPILER_ALIGNED(4)
static uint8_t gs_puc_usart_buf2[USART_BUFFER_SIZE];
/* Current bytes in buffer. */
static uint32_t gs_ul_size_usart_buf2 = USART_BUFFER_SIZE;
/* Number of bytes received in USART1 */
static uint16_t num_bytes_rx_usart2;
#endif

/* Uart channel open / closed */
static uint8_t busart_chn_open[3] = {
	false,
	false,
	false
};

#if defined(CONF_BOARD_USART0_RXD) || defined(CONF_BOARD_USART1_RXD) || defined(CONF_BOARD_USART2_RXD)

/**
 * \brief Configure Timer Counter to generate an interrupt every 10ms.
 * This interrupt will be used to flush USART input and echo back.
 */
static void _configure_TC_usart(void)
{
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk;
	uint32_t ul_frec_hz = (uint32_t)FREQ_TIMER_POLL_USART;

	/* Get system clock. */
	ul_sysclk = sysclk_get_peripheral_hz();

	/* Configure PMC. */
	pmc_enable_periph_clk(HAL_ID_TC_USART);

	/* Configure TC for a TC_FREQ frequency and trigger on RC compare. */
	tc_find_mck_divisor(ul_frec_hz, ul_sysclk, &ul_div, &ul_tcclks,
			ul_sysclk);
	tc_init(HAL_TC_USART, HAL_TC_USART_CHN, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(HAL_TC_USART, HAL_TC_USART_CHN, (ul_sysclk / ul_div) / ul_frec_hz);

	/* Configure and enable interrupt on RC compare. */
	NVIC_SetPriority((IRQn_Type)HAL_ID_TC_USART, TIMER_USART_PRIO);
	NVIC_EnableIRQ((IRQn_Type)HAL_ID_TC_USART);
	tc_enable_interrupt(HAL_TC_USART, HAL_TC_USART_CHN, TC_IER_CPCS);
}

#ifdef CONF_BOARD_USART0_RXD

/** @brief	Interrupt handler for USART0
 *
 */
static void _BUSART_RX0_buffer_proc(void)
{
	uint16_t us_wr_idx, us_data_count;
	uint16_t us_end_size, us_free_size, us_part_size;


	/* manage data */
	us_wr_idx = busart_comm_data_0.us_wq_idx;
	us_data_count = busart_comm_data_0.us_rq_count;
	us_free_size = HAL_RX_USART_BUF0_SIZE - us_data_count;
	if (gs_ul_size_usart_buf0 <= us_free_size) {
		/* there is enough space to write all data */
		us_end_size = HAL_RX_USART_BUF0_SIZE - us_wr_idx;
		if (us_end_size >= gs_ul_size_usart_buf0) {
			/* there is no overflow of us_wq_idx */
			memcpy(&busart_comm_data_0.puc_rq_buf[us_wr_idx], gs_puc_usart_buf0, gs_ul_size_usart_buf0);
			/* update counters */
			busart_comm_data_0.us_rq_count += gs_ul_size_usart_buf0;
			busart_comm_data_0.us_wq_idx += gs_ul_size_usart_buf0;
		} else {
			/* there is overflow of us_wq_idx -> write in 2
			 * steps	*/
			memcpy(&busart_comm_data_0.puc_rq_buf[us_wr_idx], gs_puc_usart_buf0, us_end_size);
			us_part_size = gs_ul_size_usart_buf0 - us_end_size;
			memcpy(&busart_comm_data_0.puc_rq_buf[0], &gs_puc_usart_buf0[us_end_size], us_part_size);
			/* update counters */
			busart_comm_data_0.us_rq_count += gs_ul_size_usart_buf0;
			busart_comm_data_0.us_wq_idx = us_part_size;
		}
	} else {
		/* there is not enough space to write all data */
		tc_start(HAL_TC_USART, HAL_TC_USART_CHN);
	}

	/* change RX buffer */
	gs_ul_size_usart_buf0 = USART_BUFFER_SIZE;

	busart0_xdmac_rx_channel_cfg.mbr_ubc = USART_BUFFER_SIZE;
	xdmac_configure_transfer(XDMAC, BUSART_XDMAC_USART0_CH_RX, &busart0_xdmac_rx_channel_cfg);
	xdmac_channel_set_descriptor_control(XDMAC, BUSART_XDMAC_USART0_CH_RX, 0);
	xdmac_channel_enable(XDMAC, BUSART_XDMAC_USART0_CH_RX);

	/* Restart timer. */
	tc_start(HAL_TC_USART, HAL_TC_USART_CHN);

}

#endif /* #ifdef CONF_BOARD_USART0_RXD */


#ifdef CONF_BOARD_USART1_RXD

/** @brief	Interrupt handler for USART1
 *
 */
static void _BUSART_RX1_buffer_proc(void)
{
	uint16_t us_wr_idx, us_data_count;
	uint16_t us_end_size, us_free_size, us_part_size;

	/* manage data */
	us_wr_idx = busart_comm_data_1.us_wq_idx;
	us_data_count = busart_comm_data_1.us_rq_count;
	us_free_size = HAL_RX_USART_BUF1_SIZE - us_data_count;
	if (gs_ul_size_usart_buf1 <= us_free_size) {
		/* there is enough space to write all data */
		us_end_size = HAL_RX_USART_BUF1_SIZE - us_wr_idx;
		if (us_end_size >= gs_ul_size_usart_buf1) {
			/* there is no overflow of us_wq_idx */
			memcpy(&busart_comm_data_1.puc_rq_buf[us_wr_idx], gs_puc_usart_buf1, gs_ul_size_usart_buf1);
			/* update counters */
			busart_comm_data_1.us_rq_count += gs_ul_size_usart_buf1;
			busart_comm_data_1.us_wq_idx += gs_ul_size_usart_buf1;
		} else {
			/* there is overflow of us_wq_idx -> write in 2
			 * steps	*/
			memcpy(&busart_comm_data_1.puc_rq_buf[us_wr_idx], gs_puc_usart_buf1, us_end_size);
			us_part_size = gs_ul_size_usart_buf1 - us_end_size;
			memcpy(&busart_comm_data_1.puc_rq_buf[0], &gs_puc_usart_buf1[us_end_size], us_part_size);
			/* update counters */
			busart_comm_data_1.us_rq_count += gs_ul_size_usart_buf1;
			busart_comm_data_1.us_wq_idx = us_part_size;
		}
	} else {
		/* there is not enough space to write all data */
		tc_start(HAL_TC_USART, HAL_TC_USART_CHN);
	}

	/* change RX buffer */
	gs_ul_size_usart_buf1 = USART_BUFFER_SIZE;

	/* Restart read on buffer. */
	busart1_xdmac_rx_channel_cfg.mbr_ubc = USART_BUFFER_SIZE;
	xdmac_configure_transfer(XDMAC, BUSART_XDMAC_USART1_CH_RX, &busart1_xdmac_rx_channel_cfg);
	xdmac_channel_set_descriptor_control(XDMAC, BUSART_XDMAC_USART1_CH_RX, 0);
	xdmac_channel_enable(XDMAC, BUSART_XDMAC_USART1_CH_RX);

	/* Restart timer. */
	tc_start(HAL_TC_USART, HAL_TC_USART_CHN);

}
#endif /* #ifdef CONF_BOARD_USART1_RXD */


#ifdef CONF_BOARD_USART2_RXD

/** @brief	Interrupt handler for USART1
 *
 */
static void _BUSART_RX2_buffer_proc(void)
{
	uint16_t us_wr_idx, us_data_count;
	uint16_t us_end_size, us_free_size, us_part_size;

	/* manage data */
	us_wr_idx = busart_comm_data_2.us_wq_idx;
	us_data_count = busart_comm_data_2.us_rq_count;
	us_free_size = HAL_RX_USART_BUF2_SIZE - us_data_count;
	if (gs_ul_size_usart_buf2 <= us_free_size) {
		/* there is enough space to write all data */
		us_end_size = HAL_RX_USART_BUF2_SIZE - us_wr_idx;

		if (us_end_size >= gs_ul_size_usart_buf2) {
			/* there is no overflow of us_wq_idx */
			memcpy(&busart_comm_data_2.puc_rq_buf[us_wr_idx], gs_puc_usart_buf2, gs_ul_size_usart_buf2);
			/* update counters */
			busart_comm_data_2.us_rq_count += gs_ul_size_usart_buf2;
			busart_comm_data_2.us_wq_idx += gs_ul_size_usart_buf2;
		} else {
			/* there is overflow of us_wq_idx -> write in 2
			 * steps	*/
			memcpy(&busart_comm_data_2.puc_rq_buf[us_wr_idx], gs_puc_usart_buf2, us_end_size);
			us_part_size = gs_ul_size_usart_buf2 - us_end_size;
			memcpy(&busart_comm_data_2.puc_rq_buf[0], &gs_puc_usart_buf2[us_end_size], us_part_size);
			/* update counters */
			busart_comm_data_2.us_rq_count += gs_ul_size_usart_buf2;
			busart_comm_data_2.us_wq_idx = us_part_size;
		}
	} else {
		/* there is not enough space to write all data */
		tc_start(HAL_TC_USART, HAL_TC_USART_CHN);
	}

	/* change RX buffer */
	gs_ul_size_usart_buf2 = USART_BUFFER_SIZE;

	/* Restart read on buffer. */
	busart1_xdmac_rx_channel_cfg.mbr_ubc = USART_BUFFER_SIZE;
	xdmac_configure_transfer(XDMAC, BUSART_XDMAC_USART1_CH_RX, &busart1_xdmac_rx_channel_cfg);
	xdmac_channel_set_descriptor_control(XDMAC, BUSART_XDMAC_USART1_CH_RX, 0);
	xdmac_channel_enable(XDMAC, BUSART_XDMAC_USART1_CH_RX);


	/* Restart timer. */
	tc_start(HAL_TC_USART, HAL_TC_USART_CHN);
}
#endif
#endif

/**
 * \brief Interrupt handler. Record the number of bytes received,
 * and then restart a read transfer on the USART if the transfer was stopped.
 */
void HAL_TC_USART_Handler(void)
{
	uint32_t ul_status;
#if defined(CONF_BOARD_USART0_RXD) || defined(CONF_BOARD_USART1_RXD) || defined(CONF_BOARD_USART2_RXD)
	uint32_t ul_byte_total = 0;
#endif

	/* Read HAL_TC_USART Status. */
	ul_status = tc_get_status(HAL_TC_USART, HAL_TC_USART_CHN);

	/* RC compare. */
	if ((ul_status & TC_SR_CPCS) == TC_SR_CPCS) {
#ifdef CONF_BOARD_USART0_RXD
		if (busart_chn_open[0]) {
			/* Read received bytes from uB ctrl register */
			ul_byte_total = USART_BUFFER_SIZE - XDMAC->XDMAC_CHID[BUSART_XDMAC_USART0_CH_RX].XDMAC_CUBC & 0x00FFFFFF;
			if (ul_byte_total > 0) {
				/* Flush the received bytes buffer */
				xdmac_channel_software_flush_request(XDMAC, BUSART_XDMAC_USART0_CH_RX);
				/* Read received bytes from uB ctrl register */
				ul_byte_total = USART_BUFFER_SIZE - XDMAC->XDMAC_CHID[BUSART_XDMAC_USART0_CH_RX].XDMAC_CUBC & 0x00FFFFFF;
				if (ul_byte_total == num_bytes_rx_usart0) {
					/* Disable timer. */
					tc_stop(HAL_TC_USART, HAL_TC_USART_CHN);

					/* Log current size */
					gs_ul_size_usart_buf0 = ul_byte_total;

					/* Process RX buffer */
					_BUSART_RX0_buffer_proc();

				} else {
					num_bytes_rx_usart0 = ul_byte_total;
				}
			} else {
				num_bytes_rx_usart0 = 0;
			}
		}
#endif

#ifdef CONF_BOARD_USART1_RXD
		if (busart_chn_open[1]) {

			/* Read received bytes from uB ctrl register */
			ul_byte_total = USART_BUFFER_SIZE - XDMAC->XDMAC_CHID[BUSART_XDMAC_USART1_CH_RX].XDMAC_CUBC & 0x00FFFFFF;
			if (ul_byte_total > 0) {

				/* Flush the received bytes buffer */
				xdmac_channel_software_flush_request(XDMAC, BUSART_XDMAC_USART1_CH_RX);
				/* Read received bytes from uB ctrl register */
				ul_byte_total = USART_BUFFER_SIZE - XDMAC->XDMAC_CHID[BUSART_XDMAC_USART1_CH_RX].XDMAC_CUBC & 0x00FFFFFF;
				if (ul_byte_total == num_bytes_rx_usart1) {
					/* Disable timer. */
					tc_stop(HAL_TC_USART, HAL_TC_USART_CHN);

					/* Log current size */
					gs_ul_size_usart_buf1 = ul_byte_total;

					/* Process RX buffer */
					_BUSART_RX1_buffer_proc();
				} else {
					num_bytes_rx_usart1 = ul_byte_total;
				}
			} else {
				num_bytes_rx_usart1 = 0;
			}
		}
#endif

#ifdef CONF_BOARD_USART2_RXD
		if (busart_chn_open[2]) {
			/* Read received bytes from uB ctrl register */
			ul_byte_total = USART_BUFFER_SIZE - XDMAC->XDMAC_CHID[BUSART_XDMAC_USART2_CH_RX].XDMAC_CUBC & 0x00FFFFFF;
			if (ul_byte_total > 0) {

				/* Flush the received bytes buffer */
				xdmac_channel_software_flush_request(XDMAC, BUSART_XDMAC_USART2_CH_RX);
				/* Read received bytes from uB ctrl register */
				ul_byte_total = USART_BUFFER_SIZE - XDMAC->XDMAC_CHID[BUSART_XDMAC_USART2_CH_RX].XDMAC_CUBC & 0x00FFFFFF;
				if (ul_byte_total == num_bytes_rx_usart2) {
					/* Disable timer. */
					tc_stop(HAL_TC_USART, HAL_TC_USART_CHN);

					/* Log current size */
					gs_ul_size_usart_buf2 = ul_byte_total;

					/* Process RX buffer */
					_BUSART_RX2_buffer_proc();

				} else {
					num_bytes_rx_usart2 = ul_byte_total;
				}
			} else {
				num_bytes_rx_usart2 = 0;
			}
		}
#endif
	}
}

/**
 * \brief This function opens an USART
 *
 * \note Opening of the specified USART implies initializing local variables and
 * opening required hardware with the following configuration:
 * - bauds as specified
 * - 8 bits, no parity, 1 stop bit
 * - enable interrupts
 *
 * \param chn			Communication channel [0, 1]
 * \param bauds			Communication speed in bauds
 *
 * \retval true on success.
 * \retval false on failure.
 */
uint8_t hal_usart_open(uint8_t chn, uint32_t bauds)
{
#if defined(CONF_BOARD_USART0_RXD) || defined(CONF_BOARD_USART1_RXD) || defined(CONF_BOARD_USART2_RXD)
	sam_usart_opt_t usart_console_settings;

	/* Expected baud rate. */
	usart_console_settings.baudrate = bauds;

	/* Configure channel mode (Normal, Automatic, Local_loopback or
	 * Remote_loopback) */
	usart_console_settings.channel_mode = US_MR_CHMODE_NORMAL;
	/* Initialize value for USART mode register */
	usart_console_settings.parity_type = US_MR_PAR_NO;
	usart_console_settings.char_length = US_MR_CHRL_8_BIT;
	usart_console_settings.stop_bits = US_MR_NBSTOP_1_BIT;
#else
	UNUSED(bauds);
#endif
	/* check usart and it is close */
	if (chn >= 3) {
		return false;
	}

	if (busart_chn_open[chn]) {
		return false;
	}

	switch (chn) {
#ifdef CONF_BOARD_USART0_RXD
	case 0:
	{
		/* Configure PMC. */
		pmc_enable_periph_clk(ID_USART0);
		/* Configure USART. */
		usart_init_rs232(USART0, &usart_console_settings,
				sysclk_get_peripheral_hz());

		/* Assign buffers to pointers */
		busart_comm_data_0.puc_tq_buf = ptr_tx_usart_buf0;
		busart_comm_data_0.puc_rq_buf = ptr_rx_usart_buf0;
		busart_comm_data_0.us_rq_count = 0;
		busart_comm_data_0.us_rq_idx = 0;
		busart_comm_data_0.us_wq_idx = 0;

		/* Configure TX and RX XDMAC channels */
		busart0_xdmac_tx_channel_cfg.mbr_ubc = 0;
		busart0_xdmac_tx_channel_cfg.mbr_sa = (uint32_t)busart_comm_data_0.puc_tq_buf;
		busart0_xdmac_tx_channel_cfg.mbr_da = (uint32_t)(&(USART0->US_THR));
		busart0_xdmac_tx_channel_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_DSYNC_MEM2PER |
			XDMAC_CC_PERID(BUSART_XDMAC_USART0_TX_PERID) |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_MEMSET_NORMAL_MODE |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DWIDTH_BYTE |
			XDMAC_CC_SIF_AHB_IF0 |
			XDMAC_CC_DIF_AHB_IF1 |
			XDMAC_CC_SAM_INCREMENTED_AM |
			XDMAC_CC_DAM_FIXED_AM;
		busart0_xdmac_tx_channel_cfg.mbr_bc = 0;
		busart0_xdmac_tx_channel_cfg.mbr_ds = 0;
		busart0_xdmac_tx_channel_cfg.mbr_sus = 0;
		busart0_xdmac_tx_channel_cfg.mbr_dus = 0;

		busart0_xdmac_rx_channel_cfg.mbr_ubc = USART_BUFFER_SIZE;
		busart0_xdmac_rx_channel_cfg.mbr_sa = (uint32_t)(&(USART0->US_RHR));
		busart0_xdmac_rx_channel_cfg.mbr_da = (uint32_t)gs_puc_usart_buf0;
		busart0_xdmac_rx_channel_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_DSYNC_PER2MEM |
			XDMAC_CC_PERID(BUSART_XDMAC_USART0_RX_PERID) |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_MEMSET_NORMAL_MODE |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DWIDTH_BYTE |
			XDMAC_CC_SIF_AHB_IF1 |
			XDMAC_CC_DIF_AHB_IF0 |
			XDMAC_CC_SAM_FIXED_AM |
			XDMAC_CC_DAM_INCREMENTED_AM;
		busart0_xdmac_rx_channel_cfg.mbr_bc = 0;
		busart0_xdmac_rx_channel_cfg.mbr_ds =  0;
		busart0_xdmac_rx_channel_cfg.mbr_sus = 0;
		busart0_xdmac_rx_channel_cfg.mbr_dus = 0;

		xdmac_configure_transfer(XDMAC, BUSART_XDMAC_USART0_CH_TX, &busart0_xdmac_tx_channel_cfg);
		xdmac_channel_set_descriptor_control(XDMAC, BUSART_XDMAC_USART0_CH_TX, 0);
		xdmac_configure_transfer(XDMAC, BUSART_XDMAC_USART0_CH_RX, &busart0_xdmac_rx_channel_cfg);
		xdmac_channel_set_descriptor_control(XDMAC, BUSART_XDMAC_USART0_CH_RX, 0);

		/* Transfer to XDMAC communication mode, disable RXRDY interrupt. */
		usart_disable_interrupt(USART0, US_IDR_RXRDY);

		xdmac_channel_enable(XDMAC, BUSART_XDMAC_USART0_CH_TX);
		xdmac_channel_enable(XDMAC, BUSART_XDMAC_USART0_CH_RX);

		/* Enable the receiver and transmitter. */
		usart_enable_tx(USART0);
		usart_enable_rx(USART0);

		busart_chn_open[chn] = true;
		num_bytes_rx_usart0 = 0;

		/* Configure TC usart */
		_configure_TC_usart();
		tc_start(HAL_TC_USART, HAL_TC_USART_CHN);

		return true;
	}
	break;
#endif
#ifdef CONF_BOARD_USART1_RXD
	case 1:
	{
		/* Configure PMC. */
		pmc_enable_periph_clk(ID_USART1);
		/* Configure USART. */
		usart_init_rs232(USART1, &usart_console_settings,
				sysclk_get_peripheral_hz());

		/* Assign buffers to pointers */
		busart_comm_data_1.puc_tq_buf = ptr_tx_usart_buf1;
		busart_comm_data_1.puc_rq_buf = ptr_rx_usart_buf1;
		busart_comm_data_1.us_rq_count = 0;
		busart_comm_data_1.us_rq_idx = 0;
		busart_comm_data_1.us_wq_idx = 0;

		/* Configure TX and RX XDMAC channels */
		busart1_xdmac_tx_channel_cfg.mbr_ubc = 0;
		busart1_xdmac_tx_channel_cfg.mbr_sa = (uint32_t)busart_comm_data_1.puc_tq_buf;
		busart1_xdmac_tx_channel_cfg.mbr_da = (uint32_t)(&(USART1->US_THR));
		busart1_xdmac_tx_channel_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_DSYNC_MEM2PER |
			XDMAC_CC_PERID(BUSART_XDMAC_USART1_TX_PERID) |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_MEMSET_NORMAL_MODE |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DWIDTH_BYTE |
			XDMAC_CC_SIF_AHB_IF0 |
			XDMAC_CC_DIF_AHB_IF1 |
			XDMAC_CC_SAM_INCREMENTED_AM |
			XDMAC_CC_DAM_FIXED_AM;
		busart1_xdmac_tx_channel_cfg.mbr_bc = 0;
		busart1_xdmac_tx_channel_cfg.mbr_ds = 0;
		busart1_xdmac_tx_channel_cfg.mbr_sus = 0;
		busart1_xdmac_tx_channel_cfg.mbr_dus = 0;

		busart1_xdmac_rx_channel_cfg.mbr_ubc = USART_BUFFER_SIZE;
		busart1_xdmac_rx_channel_cfg.mbr_sa = (uint32_t)(&(USART1->US_RHR));
		busart1_xdmac_rx_channel_cfg.mbr_da = (uint32_t)gs_puc_usart_buf1;
		busart1_xdmac_rx_channel_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_DSYNC_PER2MEM |
			XDMAC_CC_PERID(BUSART_XDMAC_USART1_RX_PERID) |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_MEMSET_NORMAL_MODE |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DWIDTH_BYTE |
			XDMAC_CC_SIF_AHB_IF1 |
			XDMAC_CC_DIF_AHB_IF0 |
			XDMAC_CC_SAM_FIXED_AM |
			XDMAC_CC_DAM_INCREMENTED_AM;
		busart1_xdmac_rx_channel_cfg.mbr_bc = 0;
		busart1_xdmac_rx_channel_cfg.mbr_ds =  0;
		busart1_xdmac_rx_channel_cfg.mbr_sus = 0;
		busart1_xdmac_rx_channel_cfg.mbr_dus = 0;

		xdmac_configure_transfer(XDMAC, BUSART_XDMAC_USART1_CH_TX, &busart1_xdmac_tx_channel_cfg);
		xdmac_channel_set_descriptor_control(XDMAC, BUSART_XDMAC_USART1_CH_TX, 0);
		xdmac_configure_transfer(XDMAC, BUSART_XDMAC_USART1_CH_RX, &busart1_xdmac_rx_channel_cfg);
		xdmac_channel_set_descriptor_control(XDMAC, BUSART_XDMAC_USART1_CH_RX, 0);

		/* Transfer to XDMAC communication mode, disable RXRDY interrupt. */
		usart_disable_interrupt(USART1, US_IDR_RXRDY);

		xdmac_channel_enable(XDMAC, BUSART_XDMAC_USART1_CH_TX);
		xdmac_channel_enable(XDMAC, BUSART_XDMAC_USART1_CH_RX);

		/* Enable the receiver and transmitter. */
		usart_enable_tx(USART1);
		usart_enable_rx(USART1);

		busart_chn_open[chn] = true;
		num_bytes_rx_usart1 = 0;

		/* Configure TC usart */
		_configure_TC_usart();
		tc_start(HAL_TC_USART, HAL_TC_USART_CHN);

		return true;
	}
	break;
#endif

#ifdef CONF_BOARD_USART2_RXD
	case 2:
	{
		/* Configure PMC. */
		pmc_enable_periph_clk(ID_USART2);
		/* Configure USART. */
		usart_init_rs232(USART2, &usart_console_settings,
				sysclk_get_peripheral_hz());

		/* Assign buffers to pointers */
		busart_comm_data_2.puc_tq_buf = ptr_tx_usart_buf2;
		busart_comm_data_2.puc_rq_buf = ptr_rx_usart_buf2;
		busart_comm_data_2.us_rq_count = 0;
		busart_comm_data_2.us_rq_idx = 0;
		busart_comm_data_2.us_wq_idx = 0;

		/* Configure TX and RX XDMAC channels */
		busart2_xdmac_tx_channel_cfg.mbr_ubc = 0;
		busart2_xdmac_tx_channel_cfg.mbr_sa = (uint32_t)busart_comm_data_2.puc_tq_buf;
		busart2_xdmac_tx_channel_cfg.mbr_da = (uint32_t)(&(USART2->US_THR));
		busart2_xdmac_tx_channel_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_DSYNC_MEM2PER |
			XDMAC_CC_PERID(BUSART_XDMAC_USART2_TX_PERID) |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_MEMSET_NORMAL_MODE |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DWIDTH_BYTE |
			XDMAC_CC_SIF_AHB_IF0 |
			XDMAC_CC_DIF_AHB_IF1 |
			XDMAC_CC_SAM_INCREMENTED_AM |
			XDMAC_CC_DAM_FIXED_AM;
		busart2_xdmac_tx_channel_cfg.mbr_bc = 0;
		busart2_xdmac_tx_channel_cfg.mbr_ds = 0;
		busart2_xdmac_tx_channel_cfg.mbr_sus = 0;
		busart2_xdmac_tx_channel_cfg.mbr_dus = 0;

		busart2_xdmac_rx_channel_cfg.mbr_ubc = USART_BUFFER_SIZE;
		busart2_xdmac_rx_channel_cfg.mbr_sa = (uint32_t)(&(USART2->US_RHR));
		busart2_xdmac_rx_channel_cfg.mbr_da = (uint32_t)gs_puc_usart_buf2;
		busart2_xdmac_rx_channel_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_DSYNC_PER2MEM |
			XDMAC_CC_PERID(BUSART_XDMAC_USART2_RX_PERID) |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_MEMSET_NORMAL_MODE |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DWIDTH_BYTE |
			XDMAC_CC_SIF_AHB_IF1 |
			XDMAC_CC_DIF_AHB_IF0 |
			XDMAC_CC_SAM_FIXED_AM |
			XDMAC_CC_DAM_INCREMENTED_AM;
		busart2_xdmac_rx_channel_cfg.mbr_bc = 0;
		busart2_xdmac_rx_channel_cfg.mbr_ds =  0;
		busart2_xdmac_rx_channel_cfg.mbr_sus = 0;
		busart2_xdmac_rx_channel_cfg.mbr_dus = 0;

		xdmac_configure_transfer(XDMAC, BUSART_XDMAC_USART2_CH_TX, &busart2_xdmac_tx_channel_cfg);
		xdmac_channel_set_descriptor_control(XDMAC, BUSART_XDMAC_USART2_CH_TX, 0);
		xdmac_configure_transfer(XDMAC, BUSART_XDMAC_USART2_CH_RX, &busart2_xdmac_rx_channel_cfg);
		xdmac_channel_set_descriptor_control(XDMAC, BUSART_XDMAC_USART2_CH_RX, 0);

		/* Transfer to XDMAC communication mode, disable RXRDY interrupt. */
		usart_disable_interrupt(USART2, US_IDR_RXRDY);

		xdmac_channel_enable(XDMAC, BUSART_XDMAC_USART2_CH_TX);
		xdmac_channel_enable(XDMAC, BUSART_XDMAC_USART2_CH_RX);

		/* Enable the receiver and transmitter. */
		usart_enable_tx(USART2);
		usart_enable_rx(USART2);

		/* Configure and enable interrupt of USART. */
		NVIC_SetPriority((IRQn_Type)USART2_IRQn, USART2_PRIO);
		NVIC_EnableIRQ(USART2_IRQn);

		busart_chn_open[chn] = true;
		num_bytes_rx_usart2 = 0;

		/* Configure TC usart */
		_configure_TC_usart();
		tc_start(HAL_TC_USART, HAL_TC_USART_CHN);

		return true;
	}
	break;
#endif
	default:
		return false;
	}
}

/**
 * \brief This function closes and disables communication in the specified
 * USART.
 *
 * \param chn  Communication channel [0, 1]
 *
 * \retval true on success.
 * \retval false on failure.
 */
uint8_t hal_usart_close(uint8_t chn)
{
	if (!busart_chn_open[chn]) {
		return false;
	}

	switch (chn) {
#ifdef CONF_BOARD_USART0_RXD
	case 0:
	{
		usart_disable_tx(USART0);
		usart_disable_rx(USART0);

		/* Stop TC */
		if (!busart_chn_open[1]) {
			tc_stop(HAL_TC_USART, HAL_TC_USART_CHN);
		}

		return true;
	}
	break;
#endif

#ifdef CONF_BOARD_USART1_RXD
	case 1:
	{
		usart_disable_tx(USART1);
		usart_disable_rx(USART1);

		/* Stop TC */
		if (!busart_chn_open[0]) {
			tc_stop(HAL_TC_USART, HAL_TC_USART_CHN);
		}

		return true;
	}
	break;
#endif

#ifdef CONF_BOARD_USART2_RXD
	case 2:
	{
		usart_disable_tx(USART2);
		usart_disable_rx(USART2);

		/* Stop TC */
		if (!busart_chn_open[0]) {
			tc_stop(HAL_TC_USART, HAL_TC_USART_CHN);
		}

		return true;
	}
	break;
#endif
	default:
		return false;
	}
}

/**
 * \brief This function receives a message.
 *
 * \note This function receives a given number of characters from the specified
 * USART. If so configured, the function waits until all characters are
 * received. In this case, the watchdog timer must be reloaded to avoid a
 * program reset.
 *
 * \param  chn     Communication channel [0, 1]
 * \param  buffer  Pointer to buffer for information
 * \param  len     Number of characters to receive
 *
 * \retval Number of received characters
 */
uint16_t hal_usart_read(uint8_t chn, void *buffer, uint16_t len)
{
#if defined(CONF_BOARD_USART0_RXD) || defined(CONF_BOARD_USART1_RXD) || defined(CONF_BOARD_USART2_RXD)
	uint16_t us_rd_idx = 0;
	uint16_t us_num_bytes_read, us_num_bytes_to_end, us_num_bytes_to_start;
	uint16_t us_total_pos;
	uint16_t us_buf_size;

	uint8_t *msg = (uint8_t *)buffer;
#else
	UNUSED(buffer);
	UNUSED(len);
#endif

	/* check usart is open */
	if (!busart_chn_open[chn]) {
		return 0;
	}

	switch (chn) {
#ifdef CONF_BOARD_USART0_RXD
	case 0:
		us_buf_size = HAL_RX_USART_BUF0_SIZE;
		/* check if there is any byte in rx queue */
		if (busart_comm_data_0.us_rq_count == 0) {
			return 0;
		}

		/* get counters */
		us_rd_idx = busart_comm_data_0.us_rq_idx;
		/* get number of bytes to read */
		if (busart_comm_data_0.us_rq_count >= len) {
			us_num_bytes_read = len;
		} else {
			us_num_bytes_read = busart_comm_data_0.us_rq_count;
		}

		/* check overflow us_rd_idx counter */
		us_total_pos = us_rd_idx + us_num_bytes_read;
		if (us_total_pos <= us_buf_size) {
			/* copy data to buffer */
			memcpy(msg, &busart_comm_data_0.puc_rq_buf[us_rd_idx], us_num_bytes_read);
			/* update counters */
			busart_comm_data_0.us_rq_count -= us_num_bytes_read;
			busart_comm_data_0.us_rq_idx += us_num_bytes_read;
		} else {
			/* copy data to buffer in fragments -> overflow
			 * us_rq_idx counter */
			us_num_bytes_to_start = us_total_pos - us_buf_size;
			us_num_bytes_to_end = us_num_bytes_read - us_num_bytes_to_start;
			memcpy(msg, &busart_comm_data_0.puc_rq_buf[us_rd_idx], us_num_bytes_to_end);
			msg += us_num_bytes_to_end;
			memcpy(msg, &busart_comm_data_0.puc_rq_buf[0], us_num_bytes_to_start);
			/* update counters */
			busart_comm_data_0.us_rq_count -= us_num_bytes_read;
			busart_comm_data_0.us_rq_idx = us_num_bytes_to_start;
		}

		return us_num_bytes_read;
#endif

#ifdef CONF_BOARD_USART1_RXD
	case 1:
		us_buf_size = HAL_RX_USART_BUF1_SIZE;
		/* check if there is any byte in rx queue */
		if (busart_comm_data_1.us_rq_count == 0) {
			return 0;
		}

		/* get counters */
		us_rd_idx = busart_comm_data_1.us_rq_idx;
		/* get number of bytes to read */
		if (busart_comm_data_1.us_rq_count >= len) {
			us_num_bytes_read = len;
		} else {
			us_num_bytes_read = busart_comm_data_1.us_rq_count;
		}

		/* check overflow us_rd_idx counter */
		us_total_pos = us_rd_idx + us_num_bytes_read;
		if (us_total_pos <= us_buf_size) {
			/* copy data to buffer */
			memcpy(msg, &busart_comm_data_1.puc_rq_buf[us_rd_idx], us_num_bytes_read);
			/* update counters */
			busart_comm_data_1.us_rq_count -= us_num_bytes_read;
			busart_comm_data_1.us_rq_idx += us_num_bytes_read;
		} else {
			/* copy data to buffer in fragments -> overflow
			 * us_rq_idx counter */
			us_num_bytes_to_start = us_total_pos - us_buf_size;
			us_num_bytes_to_end = us_num_bytes_read - us_num_bytes_to_start;
			memcpy(msg, &busart_comm_data_1.puc_rq_buf[us_rd_idx], us_num_bytes_to_end);
			msg += us_num_bytes_to_end;
			memcpy(msg, &busart_comm_data_1.puc_rq_buf[0], us_num_bytes_to_start);
			/* update counters */
			busart_comm_data_1.us_rq_count -= us_num_bytes_read;
			busart_comm_data_1.us_rq_idx = us_num_bytes_to_start;
		}
		return us_num_bytes_read;
#endif

#ifdef CONF_BOARD_USART2_RXD
	case 2:
		us_buf_size = HAL_RX_USART_BUF2_SIZE;
		/* check if there is any byte in rx queue */
		if (busart_comm_data_2.us_rq_count == 0) {
			return 0;
		}

		/* get counters */
		us_rd_idx = busart_comm_data_2.us_rq_idx;
		/* get number of bytes to read */
		if (busart_comm_data_2.us_rq_count >= len) {
			us_num_bytes_read = len;
		} else {
			us_num_bytes_read = busart_comm_data_2.us_rq_count;
		}

		/* check overflow us_rd_idx counter */
		us_total_pos = us_rd_idx + us_num_bytes_read;
		if (us_total_pos <= us_buf_size) {
			/* copy data to buffer */
			memcpy(msg, &busart_comm_data_2.puc_rq_buf[us_rd_idx], us_num_bytes_read);
			/* update counters */
			busart_comm_data_2.us_rq_count -= us_num_bytes_read;
			busart_comm_data_2.us_rq_idx += us_num_bytes_read;
		} else {
			/* copy data to buffer in fragments -> overflow
			 * us_rq_idx counter */
			us_num_bytes_to_start = us_total_pos - us_buf_size;
			us_num_bytes_to_end = us_num_bytes_read - us_num_bytes_to_start;
			memcpy(msg, &busart_comm_data_2.puc_rq_buf[us_rd_idx], us_num_bytes_to_end);
			msg += us_num_bytes_to_end;
			memcpy(msg, &busart_comm_data_2.puc_rq_buf[0], us_num_bytes_to_start);
			/* update counters */
			busart_comm_data_2.us_rq_count -= us_num_bytes_read;
			busart_comm_data_2.us_rq_idx = us_num_bytes_to_start;
		}
		return us_num_bytes_read;
#endif
	default:
		return 0;
	}
}

/**
 * \brief This function transmits a message.
 *
 * \note This function transmits characters via the specified USART.
 * If so configured, the function waits until all characters are inserted
 * in the transmission queue. In this case, the watchdog timer must be
 * reloaded to avoid a program reset.
 *
 * \param  chn     Communication channel [0, 1]
 * \param  buffer  Pointer to information to transmit
 * \param  len     Number of characters to transmit
 *
 * \retval Number of characters sent
 */
uint16_t hal_usart_write(uint8_t chn, const void *buffer, uint16_t len)
{
#if !defined(CONF_BOARD_USART0_RXD) && !defined(CONF_BOARD_USART1_RXD) && !defined(CONF_BOARD_USART2_RXD)
	UNUSED(buffer);
	UNUSED(len);
#endif
	/* check usart is open */
	if (!busart_chn_open[chn]) {
		return 0;
	}

	switch (chn) {
#ifdef CONF_BOARD_USART0_RXD
	case 0:
		if (len > HAL_TX_USART_BUF0_SIZE) {
			return 0;
		}

		/* Wait previous TX transfer finish */
		while (xdmac_channel_get_status(XDMAC) & BUSART_XDMAC_USART0_TX_STATUS) {
		}

		memcpy(&busart_comm_data_0.puc_tq_buf[0], buffer, len);
		busart0_xdmac_tx_channel_cfg.mbr_ubc = len;
		xdmac_configure_transfer(XDMAC, BUSART_XDMAC_USART0_CH_TX, &busart0_xdmac_tx_channel_cfg);
		xdmac_channel_enable(XDMAC, BUSART_XDMAC_USART0_CH_TX);
		return len;
#endif

#ifdef CONF_BOARD_USART1_RXD
	case 1:
		if (len > HAL_TX_USART_BUF1_SIZE) {
			return 0;
		}

		/* Wait previous TX transfer finish */
		while (xdmac_channel_get_status(XDMAC) & BUSART_XDMAC_USART1_TX_STATUS) {
		}

		memcpy(&busart_comm_data_1.puc_tq_buf[0], buffer, len);
		busart1_xdmac_tx_channel_cfg.mbr_ubc = len;
		xdmac_configure_transfer(XDMAC, BUSART_XDMAC_USART1_CH_TX, &busart1_xdmac_tx_channel_cfg);
		xdmac_channel_enable(XDMAC, BUSART_XDMAC_USART1_CH_TX);
		return len;
#endif

#ifdef CONF_BOARD_USART2_RXD
	case 2:
		if (len > HAL_TX_USART_BUF2_SIZE) {
			return 0;
		}

		/* Wait previous TX transfer finish */
		while (xdmac_channel_get_status(XDMAC) & BUSART_XDMAC_USART2_TX_STATUS) {
		}

		memcpy(&busart_comm_data_2.puc_tq_buf[0], buffer, len);
		busart2_xdmac_tx_channel_cfg.mbr_ubc = len;
		xdmac_configure_transfer(XDMAC, BUSART_XDMAC_USART2_CH_TX, &busart2_xdmac_tx_channel_cfg);
		xdmac_channel_enable(XDMAC, BUSART_XDMAC_USART2_CH_TX);
		return len;
#endif

	default:
		return 0;
	}
}

/**
 * \brief Get byte from USART.
 *
 * \param  chn  Communication channel [0, 1]
 *
 * \retval Byte received
 * \retval -1 in case of no reception
 */
int hal_usart_rx_char(uint8_t chn)
{
	uint8_t buf[4] = {0, 0, 0, 0};

	if (hal_usart_read(chn, buf, 1) <= 0) {
		return (-1);
	}

	return buf[0];
}

/**
 * \brief Sent byte to USART.
 *
 * \param  chn   Communication channel [0, 1]
 * \param  data  Data to sent
 *
 * \retval Number of characters sent
 */
uint16_t hal_usart_tx_char(uint8_t chn, char data)
{
	return (hal_usart_write(chn, &data, 1));
}

/* @} */
/**
 * \brief Check USART PDC transmission in course.
 *
 * \param  chn   Communication channel [0, 1]
 *
 * \retval true is USART is free to tx, false in otherwise
 */
bool hal_usart_is_free(uint8_t chn)
{
	/* check uart is open */
	if (!busart_chn_open[chn]) {
		return false;
	}

	switch (chn) {
#ifdef CONF_BOARD_USART0_RXD    //CONF_BOARD_USART0
	case 0:
		if (USART_BUFFER_SIZE - XDMAC->XDMAC_CHID[BUSART_XDMAC_USART0_CH_RX].XDMAC_CUBC & 0x00FFFFFF) {
			return false;
		} else {
			return true;
		}
#endif

#ifdef CONF_BOARD_USART1_RXD    //CONF_BOARD_USART1
	case 1:
		if (USART_BUFFER_SIZE - XDMAC->XDMAC_CHID[BUSART_XDMAC_USART1_CH_RX].XDMAC_CUBC & 0x00FFFFFF) {
			return false;
		} else {
			return true;
		}
#endif

#ifdef CONF_BOARD_USART2_RXD    //CONF_BOARD_USART2
	case 2:
		if (USART_BUFFER_SIZE - XDMAC->XDMAC_CHID[BUSART_XDMAC_USART2_CH_RX].XDMAC_CUBC & 0x00FFFFFF) {
			return false;
		} else {
			return true;
		}
#endif

	default:
		return false;
	}
}


/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */
