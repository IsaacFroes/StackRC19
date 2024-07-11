/**
 * \file
 *
 * \brief HAL_PLC
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
#include "string.h"
#include "asf.h"
#include "ioport.h"
#include "pmc.h"
#include "pio.h"
#include "pio_handler.h"
#include "spi.h"
#include "xdmac.h"
#include "hal_private.h"
#include "afec.h"

#if !(SAMV71 || SAMV70 || SAME70 || SAMS70)
#  error No valid platform.
#endif

#include "board.h"
#include "conf_board.h"

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* / @endcond */

#if defined(HAL_ATPL360_INTERFACE) && defined(HAL_ENABLE_PHY_RF)
# if (HAL_PLC_SPI_ID == PRF_SPI_ID)
#  define HAL_PLC_SPI_SHARED_RF
# endif
#endif

#ifdef CONF_BOARD_ENABLE_CACHE

/* If cortex-M7 cache enabled, there are incoherency issues when using DMA.
 * They can be solved by using TCM, MPU to define non-cacheable region, or using
 * cache maintenance operations. If TCM or non-cacheable region is enabled in
 * conf_board.h, it is assumed that all data used by DMA (hal_cm7_plc) will be
 * mapped accordingly by the linker script, and therefore cache maintenance
 * operations are not performed. */
# if (!defined(CONF_BOARD_ENABLE_TCM_AT_INIT)) && !(defined(CONF_BOARD_CONFIG_MPU_AT_INIT) && defined(MPU_HAS_NOCACHE_REGION))
#  define HAL_PLC_CM7_CACHE_MAINTENANCE
# endif
#endif

#ifdef HAL_ATPL360_INTERFACE
/** SPI Header size. */
# define HAL_PLC_SPI_HEADER_SIZE       4
/** SPI Max Msg_Data size. */
# define HAL_PLC_SPI_MSG_DATA_SIZE     512
/** SPI Max Msg_Params size. */
# define HAL_PLC_SPI_MSG_PARAMS_SIZE   118   /* Worst case = 118: sizeof(rx_msg_t) [G3] */
/** XDMAC buffer size. */
# define HAL_PLC_BUFFER_SIZE           (HAL_PLC_SPI_HEADER_SIZE + HAL_PLC_SPI_MSG_DATA_SIZE + HAL_PLC_SPI_MSG_PARAMS_SIZE)
#else
/** SPI Header size. */
# define HAL_PLC_SPI_HEADER_SIZE       4
/** XDMAC buffer size. */
# define HAL_PLC_BUFFER_SIZE           0x400 /* 1024 */
#endif

/* XDMAC channels used for HAL_PLC SPI */
# define HAL_PLC_XDMAC_CH_TX           0
# define HAL_PLC_XDMAC_CH_RX           1
/* XDMAC channel status  */
# define HAL_PLC_XDMAC_TX_STATUS       XDMAC_GS_ST0
# define HAL_PLC_XDMAC_RX_STATUS       XDMAC_GS_ST1

/* XDMAC Peripheral IDs */
# ifndef HAL_PLC_SPI_ID
#  define HAL_PLC_XDMAC_SPI_TX_PERID   XDMAC_CHANNEL_HWID_SPI0_TX
#  define HAL_PLC_XDMAC_SPI_RX_PERID   XDMAC_CHANNEL_HWID_SPI0_RX
# elif HAL_PLC_SPI_ID == ID_SPI0
#  define HAL_PLC_XDMAC_SPI_TX_PERID   XDMAC_CHANNEL_HWID_SPI0_TX
#  define HAL_PLC_XDMAC_SPI_RX_PERID   XDMAC_CHANNEL_HWID_SPI0_RX
# elif HAL_PLC_SPI_ID == ID_SPI1
#  define HAL_PLC_XDMAC_SPI_TX_PERID   XDMAC_CHANNEL_HWID_SPI1_TX
#  define HAL_PLC_XDMAC_SPI_RX_PERID   XDMAC_CHANNEL_HWID_SPI1_RX
# endif

/** XDMAC channel configuration. */
#ifdef HAL_ATPL360_INTERFACE
static xdmac_channel_config_t xdmac_tx_channel_cfg_boot;
static xdmac_channel_config_t xdmac_rx_channel_cfg_boot;
static xdmac_channel_config_t xdmac_tx_channel_cfg_phy;
static xdmac_channel_config_t xdmac_rx_channel_cfg_phy;
/** PLC busy indication */
static uint8_t suc_plc_is_busy;
#endif
static xdmac_channel_config_t xdmac_tx_channel_cfg;
static xdmac_channel_config_t xdmac_rx_channel_cfg;

#ifdef HAL_PLC_CM7_CACHE_MAINTENANCE

/* When using cache maintenance operations, buffers used by DMA must be aligned
 * to cache line size (32 bytes) */

/** XDMAC Receive buffer */
COMPILER_ALIGNED(32)
static uint8_t gs_plc_rx_buffer[(div32_ceil(HAL_PLC_BUFFER_SIZE) << 5)];
/** XDMAC Transmission buffer */
COMPILER_ALIGNED(32)
static uint8_t gs_plc_tx_buffer[(div32_ceil(HAL_PLC_BUFFER_SIZE) << 5)];

#else
/** XDMAC Receive buffer */
COMPILER_ALIGNED(4)
static uint8_t gs_plc_rx_buffer[HAL_PLC_BUFFER_SIZE];
/** XDMAC Transmission buffer */
COMPILER_ALIGNED(4)
static uint8_t gs_plc_tx_buffer[HAL_PLC_BUFFER_SIZE];
#endif

/** PLC chip select config value */
#define HAL_PLC_PCS    spi_get_pcs(HAL_PLC_CS)

/** \brief PLC interrupt handlers */
/* @{ */
static void (*plc_handler)(void);
static void (*plc_tx_signalling_handler)(void);
static void (*plc_rx_signalling_handler)(void);
/* @} */

#ifdef HAL_PLC_PVDD_MON_ADC_CHN

/** Resistor values (in ohms) of external voltage divider circuitry */
# ifndef HAL_PLC_PVDD_MON_RUP_OHM
/* Rup = 36k by default */
#  define HAL_PLC_PVDD_MON_RUP_OHM                36000
# endif
# ifndef HAL_PLC_PVDD_MON_RDOWN_OHM
/* Rdown = 10k by default */
#  define HAL_PLC_PVDD_MON_RDOWN_OHM              10000
# endif

/** Thresholds (in mV) for PVDD monitor */
# ifndef HAL_PLC_PVDD_MON_HIGHTHRES_MV
/* High threshold: 13V by defult */
#  define HAL_PLC_PVDD_MON_HIGHTHRES_MV           13000
# endif
# ifndef HAL_PLC_PVDD_MON_LOWTHRES_MV
/* Low threshold: 10V by defult */
#  define HAL_PLC_PVDD_MON_LOWTHRES_MV            10000
# endif
# ifndef HAL_PLC_PVDD_MON_HIGHTHRES_HYST_MV
/* High threshold (hysteresis): 12.9V by defult */
#  define HAL_PLC_PVDD_MON_HIGHTHRES_HYST_MV      12900
# endif
# ifndef HAL_PLC_PVDD_MON_LOWTHRES_HYST_MV
/* Low threshold (hysteresis): 10.2V by defult */
#  define HAL_PLC_PVDD_MON_LOWTHRES_HYST_MV       10200
# endif

/** Voltage reference (in mV) for ADC */
# ifndef HAL_PLC_PVDD_MON_ADC_REF_MV
/* 3.3V by default */
#  define HAL_PLC_PVDD_MON_ADC_REF_MV             3300
# endif

/* Voltage at ADC input: V_adc_in = PVDD * Rdown / (Rdown + Rup)
 * ADC converted value: (V_adc_in / V_adc_ref) * 2^Nbits */
# define HAL_PLC_PVDD_MON_THRES_DEN               ((HAL_PLC_PVDD_MON_RDOWN_OHM + HAL_PLC_PVDD_MON_RUP_OHM) * HAL_PLC_PVDD_MON_ADC_REF_MV)
# define HAL_PLC_PVDD_MON_HIGHTHRES_NUM           (HAL_PLC_PVDD_MON_HIGHTHRES_MV * HAL_PLC_PVDD_MON_RDOWN_OHM)
# define HAL_PLC_PVDD_MON_LOWTHRES_NUM            (HAL_PLC_PVDD_MON_LOWTHRES_MV * HAL_PLC_PVDD_MON_RDOWN_OHM)
# define HAL_PLC_PVDD_MON_HIGHTHRES_HYST_NUM      (HAL_PLC_PVDD_MON_HIGHTHRES_HYST_MV * HAL_PLC_PVDD_MON_RDOWN_OHM)
# define HAL_PLC_PVDD_MON_LOWTHRES_HYST_NUM       (HAL_PLC_PVDD_MON_LOWTHRES_HYST_MV * HAL_PLC_PVDD_MON_RDOWN_OHM)

# define HAL_PLC_PVDD_MON_ADC_BITS               12

/** High and Low threshold ADC values (10 bits) */
# define HAL_PLC_PVDD_MON_HIGHTHRES_ADC_VAL       ((uint16_t)div_round((uint64_t)HAL_PLC_PVDD_MON_HIGHTHRES_NUM << HAL_PLC_PVDD_MON_ADC_BITS, HAL_PLC_PVDD_MON_THRES_DEN))
# define HAL_PLC_PVDD_MON_LOWTHRES_ADC_VAL        ((uint16_t)div_round((uint64_t)HAL_PLC_PVDD_MON_LOWTHRES_NUM << HAL_PLC_PVDD_MON_ADC_BITS, HAL_PLC_PVDD_MON_THRES_DEN))
# define HAL_PLC_PVDD_MON_HIGHTHRES_HYST_ADC_VAL  ((uint16_t)div_round((uint64_t)HAL_PLC_PVDD_MON_HIGHTHRES_HYST_NUM << HAL_PLC_PVDD_MON_ADC_BITS, HAL_PLC_PVDD_MON_THRES_DEN))
# define HAL_PLC_PVDD_MON_LOWTHRES_HYST_ADC_VAL   ((uint16_t)div_round((uint64_t)HAL_PLC_PVDD_MON_LOWTHRES_HYST_NUM << HAL_PLC_PVDD_MON_ADC_BITS, HAL_PLC_PVDD_MON_THRES_DEN))

/* ADC clock frequency, tracking time and startup time must be configured
 * according to datasheet recommendations.
 * For tracking time computations, source resistance of external circuitry is
 * Rsource = 1k + 10k||36k = 8.83k  */

/** AFEC clock frequency (in Hz)  */
# ifndef HAL_PLC_PVDD_MON_AFEC_CLK_FREQ
/* 7.5 MHz by default */
#  define HAL_PLC_PVDD_MON_AFEC_CLK_FREQ         7500000
# endif

/** AFEC tracking time: 15 cycles of AFEC clock (2 us at 7.5 MHz).
 * From datasheet: tTRACKTIM (ns) >= 0.077 * Rsource (ohm) + 614 = 1.29 us */

/** AFEC startup time in clycles of AFEC clock.
 * From datasheet: tSTART >= 4 us */
# ifndef HAL_PLC_PVDD_MON_AFEC_STARTUP_TIME
/* 64 cycles by default (8.53 us at 7.5 MHz) */
#  define HAL_PLC_PVDD_MON_AFEC_STARTUP_TIME     AFEC_STARTUP_TIME_4
# endif

/** AFEC bias current control, depending on sampling rate (fs). From datasheet:
 * IBCTL=01 for fs < 500 kHz
 * IBCTL=10 for fs < 1 MHz
 * IBCTL=11 for fs > 1 MHz */
# ifndef HAL_PLC_PVDD_MON_AFEC_IBCTL
/* fs = fAFEC / 23 = 326 kHz, with fAFEC = 7.5 MHz */
#  define HAL_PLC_PVDD_MON_AFEC_IBCTL            1
# endif

/* PVDD monitor interrupt handler */
static void (*hal_plc_pvdd_mon_handler)(bool);

/* PVDD good/bad flag */
static bool sb_pvdd_good;

/**
 * \brief Update comparison window mode and thresholds for PVDD monitor
 *
 * \param b_pvdd_good Current status of PVDD: Good (true) or bad (false)
 */
static void _pvdd_mon_set_comp_mode(bool b_pvdd_good)
{
	if (b_pvdd_good) {
		/* PVDD within allowed range: Set compare mode to OUT */
		afec_set_comparison_window(HAL_PLC_PVDD_MON_AFEC_MODULE, HAL_PLC_PVDD_MON_LOWTHRES_ADC_VAL, HAL_PLC_PVDD_MON_HIGHTHRES_ADC_VAL);
		afec_set_comparison_mode(HAL_PLC_PVDD_MON_AFEC_MODULE, AFEC_CMP_MODE_3, HAL_PLC_PVDD_MON_ADC_CHN, 0);
	} else {
		/* PVDD out of allowed range: Set compare mode to IN */
		afec_set_comparison_window(HAL_PLC_PVDD_MON_AFEC_MODULE, HAL_PLC_PVDD_MON_LOWTHRES_HYST_ADC_VAL, HAL_PLC_PVDD_MON_HIGHTHRES_HYST_ADC_VAL);
		afec_set_comparison_mode(HAL_PLC_PVDD_MON_AFEC_MODULE, AFEC_CMP_MODE_2, HAL_PLC_PVDD_MON_ADC_CHN, 3);
	}
}

/**
 * \brief Handler of ADC interrupt. Check PVDD monitor ADC value. Use TX Enable
 * pin to enable/disable PLC TX depending on PVDD status. If PVDD status
 * changes, update comparison window mode and thresholds, and call upper layer
 * callback
 */
static void _pvdd_mon_adc_handler(void)
{
	uint16_t us_adc_value;
	bool b_pvdd_good;

	/* Read PVDD monitor ADC value */
	us_adc_value = (uint16_t)afec_channel_get_value(HAL_PLC_PVDD_MON_AFEC_MODULE, HAL_PLC_PVDD_MON_ADC_CHN);

	/* Compare read value with thresholds */
	b_pvdd_good = sb_pvdd_good;
	if (sb_pvdd_good) {
		/* Thresholds from good to bad PVDD */
		if ((us_adc_value < HAL_PLC_PVDD_MON_LOWTHRES_ADC_VAL) || (us_adc_value > HAL_PLC_PVDD_MON_HIGHTHRES_ADC_VAL)) {
			b_pvdd_good = false;
		}
	} else {
		/* Thresholds from bad to good PVDD */
		if ((us_adc_value > HAL_PLC_PVDD_MON_LOWTHRES_HYST_ADC_VAL) && (us_adc_value < HAL_PLC_PVDD_MON_HIGHTHRES_HYST_ADC_VAL)) {
			b_pvdd_good = true;
		}
	}

# ifdef HAL_PLC_TX_ENABLE_GPIO
	if (!b_pvdd_good) {
		/* Disable PLC TX with TX Enable pin */
		/* If there is ongoing TX it will be aborted */
		ioport_set_pin_level(HAL_PLC_TX_ENABLE_GPIO, HAL_PLC_TX_ENABLE_INACTIVE_LEVEL);
	} else {
		/* Enable PLC TX with TX Enable pin */
		ioport_set_pin_level(HAL_PLC_TX_ENABLE_GPIO, HAL_PLC_TX_ENABLE_ACTIVE_LEVEL);
	}
# endif

	if (b_pvdd_good != sb_pvdd_good) {
		/* PVDD status changed: call upper layer callback */
		if (hal_plc_pvdd_mon_handler != NULL) {
			hal_plc_pvdd_mon_handler(b_pvdd_good);
		}

		/* Update comparison window mode and thresholds */
		_pvdd_mon_set_comp_mode(b_pvdd_good);

		/* Update flag */
		sb_pvdd_good = b_pvdd_good;
	}
}

/**
 * \brief Initialize PVDD monitor. Configure ADC using Comparison Window.
 */
static void _pvdd_mon_init(void)
{
	/* Initialize PVDD status: Handler only called if PVDD is bad */
	sb_pvdd_good = true;

	struct afec_config afec_cfg;
	struct afec_ch_config afec_ch_cfg;

	/* Enable AFEC peripheral */
	afec_enable(HAL_PLC_PVDD_MON_AFEC_MODULE);

	/* AFEC configuration */
	afec_cfg.resolution = AFEC_12_BITS;
	afec_cfg.mck = sysclk_get_peripheral_hz();
	afec_cfg.afec_clock = HAL_PLC_PVDD_MON_AFEC_CLK_FREQ;
	afec_cfg.startup_time = HAL_PLC_PVDD_MON_AFEC_STARTUP_TIME;
	afec_cfg.tracktim = 0;
	afec_cfg.transfer = 2;
	afec_cfg.useq = false;
	afec_cfg.tag = false;
	afec_cfg.stm = false;
	afec_cfg.ibctl = HAL_PLC_PVDD_MON_AFEC_IBCTL;
	afec_init(HAL_PLC_PVDD_MON_AFEC_MODULE, &afec_cfg);

	/* Enable and configure channel used for PVDD monitor */
	afec_channel_enable(HAL_PLC_PVDD_MON_AFEC_MODULE, HAL_PLC_PVDD_MON_ADC_CHN);
	afec_ch_cfg.diff = false;
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(HAL_PLC_PVDD_MON_AFEC_MODULE, HAL_PLC_PVDD_MON_ADC_CHN, &afec_ch_cfg);
	afec_channel_set_analog_offset(HAL_PLC_PVDD_MON_AFEC_MODULE, HAL_PLC_PVDD_MON_ADC_CHN, 512);

	/* Set comparison window mode and thresholds */
	_pvdd_mon_set_comp_mode(true);

	/* Configure AFEC in Free-Running mode */
	afec_set_trigger(HAL_PLC_PVDD_MON_AFEC_MODULE, AFEC_TRIG_FREERUN);

	/* Set AFEC callback for PVDD monitor */
	afec_set_callback(HAL_PLC_PVDD_MON_AFEC_MODULE, AFEC_INTERRUPT_COMP_ERROR, _pvdd_mon_adc_handler, HAL_PLC_ADC_PRIO);
}

#endif /* HAL_PLC_PVDD_MON_ADC_CHN */

/**
 * \brief PLC interruption handler
 *
 * \param ul_id     Identifier
 * \param ul_mask   Mask
 *
 */
static void hal_plc_int_handler(uint32_t ul_id, uint32_t ul_mask)
{
	UNUSED(ul_id);
	UNUSED(ul_mask);
	if (plc_handler != NULL) {
		plc_handler();
	}

	/* Delete level interrupt */
	pio_get_interrupt_status(HAL_PLC_INT_PIO);
}

/**
 * \brief Disable XDMAC for spi and forbidden transmit and receive by XDMAC.
 *
 */
static void _xdmac_disable(void)
{
	uint32_t xdmaint;

	xdmaint = (XDMAC_CIE_BIE |
			XDMAC_CIE_DIE   |
			XDMAC_CIE_FIE   |
			XDMAC_CIE_RBIE  |
			XDMAC_CIE_WBIE  |
			XDMAC_CIE_ROIE);

	xdmac_channel_disable_interrupt(XDMAC, HAL_PLC_XDMAC_CH_RX, xdmaint);
	xdmac_channel_disable(XDMAC, HAL_PLC_XDMAC_CH_RX);
	xdmac_disable_interrupt(XDMAC, HAL_PLC_XDMAC_CH_RX);

	xdmac_channel_disable_interrupt(XDMAC, HAL_PLC_XDMAC_CH_TX, xdmaint);
	xdmac_channel_disable(XDMAC, HAL_PLC_XDMAC_CH_TX);
	xdmac_disable_interrupt(XDMAC, HAL_PLC_XDMAC_CH_TX);

	NVIC_ClearPendingIRQ(XDMAC_IRQn);
	NVIC_DisableIRQ(XDMAC_IRQn);
}

/**
 * \brief Reset and default configuration of SPI peripheral.
 *
 */
static void _spi_reset_and_config(void)
{
	uint32_t ul_cpuhz;
	uint32_t ul_plc_clock;
	uint8_t uc_div;

	ul_cpuhz = sysclk_get_peripheral_hz();
	ul_plc_clock = HAL_PLC_CLOCK;
	uc_div = ul_cpuhz / ul_plc_clock;

	if (ul_cpuhz % ul_plc_clock) {
		uc_div++;
	}

	/* Enable SPI peripheral. */
	spi_enable_clock(HAL_PLC_SPI_MODULE);

#ifndef HAL_PLC_SPI_SHARED_RF
	/* Reset SPI */
	spi_disable(HAL_PLC_SPI_MODULE);
	spi_reset(HAL_PLC_SPI_MODULE);

	/* Configure SPI */
	spi_set_master_mode(HAL_PLC_SPI_MODULE);
	spi_disable_mode_fault_detect(HAL_PLC_SPI_MODULE);
	spi_set_peripheral_chip_select_value(HAL_PLC_SPI_MODULE, HAL_PLC_PCS);
	spi_set_clock_polarity(HAL_PLC_SPI_MODULE, HAL_PLC_CS, 0);
	spi_set_clock_phase(HAL_PLC_SPI_MODULE, HAL_PLC_CS, 1);
#ifdef HAL_ATPL360_INTERFACE
	spi_set_bits_per_transfer(HAL_PLC_SPI_MODULE, HAL_PLC_CS, SPI_CSR_BITS_16_BIT);
#else
	spi_set_bits_per_transfer(HAL_PLC_SPI_MODULE, HAL_PLC_CS, SPI_CSR_BITS_8_BIT);
#endif
	spi_set_fixed_peripheral_select(HAL_PLC_SPI_MODULE);
	spi_set_baudrate_div(HAL_PLC_SPI_MODULE, HAL_PLC_CS, uc_div);
	spi_set_transfer_delay(HAL_PLC_SPI_MODULE, HAL_PLC_CS, HAL_PLC_DLYBS, HAL_PLC_DLYBCT);
	spi_configure_cs_behavior(HAL_PLC_SPI_MODULE, HAL_PLC_CS, SPI_CS_RISE_NO_TX);
	spi_enable(HAL_PLC_SPI_MODULE);
#else
	if (spi_is_enabled(HAL_PLC_SPI_MODULE) == 0) {
		/* Reset and configure SPI. Common registers for different Chip
		 * Selects are only written if SPI is not enabled yet */
		spi_disable(HAL_PLC_SPI_MODULE);
		spi_reset(HAL_PLC_SPI_MODULE);
		spi_set_master_mode(HAL_PLC_SPI_MODULE);
		spi_disable_mode_fault_detect(HAL_PLC_SPI_MODULE);
		spi_set_fixed_peripheral_select(HAL_PLC_SPI_MODULE);
		spi_enable(HAL_PLC_SPI_MODULE);
	}

	/* Configure SPI */
	spi_set_clock_polarity(HAL_PLC_SPI_MODULE, HAL_PLC_CS, HAL_PLC_POL);
	spi_set_clock_phase(HAL_PLC_SPI_MODULE, HAL_PLC_CS, HAL_PLC_PHA);
	spi_set_baudrate_div(HAL_PLC_SPI_MODULE, HAL_PLC_CS, uc_div);
	spi_set_transfer_delay(HAL_PLC_SPI_MODULE, HAL_PLC_CS, HAL_PLC_DLYBS, HAL_PLC_DLYBCT);
	spi_configure_cs_behavior(HAL_PLC_SPI_MODULE, HAL_PLC_CS, SPI_CS_RISE_NO_TX);
#endif
}

/**
 * \internal
 * \brief Initialize Proxy PLC controller.
 *
 * This function will change the system clock prescaler configuration to
 * match the parameters.
 *
 * \note The parameters to this function are device-specific.
 *
 */
static void _plc_if_config(void)
{
	/* Reset and configure SPI */
	_spi_reset_and_config();

	/* Initialize and enable DMA controller */
	pmc_enable_periph_clk(ID_XDMAC);

	/* Turn off XDMAC initially */
	_xdmac_disable();
#ifdef HAL_ATPL360_INTERFACE
	/* Configure TX and RX XDMAC channels for BOOT Commands */
	xdmac_tx_channel_cfg_boot.mbr_sa = (uint32_t)gs_plc_tx_buffer;
	xdmac_tx_channel_cfg_boot.mbr_da = (uint32_t)spi_get_tx_access(HAL_PLC_SPI_MODULE);
	xdmac_tx_channel_cfg_boot.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_DSYNC_MEM2PER |
			XDMAC_CC_PERID(HAL_PLC_XDMAC_SPI_TX_PERID) |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_MEMSET_NORMAL_MODE |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DWIDTH_BYTE |
			XDMAC_CC_SIF_AHB_IF0 |
			XDMAC_CC_DIF_AHB_IF1 |
			XDMAC_CC_SAM_INCREMENTED_AM |
			XDMAC_CC_DAM_FIXED_AM;
	xdmac_tx_channel_cfg_boot.mbr_bc = 0;
	xdmac_tx_channel_cfg_boot.mbr_ds = 0;
	xdmac_tx_channel_cfg_boot.mbr_sus = 0;
	xdmac_tx_channel_cfg_boot.mbr_dus = 0;

	xdmac_rx_channel_cfg_boot.mbr_sa = (uint32_t)spi_get_rx_access(HAL_PLC_SPI_MODULE);
	xdmac_rx_channel_cfg_boot.mbr_da = (uint32_t)gs_plc_rx_buffer;
	xdmac_rx_channel_cfg_boot.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_DSYNC_PER2MEM |
			XDMAC_CC_PERID(HAL_PLC_XDMAC_SPI_RX_PERID) |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_MEMSET_NORMAL_MODE |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DWIDTH_BYTE |
			XDMAC_CC_SIF_AHB_IF1 |
			XDMAC_CC_DIF_AHB_IF0 |
			XDMAC_CC_SAM_FIXED_AM |
			XDMAC_CC_DAM_INCREMENTED_AM;
	xdmac_rx_channel_cfg_boot.mbr_bc = 0;
	xdmac_rx_channel_cfg_boot.mbr_ds = 0;
	xdmac_rx_channel_cfg_boot.mbr_sus = 0;
	xdmac_rx_channel_cfg_boot.mbr_dus = 0;

	/* Configure TX and RX XDMAC channels for PHY Commands (after FW is uploaded)*/
	xdmac_tx_channel_cfg_phy.mbr_sa = (uint32_t)gs_plc_tx_buffer;
	xdmac_tx_channel_cfg_phy.mbr_da = (uint32_t)spi_get_tx_access(HAL_PLC_SPI_MODULE);
	xdmac_tx_channel_cfg_phy.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_DSYNC_MEM2PER |
			XDMAC_CC_PERID(HAL_PLC_XDMAC_SPI_TX_PERID) |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_MEMSET_NORMAL_MODE |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DWIDTH_HALFWORD |
			XDMAC_CC_SIF_AHB_IF0 |
			XDMAC_CC_DIF_AHB_IF1 |
			XDMAC_CC_SAM_INCREMENTED_AM |
			XDMAC_CC_DAM_FIXED_AM;
	xdmac_tx_channel_cfg_phy.mbr_bc = 0;
	xdmac_tx_channel_cfg_phy.mbr_ds = 0;
	xdmac_tx_channel_cfg_phy.mbr_sus = 0;
	xdmac_tx_channel_cfg_phy.mbr_dus = 0;

	xdmac_rx_channel_cfg_phy.mbr_sa = (uint32_t)spi_get_rx_access(HAL_PLC_SPI_MODULE);
	xdmac_rx_channel_cfg_phy.mbr_da = (uint32_t)gs_plc_rx_buffer;
	xdmac_rx_channel_cfg_phy.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_DSYNC_PER2MEM |
			XDMAC_CC_PERID(HAL_PLC_XDMAC_SPI_RX_PERID) |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_MEMSET_NORMAL_MODE |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DWIDTH_HALFWORD |
			XDMAC_CC_SIF_AHB_IF1 |
			XDMAC_CC_DIF_AHB_IF0 |
			XDMAC_CC_SAM_FIXED_AM |
			XDMAC_CC_DAM_INCREMENTED_AM;
	xdmac_rx_channel_cfg_phy.mbr_bc = 0;
	xdmac_rx_channel_cfg_phy.mbr_ds = 0;
	xdmac_rx_channel_cfg_phy.mbr_sus = 0;
	xdmac_rx_channel_cfg_phy.mbr_dus = 0;
#else
	/* Configure TX and RX XDMAC channels for ATPL230 */
	xdmac_tx_channel_cfg.mbr_sa = (uint32_t)gs_plc_tx_buffer;
	xdmac_tx_channel_cfg.mbr_da = (uint32_t)spi_get_tx_access(HAL_PLC_SPI_MODULE);
	xdmac_tx_channel_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_DSYNC_MEM2PER |
			XDMAC_CC_PERID(HAL_PLC_XDMAC_SPI_TX_PERID) |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_MEMSET_NORMAL_MODE |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DWIDTH_BYTE |
			XDMAC_CC_SIF_AHB_IF0 |
			XDMAC_CC_DIF_AHB_IF1 |
			XDMAC_CC_SAM_INCREMENTED_AM |
			XDMAC_CC_DAM_FIXED_AM;
	xdmac_tx_channel_cfg.mbr_bc = 0;
	xdmac_tx_channel_cfg.mbr_ds = 0;
	xdmac_tx_channel_cfg.mbr_sus = 0;
	xdmac_tx_channel_cfg.mbr_dus = 0;

	xdmac_rx_channel_cfg.mbr_sa = (uint32_t)spi_get_rx_access(HAL_PLC_SPI_MODULE);
	xdmac_rx_channel_cfg.mbr_da = (uint32_t)gs_plc_rx_buffer;
	xdmac_rx_channel_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
			XDMAC_CC_DSYNC_PER2MEM |
			XDMAC_CC_PERID(HAL_PLC_XDMAC_SPI_RX_PERID) |
			XDMAC_CC_CSIZE_CHK_1 |
			XDMAC_CC_MEMSET_NORMAL_MODE |
			XDMAC_CC_MBSIZE_SINGLE |
			XDMAC_CC_DWIDTH_BYTE |
			XDMAC_CC_SIF_AHB_IF1 |
			XDMAC_CC_DIF_AHB_IF0 |
			XDMAC_CC_SAM_FIXED_AM |
			XDMAC_CC_DAM_INCREMENTED_AM;
	xdmac_rx_channel_cfg.mbr_bc = 0;
	xdmac_rx_channel_cfg.mbr_ds = 0;
	xdmac_rx_channel_cfg.mbr_sus = 0;
	xdmac_rx_channel_cfg.mbr_dus = 0;
#endif /* HAL_ATPL360_INTERFACE */
}


/**
 * \brief Reset internal PLC Modem.
 *
 */
void hal_plc_reset(void)
{
#ifdef HAL_PLC_ARST_GPIO
	uint32_t ul_delay = 0x00FFFFFF;

	/* Reset on ARST of modem PLC */
	gpio_set_pin_low(HAL_PLC_ARST_GPIO);
	/* Clear ARST of modem PLC */
	gpio_set_pin_high(HAL_PLC_ARST_GPIO);

	/* Wait to initialize system */
	while (ul_delay) {
		ul_delay--;
	}
#else
	/* ATPL360/460 device */
	/* Enable LDO line */
	ioport_set_pin_level(HAL_PLC_LDO_EN_GPIO, HAL_PLC_LDO_EN_ACTIVE_LEVEL);
	ioport_set_pin_dir(HAL_PLC_LDO_EN_GPIO, IOPORT_DIR_OUTPUT);
	delay_ms(1);

	/* Reset on RST of modem PLC */
	ioport_set_pin_level(HAL_PLC_RESET_GPIO, HAL_PLC_RESET_ACTIVE_LEVEL);
	delay_ms(1);
	/* Clear RST of modem PLC */
	ioport_set_pin_level(HAL_PLC_RESET_GPIO, HAL_PLC_RESET_INACTIVE_LEVEL);
	delay_ms(50);
#endif
}

/**
 * \brief Initialize PLC interface
 *
 */
void hal_plc_init(void)
{
	/* Init PLC handler */
	plc_handler = NULL;
	/* signalling handler must be set in the application. */
	plc_tx_signalling_handler = NULL;
	plc_rx_signalling_handler = NULL;

	/* Initialize PLC */
	_plc_if_config();

#ifdef HAL_PLC_ARST_GPIO
	/* Configure PLC reset pins */
	ioport_set_pin_dir(HAL_PLC_ARST_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(HAL_PLC_ARST_GPIO, HAL_PLC_ARST_INACTIVE_LEVEL);

	ioport_set_pin_dir(HAL_PLC_SRST_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(HAL_PLC_SRST_GPIO, HAL_PLC_SRST_INACTIVE_LEVEL);
#else
	/* ATPL360/460 device */
  #ifdef HAL_PLC_STBY_GPIO
	/* Configure STBY pin */
	ioport_set_pin_level(HAL_PLC_STBY_GPIO, HAL_PLC_STBY_INACTIVE_LEVEL);
	ioport_set_pin_dir(HAL_PLC_STBY_GPIO, IOPORT_DIR_OUTPUT);
  #endif

	/* Configure LDO_EN pin */
	ioport_set_pin_level(HAL_PLC_LDO_EN_GPIO, HAL_PLC_LDO_EN_ACTIVE_LEVEL);
	ioport_set_pin_dir(HAL_PLC_LDO_EN_GPIO, IOPORT_DIR_OUTPUT);

	/* Configure PLC reset pin */
	ioport_set_pin_level(HAL_PLC_RESET_GPIO, HAL_PLC_RESET_ACTIVE_LEVEL);
	ioport_set_pin_dir(HAL_PLC_RESET_GPIO, IOPORT_DIR_OUTPUT);

  #ifdef HAL_PLC_NTHW0_GPIO
	/* Configure NTHW0 pin (only PL460/PL480) */
	ioport_set_pin_dir(HAL_PLC_NTHW0_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(HAL_PLC_NTHW0_GPIO, IOPORT_MODE_PULLUP);
  #endif

  #ifdef HAL_PLC_CD_GPIO
	/* Configure CD pin */
	ioport_set_pin_dir(HAL_PLC_CD_GPIO, IOPORT_DIR_INPUT);
  #endif

  #ifdef HAL_PLC_TX_ENABLE_GPIO
    #ifdef HAL_PLC_CD_GPIO
      #error Carrier Detect (PL360) and TX Enable (PL460) cannot be used at the same time
    #endif

	/* Configure TX Enable pin (only PL460/480) */
	ioport_set_pin_level(HAL_PLC_TX_ENABLE_GPIO, HAL_PLC_TX_ENABLE_ACTIVE_LEVEL);
	ioport_set_pin_dir(HAL_PLC_TX_ENABLE_GPIO, IOPORT_DIR_OUTPUT);
  #endif

  #ifdef HAL_PLC_PVDD_MON_ADC_CHN
	/* Initialize PVDD monitor */
	_pvdd_mon_init();
  #endif
#endif
}

/**
 * \brief Set an interrupt handler for the specified interrupt source.
 */
void hal_plc_set_handler(void (*p_handler)(void))
{
	plc_handler = p_handler;

	/* Configure PLC interruption pin */
	ioport_set_pin_mode(HAL_PLC_INT_GPIO, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_dir(HAL_PLC_INT_GPIO, IOPORT_DIR_INPUT);

	/* Configure PLC Interruption */
	pmc_enable_periph_clk(HAL_PLC_INT_ID);
	pio_handler_set(HAL_PLC_INT_PIO, HAL_PLC_INT_ID, HAL_PLC_INT_MASK, HAL_PLC_INT_ATTR, hal_plc_int_handler);

	NVIC_SetPriority(HAL_PLC_INT_IRQn, HAL_PLC_PRIO);
	NVIC_ClearPendingIRQ(HAL_PLC_INT_IRQn);
	NVIC_EnableIRQ(HAL_PLC_INT_IRQn);
#ifndef HAL_ATPL360_INTERFACE
	pio_enable_interrupt(HAL_PLC_INT_PIO, HAL_PLC_INT_MASK);
#endif
}

/**
 * \brief Set callback for PVDD monitor. Callback handler will be used to notify
 * PVDD events (true: PVDD within allowed range; false: PVDD out of allowed
 * range). Handler is only called when PVDD status changes (from good to bad or
 * from bad to good). It will be called inside this function to indicate the
 * current PVDD status.
 *
 * \param p_handler Handler to notify PVDD event
 */
void hal_plc_pvdd_mon_set_handler(void (*p_handler)(bool))
{
#ifdef HAL_PLC_PVDD_MON_ADC_CHN
	hal_plc_pvdd_mon_handler = p_handler;
	if (hal_plc_pvdd_mon_handler != NULL) {
		hal_plc_pvdd_mon_handler(sb_pvdd_good);
	}

#else
	UNUSED(p_handler);
#endif
}

/**
 * \brief Set a signaling handler for the PLC transmission.
 */
void hal_plc_set_tx_signalling_handler(void (*p_handler)(void))
{
	plc_tx_signalling_handler = p_handler;
}

/**
 * \brief Set a signaling handler for the PLC reception.
 */
void hal_plc_set_rx_signalling_handler(void (*p_handler)(void))
{
	plc_rx_signalling_handler = p_handler;
}

/**
 * \brief Function called in PLC TX event
 */
void hal_plc_tx_signal(void)
{
	if (plc_tx_signalling_handler) {
		plc_tx_signalling_handler();
	}
}

/**
 * \brief Function called in PLC RX event
 */
void hal_plc_rx_signal(void)
{
	if (plc_rx_signalling_handler) {
		plc_rx_signalling_handler();
	}
}

#ifdef HAL_ATPL360_INTERFACE
bool hal_plc_send_boot_cmd(uint16_t us_cmd, uint32_t ul_addr, uint32_t ul_data_len, uint8_t *puc_data_buf, uint8_t *puc_data_read)
{
	uint8_t *puc_tx_buf;
	uint32_t ul_spi_busy_cnt;
	uint16_t us_tx_size;

	/* Check length */
	if ((ul_data_len + 6) > HAL_PLC_BUFFER_SIZE) {
		return false;
	}

	/* Disable PLC interrupt to avoid SPI access in the middle of a transaction */
	NVIC_DisableIRQ(HAL_PLC_INT_IRQn);

	/* Check SPI status */
	if (suc_plc_is_busy) {
		return false;
	}

	/* Update SPI status */
	suc_plc_is_busy = true;

#ifdef HAL_PLC_SPI_SHARED_RF

	/* Enter critical region. Disable all interrupts except highest priority
	 * (<2: 0, 1) to avoid wrong SPI transaction if same SPI (shared with
	 * RF) is used from IRQ */
	uint32_t ul_basepri_prev = __get_BASEPRI();
	if (ul_basepri_prev != (1 << (8 - __NVIC_PRIO_BITS))) {
		__set_BASEPRI(2 << (8 - __NVIC_PRIO_BITS));
	}
#endif

	/* Waiting transfer done*/
	ul_spi_busy_cnt = 0;
	while ((xdmac_channel_get_status(XDMAC) & (HAL_PLC_XDMAC_TX_STATUS | HAL_PLC_XDMAC_RX_STATUS)) ||
			!(spi_read_status(HAL_PLC_SPI_MODULE) & SPI_SR_TXEMPTY)) {
		ul_spi_busy_cnt++;
		if (ul_spi_busy_cnt > 5000000) {
			/* Update SPI status */
			suc_plc_is_busy = false;
			/* Enable PLC interrupt */
			NVIC_EnableIRQ(HAL_PLC_INT_IRQn);
#ifdef HAL_PLC_SPI_SHARED_RF
			__set_BASEPRI(ul_basepri_prev);
#endif
			return false;
		}
	}

	/* Disable the RX and TX XDMAC transfer requests */
	xdmac_channel_disable(XDMAC, HAL_PLC_XDMAC_CH_RX);
	xdmac_channel_disable(XDMAC, HAL_PLC_XDMAC_CH_TX);

	/* Set 8 bits per transfer */
	spi_set_bits_per_transfer(HAL_PLC_SPI_MODULE, HAL_PLC_CS, SPI_CSR_BITS_8_BIT);

#ifdef HAL_PLC_SPI_SHARED_RF

	/* Set chip select. Done in every SPI transaction in case same SPI with
	 * another CS is used by another module */
	spi_set_peripheral_chip_select_value(HAL_PLC_SPI_MODULE, HAL_PLC_PCS);

	/* Read received data from SPI in order to clear RDRF flag (if previous
	 * transaction with RF215 was in write mode the received data is not
	 * read). Otherwise the XDMAC would read first the last received byte
	 * from previous transaction */
	spi_get(HAL_PLC_SPI_MODULE);
#endif

	/* Configure Tx buffer */
	puc_tx_buf = gs_plc_tx_buffer;

	memcpy(puc_tx_buf, &ul_addr, sizeof(uint32_t));
	puc_tx_buf +=  sizeof(uint32_t);
	memcpy(puc_tx_buf, &us_cmd, sizeof(uint16_t));
	puc_tx_buf +=  sizeof(uint16_t);

	memcpy(puc_tx_buf, puc_data_buf, ul_data_len);

	puc_tx_buf += ul_data_len;

	us_tx_size = puc_tx_buf - gs_plc_tx_buffer;

#ifdef HAL_PLC_CM7_CACHE_MAINTENANCE
	/* Clean DMA Tx buffer cache to avoid incoherency issues */
	SCB_CleanDCache_by_Addr((uint32_t *)gs_plc_tx_buffer, us_tx_size);
#endif

	/* Configure TX and RX XDMAC channels */
	xdmac_rx_channel_cfg_boot.mbr_ubc = us_tx_size;
	xdmac_tx_channel_cfg_boot.mbr_ubc = us_tx_size;

	xdmac_configure_transfer(XDMAC, HAL_PLC_XDMAC_CH_RX, &xdmac_rx_channel_cfg_boot);
	xdmac_channel_set_descriptor_control(XDMAC, HAL_PLC_XDMAC_CH_RX, 0);

	xdmac_configure_transfer(XDMAC, HAL_PLC_XDMAC_CH_TX, &xdmac_tx_channel_cfg_boot);
	xdmac_channel_set_descriptor_control(XDMAC, HAL_PLC_XDMAC_CH_TX, 0);

	/* Enable the RX and TX XDMAC transfer requests */
	xdmac_channel_enable(XDMAC, HAL_PLC_XDMAC_CH_RX);
	xdmac_channel_enable(XDMAC, HAL_PLC_XDMAC_CH_TX);

#ifdef HAL_PLC_SPI_SHARED_RF
	/* Protection: Wait until SPI is detected as busy */
	ul_spi_busy_cnt = 0;
	while (spi_read_status(HAL_PLC_SPI_MODULE) & SPI_SR_TXEMPTY) {
		ul_spi_busy_cnt++;
		if (ul_spi_busy_cnt > 50) {
			/* It could have started and finished already */
			break;
		}
	}

	/* Leave critical region. The SPI transaction has started and interrupts
	 * can be enabled (RF SPI access function will block until this SPI
	 * transaction has finished) */
	__set_BASEPRI(ul_basepri_prev);
#endif

	/* Waiting transfer done and read */
	if (puc_data_read) {
#ifdef HAL_PLC_CM7_CACHE_MAINTENANCE
		/* Invalidate DMA Rx buffer cache to avoid incoherency issues */
		SCB_InvalidateDCache_by_Addr((uint32_t *)gs_plc_rx_buffer, us_tx_size);
#endif

		/* while(pdc_read_tx_counter(g_pdc) > 0); */
		ul_spi_busy_cnt = 0;
		while (xdmac_channel_get_status(XDMAC) & HAL_PLC_XDMAC_RX_STATUS) {
			ul_spi_busy_cnt++;
			if (ul_spi_busy_cnt > 5000000) {
				/* Update SPI status */
				suc_plc_is_busy = false;
				/* Enable PLC interrupt */
				NVIC_EnableIRQ(HAL_PLC_INT_IRQn);
				return false;
			}
		}

		memcpy(puc_data_read, &gs_plc_rx_buffer[6], ul_data_len);
	}

	/* Update SPI status */
	suc_plc_is_busy = false;

	/* Enable PLC interrupt */
	NVIC_EnableIRQ(HAL_PLC_INT_IRQn);

	return true;
}

bool hal_plc_send_wrrd_cmd(uint8_t uc_cmd, void *px_spi_data, void *px_spi_status_info)
{
	uint8_t *puc_tx_buf;
	uint32_t ul_spi_busy_cnt;
	uint16_t us_tx_size;
	uint16_t us_len_wr_rd;
	spi_data_t *px_data;
	spi_status_info_t *px_status_info;

	px_data = (spi_data_t *)px_spi_data;
	px_status_info = (spi_status_info_t *)px_spi_status_info;

	/* Check length */
	if ((px_data->us_len == 0) || ((px_data->us_len + HAL_PLC_SPI_HEADER_SIZE) > HAL_PLC_BUFFER_SIZE)) {
		return false;
	}

	/* Disable PLC interrupt to avoid SPI access in the middle of a transaction */
	NVIC_DisableIRQ(HAL_PLC_INT_IRQn);

	/* Check SPI status */
	if (suc_plc_is_busy) {
		return false;
	}

	/* Update SPI status */
	suc_plc_is_busy = true;

#ifdef HAL_PLC_SPI_SHARED_RF

	/* Enter critical region. Disable all interrupts except highest priority
	 * (<2: 0, 1) to avoid wrong SPI transaction if same SPI (shared with
	 * RF) is used from IRQ */
	uint32_t ul_basepri_prev = __get_BASEPRI();
	if (ul_basepri_prev != (1 << (8 - __NVIC_PRIO_BITS))) {
		__set_BASEPRI(2 << (8 - __NVIC_PRIO_BITS));
	}
#endif

	/* Waiting transfer done while(pdc_read_tx_counter(g_pdc) > 0); */
	ul_spi_busy_cnt = 0;
	while ((xdmac_channel_get_status(XDMAC) & (HAL_PLC_XDMAC_TX_STATUS | HAL_PLC_XDMAC_RX_STATUS)) ||
			!(spi_read_status(HAL_PLC_SPI_MODULE) & SPI_SR_TXEMPTY)) {
		ul_spi_busy_cnt++;
		if (ul_spi_busy_cnt > 5000000) {
			/* Update SPI status */
			suc_plc_is_busy = false;
			/* Enable PLC interrupt */
			NVIC_EnableIRQ(HAL_PLC_INT_IRQn);
#ifdef HAL_PLC_SPI_SHARED_RF
			__set_BASEPRI(ul_basepri_prev);
#endif
			return false;
		}
	}

	/* Disable the RX and TX XDMAC transfer requests */
	xdmac_channel_disable(XDMAC, HAL_PLC_XDMAC_CH_RX);
	xdmac_channel_disable(XDMAC, HAL_PLC_XDMAC_CH_TX);

	/* Set 16 bits per transfer */
	spi_set_bits_per_transfer(HAL_PLC_SPI_MODULE, HAL_PLC_CS, SPI_CSR_BITS_16_BIT);

#ifdef HAL_PLC_SPI_SHARED_RF

	/* Set chip select. Done in every SPI transaction in case same SPI with
	 * another CS is used by another module */
	spi_set_peripheral_chip_select_value(HAL_PLC_SPI_MODULE, HAL_PLC_PCS);

	/* Read received data from SPI in order to clear RDRF flag (if previous
	 * transaction with RF215 was in write mode the received data is not
	 * read). Otherwise the XDMAC would read first the last received byte
	 * from previous transaction */
	spi_get(HAL_PLC_SPI_MODULE);
#endif

	/** Configure PLC Tx buffer **/
	puc_tx_buf = gs_plc_tx_buffer;
	/* Address */
	*puc_tx_buf++ = (uint8_t)(px_data->us_address & 0xFF);
	*puc_tx_buf++ = (uint8_t)(px_data->us_address >> 8);
	/* Length & read/write */
	us_len_wr_rd = (((px_data->us_len + 1) / 2) & HAL_PLC_LEN_MASK) | (uc_cmd << HAL_PLC_WR_RD_POS);
	*puc_tx_buf++ = (uint8_t)(us_len_wr_rd);
	*puc_tx_buf++ = (uint8_t)(us_len_wr_rd >> 8);

	if (uc_cmd == HAL_PLC_CMD_WRITE) {
		memcpy(puc_tx_buf, px_data->puc_data_buf, px_data->us_len);
	} else {
		memset(puc_tx_buf, 0, px_data->us_len);
	}

	puc_tx_buf += px_data->us_len;

	us_tx_size = puc_tx_buf - gs_plc_tx_buffer;
	if (us_tx_size % 2) {
		/* Add 1 padding byte */
		*puc_tx_buf++ = 0;
		us_tx_size++;
	}

#ifdef HAL_PLC_CM7_CACHE_MAINTENANCE
	/* Clean DMA Tx buffer cache to avoid incoherency issues */
	SCB_CleanDCache_by_Addr((uint32_t *)gs_plc_tx_buffer, us_tx_size);
#endif

	/* Configure TX and RX XDMAC channels */
	xdmac_rx_channel_cfg_phy.mbr_ubc = us_tx_size / 2;
	xdmac_tx_channel_cfg_phy.mbr_ubc = us_tx_size / 2;

	xdmac_configure_transfer(XDMAC, HAL_PLC_XDMAC_CH_RX, &xdmac_rx_channel_cfg_phy);
	xdmac_channel_set_descriptor_control(XDMAC, HAL_PLC_XDMAC_CH_RX, 0);

	xdmac_configure_transfer(XDMAC, HAL_PLC_XDMAC_CH_TX, &xdmac_tx_channel_cfg_phy);
	xdmac_channel_set_descriptor_control(XDMAC, HAL_PLC_XDMAC_CH_TX, 0);

	/* Enable the RX and TX XDMAC transfer requests */
	xdmac_channel_enable(XDMAC, HAL_PLC_XDMAC_CH_RX);
	xdmac_channel_enable(XDMAC, HAL_PLC_XDMAC_CH_TX);

#ifdef HAL_PLC_SPI_SHARED_RF
	/* Protection: Wait until SPI is detected as busy */
	ul_spi_busy_cnt = 0;
	while (spi_read_status(HAL_PLC_SPI_MODULE) & SPI_SR_TXEMPTY) {
		ul_spi_busy_cnt++;
		if (ul_spi_busy_cnt > 50) {
			/* It could have started and finished already */
			break;
		}
	}

	/* Leave critical region. The SPI transaction has started and interrupts
	 * can be enabled (RF SPI access function will block until this SPI
	 * transaction has finished) */
	__set_BASEPRI(ul_basepri_prev);
#endif

#ifdef HAL_PLC_CM7_CACHE_MAINTENANCE
	/* Invalidate DMA Rx buffer cache to avoid incoherency issues */
	SCB_InvalidateDCache_by_Addr((uint32_t *)gs_plc_rx_buffer, us_tx_size);
#endif

	/* Waiting transfer done*/
	/* while(pdc_read_tx_counter(g_pdc) > 0); */
	ul_spi_busy_cnt = 0;
	while (xdmac_channel_get_status(XDMAC) & HAL_PLC_XDMAC_RX_STATUS) {
		ul_spi_busy_cnt++;
		if (ul_spi_busy_cnt > 5000000) {
			/* Update SPI status */
			suc_plc_is_busy = false;
			/* Enable PLC interrupt */
			NVIC_EnableIRQ(HAL_PLC_INT_IRQn);
			return false;
		}
	}

	if (uc_cmd == HAL_PLC_CMD_READ) {
		memcpy(px_data->puc_data_buf, &gs_plc_rx_buffer[HAL_PLC_SPI_HEADER_SIZE], px_data->us_len);
	}

	px_status_info->us_header_id = HAL_PLC_GET_ID_HEADER(gs_plc_rx_buffer[0], gs_plc_rx_buffer[1]);
	if (HAL_PLC_CHECK_ID_BOOT_HEADER(px_status_info->us_header_id)) {
		px_status_info->ul_flags = HAL_PLC_GET_FLAGS_FROM_BOOT(gs_plc_rx_buffer[0], gs_plc_rx_buffer[2], gs_plc_rx_buffer[3]);
	} else if (HAL_PLC_CHECK_ID_CORTEX_HEADER(px_status_info->us_header_id)) {
		px_status_info->ul_flags = HAL_PLC_GET_FLAGS_FROM_CORTEX(gs_plc_rx_buffer[2], gs_plc_rx_buffer[3]);
	} else {
		px_status_info->ul_flags = 0;
	}

	/* Update SPI status */
	suc_plc_is_busy = false;

	/* Enable PLC interrupt(); */
	NVIC_EnableIRQ(HAL_PLC_INT_IRQn);

	return true;
}

void hal_plc_enable_interrupt(bool enable)
{
	if (enable) {
		pio_enable_interrupt(HAL_PLC_INT_PIO, HAL_PLC_INT_MASK);
	} else {
		pio_disable_interrupt(HAL_PLC_INT_PIO, HAL_PLC_INT_MASK);
	}
}

void hal_plc_delay(uint8_t uc_tref, uint32_t ul_delay)
{
	if (uc_tref == HAL_TREF_SEC) {
		delay_s(ul_delay);
	} else if (uc_tref == HAL_TREF_MS) {
		delay_ms(ul_delay);
	} else if (uc_tref == HAL_TREF_US) {
		delay_us(ul_delay);
	}
}

bool hal_plc_set_stby_mode(bool sleep)
{
#ifdef HAL_PLC_STBY_GPIO
	if (sleep) {
		/* Set RESET pin */
		ioport_set_pin_level(HAL_PLC_RESET_GPIO, HAL_PLC_RESET_ACTIVE_LEVEL);
		/* Set STBY pin */
		ioport_set_pin_level(HAL_PLC_STBY_GPIO, HAL_PLC_STBY_ACTIVE_LEVEL);
	} else {
		/* Clear STBY pin */
		ioport_set_pin_level(HAL_PLC_STBY_GPIO, HAL_PLC_STBY_INACTIVE_LEVEL);
		/* Wait STBY disabled */
		delay_us(100);
		/* Clear RESET pin */
		ioport_set_pin_level(HAL_PLC_RESET_GPIO, HAL_PLC_RESET_INACTIVE_LEVEL);
		/* Wait RESET disabled */
		delay_us(750);
	}

	return true;
#else
	/* STBY pin not available */
	return false;
#endif
}

bool hal_plc_get_thermal_warning(void)
{
#ifdef HAL_PLC_NTHW0_GPIO
	if (ioport_get_pin_level(HAL_PLC_NTHW0_GPIO)) {
		/* NTHW0 high level: Normal condition */
		return false;
	} else {
		/* NTHW0 low level: High temperature (>110�C) condition */
		return true;
	}

#else
	/* NTHW0 pin not available */
	return false;
#endif
}

/**
 * \brief Critical initialization of HAL_PLC interface. PL360 control pins are
 * initialized (otherwise all GPIO with pull-up at reset). This function should
 * be called as soon as possible after Reset_Handler to avoid issues in
 * PL460/480 without pull-down resistor in the PLC reset pin.
 * This initialization is not critical for PL360/485 or PL460/480 with pull-down
 * resistor in the PLC reset pin.
 */
void hal_plc_crit_init(void)
{
	/* Configure PLC reset pin */
	pmc_enable_periph_clk(pio_get_pin_group_id(HAL_PLC_RESET_GPIO));
	ioport_set_pin_level(HAL_PLC_RESET_GPIO, HAL_PLC_RESET_ACTIVE_LEVEL);
	ioport_set_pin_dir(HAL_PLC_RESET_GPIO, IOPORT_DIR_OUTPUT);

#ifdef HAL_PLC_STBY_GPIO
	/* Configure STBY pin */
	pmc_enable_periph_clk(pio_get_pin_group_id(HAL_PLC_STBY_GPIO));
	ioport_set_pin_level(HAL_PLC_STBY_GPIO, HAL_PLC_STBY_INACTIVE_LEVEL);
	ioport_set_pin_dir(HAL_PLC_STBY_GPIO, IOPORT_DIR_OUTPUT);
#endif

	/* Configure LDO_EN pin */
	pmc_enable_periph_clk(pio_get_pin_group_id(HAL_PLC_LDO_EN_GPIO));
	ioport_set_pin_level(HAL_PLC_LDO_EN_GPIO, HAL_PLC_LDO_EN_INACTIVE_LEVEL);
	ioport_set_pin_dir(HAL_PLC_LDO_EN_GPIO, IOPORT_DIR_OUTPUT);
}

#endif

/* / @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* / @endcond */
