/**
 * \file
 *
 * \brief HAL_PLC
 *
 * Copyright (c) 2024 Atmel Corporation. All rights reserved.
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
#include "board.h"
#include "ioport.h"
#include "pmc.h"
#include "pio.h"
#include "pio_handler.h"
#include "pdc.h"
#include "spi.h"
#include "hal_private.h"
#include "conf_hal.h"
#include "delay.h"

#ifdef HAL_ATPL360_INTERFACE
#include "atpl360_hal_spi.h"
#endif

#if SAM4C || SAM4CM || PIC32CX
# include "adc.h"
#elif SAMG55
# include "adc2.h"
#endif

#ifndef HAL_PLC_POL
#define HAL_PLC_POL           0
#endif

#ifndef HAL_PLC_PHA
#define HAL_PLC_PHA           1
#endif

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

#ifdef HAL_ATPL360_INTERFACE
/** SPI Header size. */
# define HAL_PLC_SPI_HEADER_SIZE           4
/** SPI Max Msg_Data size. */
# define HAL_PLC_SPI_MSG_DATA_SIZE         512
/** SPI Max Msg_Params size. */
# define HAL_PLC_SPI_MSG_PARAMS_SIZE       118   /* Worst case = 118: sizeof(rx_msg_t) [G3] */
/** PDC buffer size. */
# define HAL_PLC_BUFFER_SIZE               (HAL_PLC_SPI_HEADER_SIZE + HAL_PLC_SPI_MSG_DATA_SIZE + HAL_PLC_SPI_MSG_PARAMS_SIZE)
#else
/** PDC buffer size. */
# define HAL_PLC_BUFFER_SIZE               800
#endif

/** PDC Receive buffer */
COMPILER_ALIGNED(4)
static uint8_t gs_plc_rx_buffer[HAL_PLC_BUFFER_SIZE];
/** PDC Transmission buffer */
COMPILER_ALIGNED(4)
static uint8_t gs_plc_tx_buffer[HAL_PLC_BUFFER_SIZE];
/** PDC RX data packet */
pdc_packet_t g_plc_rx_packet;
/** PDC TX data packet */
pdc_packet_t g_plc_tx_packet;
/** Pointer to PDC register base */
Pdc *g_plc_pdc;

/** PLC busy indication */
static uint8_t suc_plc_is_busy;

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

# if SAM4C || SAM4CM
#  define HAL_PLC_PVDD_MON_ADC_BITS               10
# elif SAMG55 || PIC32CX
#  define HAL_PLC_PVDD_MON_ADC_BITS               12
# endif

/** High and Low threshold ADC values (10 bits) */
# define HAL_PLC_PVDD_MON_HIGHTHRES_ADC_VAL       ((uint16_t)div_round((uint64_t)HAL_PLC_PVDD_MON_HIGHTHRES_NUM << HAL_PLC_PVDD_MON_ADC_BITS, HAL_PLC_PVDD_MON_THRES_DEN))
# define HAL_PLC_PVDD_MON_LOWTHRES_ADC_VAL        ((uint16_t)div_round((uint64_t)HAL_PLC_PVDD_MON_LOWTHRES_NUM << HAL_PLC_PVDD_MON_ADC_BITS, HAL_PLC_PVDD_MON_THRES_DEN))
# define HAL_PLC_PVDD_MON_HIGHTHRES_HYST_ADC_VAL  ((uint16_t)div_round((uint64_t)HAL_PLC_PVDD_MON_HIGHTHRES_HYST_NUM << HAL_PLC_PVDD_MON_ADC_BITS, HAL_PLC_PVDD_MON_THRES_DEN))
# define HAL_PLC_PVDD_MON_LOWTHRES_HYST_ADC_VAL   ((uint16_t)div_round((uint64_t)HAL_PLC_PVDD_MON_LOWTHRES_HYST_NUM << HAL_PLC_PVDD_MON_ADC_BITS, HAL_PLC_PVDD_MON_THRES_DEN))

/* ADC clock frequency, tracking time and startup time must be configured
 * according to datasheet recommendations.
 * For tracking time computations, source resistance of external circuitry is
 * Rsource = 1k + 10k||36k = 8.83k  */

# if SAM4C || SAM4CM
/** ADC clock frequency (in Hz)  */
#  ifndef HAL_PLC_PVDD_MON_ADC_CLK_FREQ
/* 6 MHz by default */
#   define HAL_PLC_PVDD_MON_ADC_CLK_FREQ          6000000
#  endif

/** ADC tracking time: 1-16 cycles of ADC clock.
 * From datasheet: tTRACKTIM (ns) >= 0.12 * Rsource (ohm) + 500 = 1.56 us */
#  ifndef HAL_PLC_PVDD_MON_ADC_TRACK_TIME
/* 15 cycles by default (2.5 us at 6 MHz) */
#   define HAL_PLC_PVDD_MON_ADC_TRACK_TIME        14
#  endif

/** ADC startup time in clycles of ADC clock.
 * From datasheet: tSTART >= 40 us */
#  ifndef HAL_PLC_PVDD_MON_ADC_STARTUP_TIME
/* 512 cycles by default (85.3 us at 6 MHz) */
#   define HAL_PLC_PVDD_MON_ADC_STARTUP_TIME      ADC_STARTUP_TIME_8
#  endif

# elif PIC32CX
/** ADC clock frequency (in Hz)  */
#  ifndef HAL_PLC_PVDD_MON_ADC_CLK_FREQ
/* 10 MHz by default */
#   define HAL_PLC_PVDD_MON_ADC_CLK_FREQ          10000000
#  endif

/** ADC tracking time: 1-246 cycles of ADC clock.
 * From datasheet: tTRACKTIM (ns) >= 0.024 * Rsource (ohm) + 192 = 0.404 us */
#  ifndef HAL_PLC_PVDD_MON_ADC_TRACK_TIME
/* 7 cycles by default (0.7 us at 10 MHz; TRACKX = 0) */
#   define HAL_PLC_PVDD_MON_ADC_TRACK_TIME        15
#  endif

/** ADC startup time in clycles of ADC clock.
 * From datasheet: tSTART >= 5 us */
#  ifndef HAL_PLC_PVDD_MON_ADC_STARTUP_TIME
/* 80 cycles by default (8 us at 12.5 MHz) */
#   define HAL_PLC_PVDD_MON_ADC_STARTUP_TIME      ADC_STARTUP_TIME_5
#  endif

# elif SAMG55
/** ADC clock frequency (in Hz)  */
#  ifndef HAL_PLC_PVDD_MON_ADC_CLK_FREQ
/* 3.2 MHz by default */
#   define HAL_PLC_PVDD_MON_ADC_CLK_FREQ          3200000
#  endif

/** ADC tracking time: 6 (0-14) or 7 (15) cycles of ADC clock.
 * From datasheet: tTRACKTIM (ns) >= 0.12 * Rsource (ohm) + 250 = 1.31 us */
#  ifndef HAL_PLC_PVDD_MON_ADC_TRACK_TIME
/* 7 cycles by default (2.19 us at 3.2 MHz) */
#   define HAL_PLC_PVDD_MON_ADC_TRACK_TIME        15
#  endif

/** ADC startup time in clycles of ADC clock.
 * From datasheet: tSTART >= 4 us */
#  ifndef HAL_PLC_PVDD_MON_ADC_STARTUP_TIME
/* 24 cycles by default (7.5 us at 4 MHz) */
#   define HAL_PLC_PVDD_MON_ADC_STARTUP_TIME      ADC_STARTUP_TIME_3
#  endif
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
# if SAM4C || SAM4CM || PIC32CX
		adc_set_comparison_window(ADC, HAL_PLC_PVDD_MON_LOWTHRES_ADC_VAL, HAL_PLC_PVDD_MON_HIGHTHRES_ADC_VAL);
		adc_set_comparison_filter(ADC, 0);
		adc_set_comparison_mode(ADC, ADC_EMR_CMPMODE_OUT);
# elif SAMG55
		adc_set_comparison_window(ADC, HAL_PLC_PVDD_MON_LOWTHRES_ADC_VAL, HAL_PLC_PVDD_MON_HIGHTHRES_ADC_VAL);
		adc_set_comparison_mode(ADC, ADC_CMP_MODE_3, HAL_PLC_PVDD_MON_ADC_CHN, 0);
# endif
	} else {
		/* PVDD out of allowed range: Set compare mode to IN */
# if SAM4C || SAM4CM || PIC32CX
		adc_set_comparison_window(ADC, HAL_PLC_PVDD_MON_LOWTHRES_HYST_ADC_VAL, HAL_PLC_PVDD_MON_HIGHTHRES_HYST_ADC_VAL);
		adc_set_comparison_filter(ADC, 3);
		adc_set_comparison_mode(ADC, ADC_EMR_CMPMODE_IN);
# elif SAMG55
		adc_set_comparison_window(ADC, HAL_PLC_PVDD_MON_LOWTHRES_HYST_ADC_VAL, HAL_PLC_PVDD_MON_HIGHTHRES_HYST_ADC_VAL);
		adc_set_comparison_mode(ADC, ADC_CMP_MODE_2, HAL_PLC_PVDD_MON_ADC_CHN, 3);
# endif
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
# if SAM4C || SAM4CM || PIC32CX
	us_adc_value = (uint16_t)adc_get_channel_value(ADC, HAL_PLC_PVDD_MON_ADC_CHN);
# elif SAMG55
	us_adc_value = (uint16_t)adc_channel_get_value(ADC, HAL_PLC_PVDD_MON_ADC_CHN);
# endif

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

# if SAM4C || SAM4CM || PIC32CX

/**
 * \brief Interrupt handler for ADC.
 */
void ADC_Handler(void)
{
	if (adc_get_status(ADC) & ADC_ISR_COMPE) {
		_pvdd_mon_adc_handler();
	}
}

# endif

/**
 * \brief Initialize PVDD monitor. Configure ADC using Comparison Window.
 */
static void _pvdd_mon_init(void)
{
	/* Initialize PVDD status: Handler only called if PVDD is bad */
	sb_pvdd_good = true;

# if SAM4C || SAM4CM || PIC32CX
	/* Enable ADC peripheral */
	pmc_enable_periph_clk(ID_ADC);

	/* ADC configuration */
	adc_init(ADC, sysclk_get_peripheral_hz(), HAL_PLC_PVDD_MON_ADC_CLK_FREQ, HAL_PLC_PVDD_MON_ADC_STARTUP_TIME);
#  if PIC32CX
	adc_configure_timing(ADC, HAL_PLC_PVDD_MON_ADC_TRACK_TIME, 2);
	adc_set_resolution(ADC, ADC_12_BITS);
#  else
	struct adc_internal_ref adc_int_ref;

	adc_configure_timing(ADC, HAL_PLC_PVDD_MON_ADC_TRACK_TIME);
	adc_set_resolution(ADC, ADC_10_BITS);

	adc_int_ref.adc_internal_ref_change_enable = false;
	adc_int_ref.volt = ADC_INTERNAL_REF_3275MV;
	adc_int_ref.adc_force_internal_ref = true;
	adc_int_ref.adc_internal_ref_on = false;
	adc_set_internal_reference_voltage(ADC, &adc_int_ref);
#  endif

	/* Enable channel used for PVDD monitor */
	adc_enable_channel(ADC, HAL_PLC_PVDD_MON_ADC_CHN);

	/* Set comparison window mode and thresholds */
	_pvdd_mon_set_comp_mode(true);
	adc_set_comparison_channel(ADC, HAL_PLC_PVDD_MON_ADC_CHN);

	/* Configure ADC in Free-Running mode */
#  if PIC32CX
	adc_set_trigger(ADC, ADC_TRGR_TRGMOD_CONTINUOUS, 0);
#  else
	adc_configure_trigger(ADC, ADC_TRIG_SW, 1);
#  endif

	/* Enable ADC interrupt for PVDD monitor */
	NVIC_SetPriority(ADC_IRQn, HAL_PLC_ADC_PRIO);
	NVIC_ClearPendingIRQ(ADC_IRQn);
	NVIC_EnableIRQ(ADC_IRQn);
	adc_enable_interrupt(ADC, ADC_ISR_COMPE);

# elif SAMG55
	struct adc_config adc_cfg;

	/* Enable ADC peripheral */
	adc_enable();

	/* ADC configuration */
	adc_cfg.resolution = ADC_12_BITS;
	adc_cfg.mck = sysclk_get_peripheral_hz();
	adc_cfg.adc_clock = HAL_PLC_PVDD_MON_ADC_CLK_FREQ;
	adc_cfg.startup_time = HAL_PLC_PVDD_MON_ADC_STARTUP_TIME;
	adc_cfg.tracktim = HAL_PLC_PVDD_MON_ADC_TRACK_TIME;
	adc_cfg.transfer = 2;
	adc_cfg.useq = false;
	adc_cfg.tag = false;
	adc_cfg.aste = false;
	adc_init(ADC, &adc_cfg);

	/* Enable channel used for PVDD monitor */
	adc_channel_enable(ADC, HAL_PLC_PVDD_MON_ADC_CHN);

	/* Set comparison window mode and thresholds */
	_pvdd_mon_set_comp_mode(true);

	/* Configure ADC in Free-Running mode */
	adc_set_trigger(ADC, ADC_TRIG_FREERUN);

	/* Set ADC callback for PVDD monitor */
	adc_set_callback(ADC, ADC_INTERRUPT_COMP_ERROR, _pvdd_mon_adc_handler, HAL_PLC_ADC_PRIO);
# endif
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
	spi_set_clock_polarity(HAL_PLC_SPI_MODULE, HAL_PLC_CS, HAL_PLC_POL);
	spi_set_clock_phase(HAL_PLC_SPI_MODULE, HAL_PLC_CS, HAL_PLC_PHA);
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

	/* Get board PLC PDC base address and enable receiver and transmitter. */
	g_plc_pdc = spi_get_pdc_base(HAL_PLC_SPI_MODULE);
}

/**
 * \brief Reset internal PLC Modem.
 *
 */
void hal_plc_reset(void)
{
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

	/* plc free */
	suc_plc_is_busy = false;

	/* Configure PLC reset pins */
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
 * \brief Set a signalling handler for the PLC transmission.
 */
void hal_plc_set_tx_signalling_handler(void (*p_handler)(void))
{
	plc_tx_signalling_handler = p_handler;
}

/**
 * \brief Set a signalling handler for the PLC reception.
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

	/* Waiting transfer done while(pdc_read_tx_counter(g_pdc) > 0); */
	ul_spi_busy_cnt = 0;
	while ((spi_read_status(HAL_PLC_SPI_MODULE) & (SPI_SR_TXEMPTY | SPI_SR_RXBUFF)) != (SPI_SR_TXEMPTY | SPI_SR_RXBUFF)) {
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

	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(g_plc_pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);

	/* Set 8 bits per transfer */
	spi_set_bits_per_transfer(HAL_PLC_SPI_MODULE, HAL_PLC_CS, SPI_CSR_BITS_8_BIT);

#ifdef HAL_PLC_SPI_SHARED_RF

	/* Set chip select. Done in every SPI transaction in case same SPI with
	 * another CS is used by another module */
	spi_set_peripheral_chip_select_value(HAL_PLC_SPI_MODULE, HAL_PLC_PCS);

	/* Read received data from SPI in order to clear RDRF flag (if previous
	 * transaction with RF215 was in write mode the received data is not
	 * read). Otherwise the PDC would read first the last received byte from
	 * previous transaction */
	spi_get(HAL_PLC_SPI_MODULE);
#endif

	/* Configure Tx buffer */
	puc_tx_buf = gs_plc_tx_buffer;

	memcpy(puc_tx_buf, (uint8_t *)&ul_addr, sizeof(uint32_t));
	puc_tx_buf +=  sizeof(uint32_t);
	memcpy(puc_tx_buf, (uint8_t *)&us_cmd, sizeof(uint16_t));
	puc_tx_buf +=  sizeof(uint16_t);

	memcpy(puc_tx_buf, puc_data_buf, ul_data_len);

	puc_tx_buf += ul_data_len;

	us_tx_size = puc_tx_buf - gs_plc_tx_buffer;

	/* Configure DMA channels */
	g_plc_rx_packet.ul_addr = (uint32_t)gs_plc_rx_buffer;
	g_plc_rx_packet.ul_size = us_tx_size;
	pdc_rx_init(g_plc_pdc, &g_plc_rx_packet, NULL);

	g_plc_tx_packet.ul_addr = (uint32_t)gs_plc_tx_buffer;
	g_plc_tx_packet.ul_size = us_tx_size;
	pdc_tx_init(g_plc_pdc, &g_plc_tx_packet, NULL);

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(g_plc_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

#ifdef HAL_PLC_SPI_SHARED_RF
	/* Protection: Wait until SPI is detected as busy */
	ul_spi_busy_cnt = 0;
	while ((spi_read_status(HAL_PLC_SPI_MODULE) & (SPI_SR_TXEMPTY | SPI_SR_RXBUFF)) == (SPI_SR_TXEMPTY | SPI_SR_RXBUFF)) {
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
		/* while(pdc_read_tx_counter(g_pdc) > 0); */
		ul_spi_busy_cnt = 0;
		while ((spi_read_status(HAL_PLC_SPI_MODULE) & SPI_SR_RXBUFF) == 0) {
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
	while ((spi_read_status(HAL_PLC_SPI_MODULE) & (SPI_SR_TXEMPTY | SPI_SR_RXBUFF)) != (SPI_SR_TXEMPTY | SPI_SR_RXBUFF)) {
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

	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(g_plc_pdc, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);

	/* Set 16 bits per transfer */
	spi_set_bits_per_transfer(HAL_PLC_SPI_MODULE, HAL_PLC_CS, SPI_CSR_BITS_16_BIT);

#ifdef HAL_PLC_SPI_SHARED_RF

	/* Set chip select. Done in every SPI transaction in case same SPI with
	 * another CS is used by another module */
	spi_set_peripheral_chip_select_value(HAL_PLC_SPI_MODULE, HAL_PLC_PCS);

	/* Read received data from SPI in order to clear RDRF flag (if previous
	 * transaction with RF215 was in write mode the received data is not
	 * read). Otherwise the PDC would read first the last received byte from
	 * previous transaction */
	spi_get(HAL_PLC_SPI_MODULE);
#endif

	/** Configure PLC Tx buffer **/
	puc_tx_buf = gs_plc_tx_buffer;
	/* Address */
	*puc_tx_buf++ = (uint8_t)(px_data->us_address & 0xFF);
	*puc_tx_buf++ = (uint8_t)(px_data->us_address >> 8);
	/* Length & read/write */
	us_len_wr_rd = (((px_data->us_len + 1) / 2) & HAL_PLC_LEN_MASK) | (uc_cmd << HAL_PLC_WR_RD_POS);
	*puc_tx_buf++ = (uint8_t)(us_len_wr_rd & 0xFF);
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

	/* Configure DMA channels */
	g_plc_rx_packet.ul_addr = (uint32_t)gs_plc_rx_buffer;
	g_plc_rx_packet.ul_size = us_tx_size / 2;
	pdc_rx_init(g_plc_pdc, &g_plc_rx_packet, NULL);

	g_plc_tx_packet.ul_addr = (uint32_t)gs_plc_tx_buffer;
	g_plc_tx_packet.ul_size = us_tx_size / 2;
	pdc_tx_init(g_plc_pdc, &g_plc_tx_packet, NULL);

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(g_plc_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

#ifdef HAL_PLC_SPI_SHARED_RF
	/* Protection: Wait until SPI is detected as busy */
	ul_spi_busy_cnt = 0;
	while ((spi_read_status(HAL_PLC_SPI_MODULE) & (SPI_SR_TXEMPTY | SPI_SR_RXBUFF)) == (SPI_SR_TXEMPTY | SPI_SR_RXBUFF)) {
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

	/* Waiting transfer done*/
	/* while(pdc_read_tx_counter(g_pdc) > 0); */
	ul_spi_busy_cnt = 0;
	while ((spi_read_status(HAL_PLC_SPI_MODULE) & SPI_SR_RXBUFF) == 0) {
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

	/* Enable PLC interrupt */
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
	(void)sleep;
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
		/* NTHW0 low level: High temperature (>110ºC) condition */
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
