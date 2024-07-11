/**
 * \file
 *
 * \brief Couping and TX configuration for PL360 (PRIME).
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

/* Coupling configuration includes */
#include "coup_tx_config.h"
#include "coup_conf.h"

/**********************************************************************************************************************************/
/* Coupling and TX parameter configuration. Each PRIME channel needs its own calibration. *****************************************/
/* PL360 PRIME PHY initialized in Channel 1 (CENELEC-A). Coupling and TX parameters must be reconfigured when channel is changed. */
/* IMPORTANT!!! The given values were obtained from calibration with MCHP EKs. ****************************************************/
/* For customer HW designs, calibration values should be checked with MCHP PHY Calibration Tool. **********************************/
/**********************************************************************************************************************************/

/* Maximum value of ATPL360_REG_NUM_TX_LEVELS */
/* Number of TX attenuation levels (1 dB step) suppoting automatic TX mode */
#define MAX_NUM_TX_LEVELS                8

/* Struct definition of Coupling and TX parameters (only include parameters that change for all channels). */
typedef struct coup_tx_params {
	uint32_t pul_rms_hi[MAX_NUM_TX_LEVELS];
	uint32_t pul_rms_vlo[MAX_NUM_TX_LEVELS];
	uint32_t pul_th_hi[MAX_NUM_TX_LEVELS << 1];
	uint32_t pul_th_vlo[MAX_NUM_TX_LEVELS << 1];
	uint16_t pus_equ_hi[97];
	uint16_t pus_equ_vlo[97];
	uint16_t pus_gain_hi[3];
	uint16_t pus_gain_vlo[3];
} coup_tx_params_t;

typedef struct coup_tx_params_2chn {
	uint32_t pul_rms_hi[MAX_NUM_TX_LEVELS];
	uint32_t pul_rms_vlo[MAX_NUM_TX_LEVELS];
	uint32_t pul_th_hi[MAX_NUM_TX_LEVELS << 1];
	uint32_t pul_th_vlo[MAX_NUM_TX_LEVELS << 1];
	uint16_t pus_equ_hi[97 << 1];
	uint16_t pus_equ_vlo[97 << 1];
	uint16_t pus_gain_hi[3];
	uint16_t pus_gain_vlo[3];
} coup_tx_params_2chn_t;

#ifdef COUP_CONF_CHN_1_ENABLE
/* Channel 1 (CENELEC-A) Coupling and TX parameter configuration */

static const coup_tx_params_t spx_coup_params_chn_1 = {
	MAX_RMS_HI_VALUES_CHN_1, MAX_RMS_VLO_VALUES_CHN_1,
	TH_HI_VALUES_CHN_1, TH_VLO_VALUES_CHN_1,
	PREDIST_COEF_HI_VALUES_CHN_1, PREDIST_COEF_VLO_VALUES_CHN_1,
	{IFFT_GAIN_HI_INI_CHN_1, IFFT_GAIN_HI_MIN_CHN_1, IFFT_GAIN_HI_MAX_CHN_1},
	{IFFT_GAIN_VLO_INI_CHN_1, IFFT_GAIN_VLO_MIN_CHN_1, IFFT_GAIN_VLO_MAX_CHN_1}
};

static const uint32_t spul_dacc_cfg_chn_1[17] = DACC_CFG_TABLE_CHN_1;

#endif

#ifdef COUP_CONF_CHN_FCC_ENABLE
/* Channel 2 - 8 (FCC / CENELEC-BCD) Coupling and TX parameter configuration */
/* Calibration values different for each channel */

static const coup_tx_params_t spx_coup_params_chn_fcc[7] = {
	{MAX_RMS_HI_VALUES_CHN_2, MAX_RMS_VLO_VALUES_CHN_2,
	 TH_HI_VALUES_CHN_2, TH_VLO_VALUES_CHN_2,
	 PREDIST_COEF_HI_VALUES_CHN_2, PREDIST_COEF_VLO_VALUES_CHN_2,
	 {IFFT_GAIN_HI_INI_CHN_2, IFFT_GAIN_HI_MIN_CHN_2, IFFT_GAIN_HI_MAX_CHN_2},
	 {IFFT_GAIN_VLO_INI_CHN_2, IFFT_GAIN_VLO_MIN_CHN_2, IFFT_GAIN_VLO_MAX_CHN_2}},
	{MAX_RMS_HI_VALUES_CHN_3, MAX_RMS_VLO_VALUES_CHN_3,
	 TH_HI_VALUES_CHN_3, TH_VLO_VALUES_CHN_3,
	 PREDIST_COEF_HI_VALUES_CHN_3, PREDIST_COEF_VLO_VALUES_CHN_3,
	 {IFFT_GAIN_HI_INI_CHN_3, IFFT_GAIN_HI_MIN_CHN_3, IFFT_GAIN_HI_MAX_CHN_3},
	 {IFFT_GAIN_VLO_INI_CHN_3, IFFT_GAIN_VLO_MIN_CHN_3, IFFT_GAIN_VLO_MAX_CHN_3}},
	{MAX_RMS_HI_VALUES_CHN_4, MAX_RMS_VLO_VALUES_CHN_4,
	 TH_HI_VALUES_CHN_4, TH_VLO_VALUES_CHN_4,
	 PREDIST_COEF_HI_VALUES_CHN_4, PREDIST_COEF_VLO_VALUES_CHN_4,
	 {IFFT_GAIN_HI_INI_CHN_4, IFFT_GAIN_HI_MIN_CHN_4, IFFT_GAIN_HI_MAX_CHN_4},
	 {IFFT_GAIN_VLO_INI_CHN_4, IFFT_GAIN_VLO_MIN_CHN_4, IFFT_GAIN_VLO_MAX_CHN_4}},
	{MAX_RMS_HI_VALUES_CHN_5, MAX_RMS_VLO_VALUES_CHN_5,
	 TH_HI_VALUES_CHN_5, TH_VLO_VALUES_CHN_5,
	 PREDIST_COEF_HI_VALUES_CHN_5, PREDIST_COEF_VLO_VALUES_CHN_5,
	 {IFFT_GAIN_HI_INI_CHN_5, IFFT_GAIN_HI_MIN_CHN_5, IFFT_GAIN_HI_MAX_CHN_5},
	 {IFFT_GAIN_VLO_INI_CHN_5, IFFT_GAIN_VLO_MIN_CHN_5, IFFT_GAIN_VLO_MAX_CHN_5}},
	{MAX_RMS_HI_VALUES_CHN_6, MAX_RMS_VLO_VALUES_CHN_6,
	 TH_HI_VALUES_CHN_6, TH_VLO_VALUES_CHN_6,
	 PREDIST_COEF_HI_VALUES_CHN_6, PREDIST_COEF_VLO_VALUES_CHN_6,
	 {IFFT_GAIN_HI_INI_CHN_6, IFFT_GAIN_HI_MIN_CHN_6, IFFT_GAIN_HI_MAX_CHN_6},
	 {IFFT_GAIN_VLO_INI_CHN_6, IFFT_GAIN_VLO_MIN_CHN_6, IFFT_GAIN_VLO_MAX_CHN_6}},
	{MAX_RMS_HI_VALUES_CHN_7, MAX_RMS_VLO_VALUES_CHN_7,
	 TH_HI_VALUES_CHN_7, TH_VLO_VALUES_CHN_7,
	 PREDIST_COEF_HI_VALUES_CHN_7, PREDIST_COEF_VLO_VALUES_CHN_7,
	 {IFFT_GAIN_HI_INI_CHN_7, IFFT_GAIN_HI_MIN_CHN_7, IFFT_GAIN_HI_MAX_CHN_7},
	 {IFFT_GAIN_VLO_INI_CHN_7, IFFT_GAIN_VLO_MIN_CHN_7, IFFT_GAIN_VLO_MAX_CHN_7}},
	{MAX_RMS_HI_VALUES_CHN_8, MAX_RMS_VLO_VALUES_CHN_8,
	 TH_HI_VALUES_CHN_8, TH_VLO_VALUES_CHN_8,
	 PREDIST_COEF_HI_VALUES_CHN_8, PREDIST_COEF_VLO_VALUES_CHN_8,
	 {IFFT_GAIN_HI_INI_CHN_8, IFFT_GAIN_HI_MIN_CHN_8, IFFT_GAIN_HI_MAX_CHN_8},
	 {IFFT_GAIN_VLO_INI_CHN_8, IFFT_GAIN_VLO_MIN_CHN_8, IFFT_GAIN_VLO_MAX_CHN_8}}
};

static const uint32_t spul_dacc_cfg_chn_fcc[17] = DACC_CFG_TABLE_CHN_FCC;

#endif

#ifdef COUP_CONF_2CHN_ENABLE
/* PRIME Double Channel Coupling and TX parameter configuration */
/* Calibration values different for each double channel */

static const coup_tx_params_2chn_t spx_coup_params_chn_2chn[7] = {
	{MAX_RMS_HI_VALUES_2CHN_1_2, MAX_RMS_VLO_VALUES_2CHN_1_2,
	 TH_HI_VALUES_2CHN_1_2, TH_VLO_VALUES_2CHN_1_2,
	 PREDIST_COEF_HI_VALUES_2CHN_1_2, PREDIST_COEF_VLO_VALUES_2CHN_1_2,
	 {IFFT_GAIN_HI_INI_2CHN_1_2, IFFT_GAIN_HI_MIN_2CHN_1_2, IFFT_GAIN_HI_MAX_2CHN_1_2},
	 {IFFT_GAIN_VLO_INI_2CHN_1_2, IFFT_GAIN_VLO_MIN_2CHN_1_2, IFFT_GAIN_VLO_MAX_2CHN_1_2}},
	{MAX_RMS_HI_VALUES_2CHN_2_3, MAX_RMS_VLO_VALUES_2CHN_2_3,
	 TH_HI_VALUES_2CHN_2_3, TH_VLO_VALUES_2CHN_2_3,
	 PREDIST_COEF_HI_VALUES_2CHN_2_3, PREDIST_COEF_VLO_VALUES_2CHN_2_3,
	 {IFFT_GAIN_HI_INI_2CHN_2_3, IFFT_GAIN_HI_MIN_2CHN_2_3, IFFT_GAIN_HI_MAX_2CHN_2_3},
	 {IFFT_GAIN_VLO_INI_2CHN_2_3, IFFT_GAIN_VLO_MIN_2CHN_2_3, IFFT_GAIN_VLO_MAX_2CHN_2_3}},
	{MAX_RMS_HI_VALUES_2CHN_3_4, MAX_RMS_VLO_VALUES_2CHN_3_4,
	 TH_HI_VALUES_2CHN_3_4, TH_VLO_VALUES_2CHN_3_4,
	 PREDIST_COEF_HI_VALUES_2CHN_3_4, PREDIST_COEF_VLO_VALUES_2CHN_3_4,
	 {IFFT_GAIN_HI_INI_2CHN_3_4, IFFT_GAIN_HI_MIN_2CHN_3_4, IFFT_GAIN_HI_MAX_2CHN_3_4},
	 {IFFT_GAIN_VLO_INI_2CHN_3_4, IFFT_GAIN_VLO_MIN_2CHN_3_4, IFFT_GAIN_VLO_MAX_2CHN_3_4}},
	{MAX_RMS_HI_VALUES_2CHN_4_5, MAX_RMS_VLO_VALUES_2CHN_4_5,
	 TH_HI_VALUES_2CHN_4_5, TH_VLO_VALUES_2CHN_4_5,
	 PREDIST_COEF_HI_VALUES_2CHN_4_5, PREDIST_COEF_VLO_VALUES_2CHN_4_5,
	 {IFFT_GAIN_HI_INI_2CHN_4_5, IFFT_GAIN_HI_MIN_2CHN_4_5, IFFT_GAIN_HI_MAX_2CHN_4_5},
	 {IFFT_GAIN_VLO_INI_2CHN_4_5, IFFT_GAIN_VLO_MIN_2CHN_4_5, IFFT_GAIN_VLO_MAX_2CHN_4_5}},
	{MAX_RMS_HI_VALUES_2CHN_5_6, MAX_RMS_VLO_VALUES_2CHN_5_6,
	 TH_HI_VALUES_2CHN_5_6, TH_VLO_VALUES_2CHN_5_6,
	 PREDIST_COEF_HI_VALUES_2CHN_5_6, PREDIST_COEF_VLO_VALUES_2CHN_5_6,
	 {IFFT_GAIN_HI_INI_2CHN_5_6, IFFT_GAIN_HI_MIN_2CHN_5_6, IFFT_GAIN_HI_MAX_2CHN_5_6},
	 {IFFT_GAIN_VLO_INI_2CHN_5_6, IFFT_GAIN_VLO_MIN_2CHN_5_6, IFFT_GAIN_VLO_MAX_2CHN_5_6}},
	{MAX_RMS_HI_VALUES_2CHN_6_7, MAX_RMS_VLO_VALUES_2CHN_6_7,
	 TH_HI_VALUES_2CHN_6_7, TH_VLO_VALUES_2CHN_6_7,
	 PREDIST_COEF_HI_VALUES_2CHN_6_7, PREDIST_COEF_VLO_VALUES_2CHN_6_7,
	 {IFFT_GAIN_HI_INI_2CHN_6_7, IFFT_GAIN_HI_MIN_2CHN_6_7, IFFT_GAIN_HI_MAX_2CHN_6_7},
	 {IFFT_GAIN_VLO_INI_2CHN_6_7, IFFT_GAIN_VLO_MIN_2CHN_6_7, IFFT_GAIN_VLO_MAX_2CHN_6_7}},
	{MAX_RMS_HI_VALUES_2CHN_7_8, MAX_RMS_VLO_VALUES_2CHN_7_8,
	 TH_HI_VALUES_2CHN_7_8, TH_VLO_VALUES_2CHN_7_8,
	 PREDIST_COEF_HI_VALUES_2CHN_7_8, PREDIST_COEF_VLO_VALUES_2CHN_7_8,
	 {IFFT_GAIN_HI_INI_2CHN_7_8, IFFT_GAIN_HI_MIN_2CHN_7_8, IFFT_GAIN_HI_MAX_2CHN_7_8},
	 {IFFT_GAIN_VLO_INI_2CHN_7_8, IFFT_GAIN_VLO_MIN_2CHN_7_8, IFFT_GAIN_VLO_MAX_2CHN_7_8}}
};

static const uint32_t spul_dacc_cfg_2chn[17] = DACC_CFG_TABLE_2CHN;

#endif

/* Default number of TX attenuation levels (1 dB step) suppoting automatic TX mode */
#ifndef NUM_TX_LEVELS_CHN_1
# define NUM_TX_LEVELS_CHN_1                      8
#endif
#ifndef NUM_TX_LEVELS_CHN_FCC
# define NUM_TX_LEVELS_CHN_FCC                    8
#endif
#ifndef NUM_TX_LEVELS_2CHN
# define NUM_TX_LEVELS_2CHN                       8
#endif

/**
 * \brief Configure Coupling and TX parameters for each channel (Single Channel)
 *
 * \param px_atpl360_desc Pointer to PL360 controller despriptor
 * \param uc_chn PRIME channel (Single Channel). 1 to 8
 */
static inline void _configure_tx_coup_params_1chn(atpl360_descriptor_t *px_atpl360_desc, uint8_t uc_chn)
{
#if ((!defined(COUP_CONF_CHN_1_ENABLE)) && (!defined(COUP_CONF_CHN_FCC_ENABLE)))
	UNUSED(px_atpl360_desc);
	UNUSED(uc_chn);
#else
	coup_tx_params_t *px_coup_params;
	uint32_t *pul_dacc_table;
	uint8_t uc_num_tx_levels;
	uint8_t uc_plc_drv_cfg;
	bool b_write_config = false;

	if (uc_chn == 1) {
# ifdef COUP_CONF_CHN_1_ENABLE
		/* CENELEC-A (Channel 1) */
		px_coup_params = (coup_tx_params_t *)&spx_coup_params_chn_1;
		pul_dacc_table = (uint32_t *)spul_dacc_cfg_chn_1;
		uc_num_tx_levels = NUM_TX_LEVELS_CHN_1;
		uc_plc_drv_cfg = PLC_IC_DRV_CFG_CHN_1;

		b_write_config = true;
# endif
	} else {
# ifdef COUP_CONF_CHN_FCC_ENABLE
		/* FCC (Channel 3-8) / CENELEC-BCD (Channel 2) */
		px_coup_params = (coup_tx_params_t *)&spx_coup_params_chn_fcc[uc_chn - 2];
		pul_dacc_table = (uint32_t *)spul_dacc_cfg_chn_fcc;
		uc_num_tx_levels = NUM_TX_LEVELS_CHN_FCC;
		uc_plc_drv_cfg = PLC_IC_DRV_CFG_CHN_FCC;

		b_write_config = true;
# endif
	}

	if (b_write_config) {
		/* Write PLC_IC_DRV_CFG. 1 byte */
		px_atpl360_desc->set_config(ATPL360_REG_PLC_IC_DRIVER_CFG, &uc_plc_drv_cfg, 1);

		/* Write DACC_TABLE values. 68 bytes */
		px_atpl360_desc->set_config(ATPL360_REG_DACC_TABLE_CFG, pul_dacc_table, 17 << 2);

		/* Write NUM_TX_LEVELS. 1 byte */
		px_atpl360_desc->set_config(ATPL360_REG_NUM_TX_LEVELS, &uc_num_tx_levels, 1);

		/* Write MAX_RMS_TABLE values. 32 x 2 bytes (HI and VLO) */
		px_atpl360_desc->set_config(ATPL360_REG_MAX_RMS_TABLE_HI, px_coup_params->pul_rms_hi, MAX_NUM_TX_LEVELS << 2);
		px_atpl360_desc->set_config(ATPL360_REG_MAX_RMS_TABLE_VLO, px_coup_params->pul_rms_vlo, MAX_NUM_TX_LEVELS << 2);

		/* Write THRESHOLDS_TABLE values. 64 x 2 bytes (HI and VLO) */
		px_atpl360_desc->set_config(ATPL360_REG_THRESHOLDS_TABLE_HI, px_coup_params->pul_th_hi, MAX_NUM_TX_LEVELS << 3);
		px_atpl360_desc->set_config(ATPL360_REG_THRESHOLDS_TABLE_VLO, px_coup_params->pul_th_vlo, MAX_NUM_TX_LEVELS << 3);

		/* Write GAIN_TABLE values. 6 x 2 bytes (HI and VLO) */
		px_atpl360_desc->set_config(ATPL360_REG_GAIN_TABLE_HI, px_coup_params->pus_gain_hi, 6);
		px_atpl360_desc->set_config(ATPL360_REG_GAIN_TABLE_VLO, px_coup_params->pus_gain_vlo, 6);

		/* Write Equalization values. 194 x 2 bytes (HI and VLO) */
		px_atpl360_desc->set_config(ATPL360_REG_PREDIST_COEF_TABLE_HI, px_coup_params->pus_equ_hi, 97 << 1);
		px_atpl360_desc->set_config(ATPL360_REG_PREDIST_COEF_TABLE_VLO, px_coup_params->pus_equ_vlo, 97 << 1);
	}
#endif
}

/**
 * \brief Configure Coupling and TX parameters for each channel (Double Channel)
 *
 * \param px_atpl360_desc Pointer to PL360 controller despriptor
 * \param uc_chn PRIME channel (Double Channel). 1 to 7
 */
static inline void _configure_tx_coup_params_2chn(atpl360_descriptor_t *px_atpl360_desc, uint8_t uc_chn)
{
#ifndef COUP_CONF_2CHN_ENABLE
	UNUSED(px_atpl360_desc);
	UNUSED(uc_chn);
#else
	coup_tx_params_2chn_t *px_coup_params;
	uint8_t uc_num_tx_levels;
	uint8_t uc_plc_drv_cfg;

	/* Write PLC_IC_DRV_CFG. 1 byte */
	uc_plc_drv_cfg = PLC_IC_DRV_CFG_2CHN;
	px_atpl360_desc->set_config(ATPL360_REG_PLC_IC_DRIVER_CFG, &uc_plc_drv_cfg, 1);

	/* Write DACC_TABLE values. 68 bytes */
	px_atpl360_desc->set_config(ATPL360_REG_DACC_TABLE_CFG, (uint32_t *)spul_dacc_cfg_2chn, 17 << 2);

	/* Write NUM_TX_LEVELS. 1 byte */
	uc_num_tx_levels = NUM_TX_LEVELS_2CHN;
	px_atpl360_desc->set_config(ATPL360_REG_NUM_TX_LEVELS, &uc_num_tx_levels, 1);

	/* Write MAX_RMS_TABLE values. 32 x 2 bytes (HI and VLO) */
	px_coup_params = (coup_tx_params_2chn_t *)&spx_coup_params_chn_2chn[uc_chn - 1];
	px_atpl360_desc->set_config(ATPL360_REG_MAX_RMS_TABLE_HI, px_coup_params->pul_rms_hi, MAX_NUM_TX_LEVELS << 2);
	px_atpl360_desc->set_config(ATPL360_REG_MAX_RMS_TABLE_VLO, px_coup_params->pul_rms_vlo, MAX_NUM_TX_LEVELS << 2);

	/* Write THRESHOLDS_TABLE values. 64 x 2 bytes (HI and VLO) */
	px_atpl360_desc->set_config(ATPL360_REG_THRESHOLDS_TABLE_HI, px_coup_params->pul_th_hi, MAX_NUM_TX_LEVELS << 3);
	px_atpl360_desc->set_config(ATPL360_REG_THRESHOLDS_TABLE_VLO, px_coup_params->pul_th_vlo, MAX_NUM_TX_LEVELS << 3);

	/* Write GAIN_TABLE values. 6 x 2 bytes (HI and VLO) */
	px_atpl360_desc->set_config(ATPL360_REG_GAIN_TABLE_HI, px_coup_params->pus_gain_hi, 6);
	px_atpl360_desc->set_config(ATPL360_REG_GAIN_TABLE_VLO, px_coup_params->pus_gain_vlo, 6);

	/* Write Equalization values. 194 x 2 x 2 bytes (HI and VLO) */
	px_atpl360_desc->set_config(ATPL360_REG_PREDIST_COEF_TABLE_HI, px_coup_params->pus_equ_hi, 97 << 1);
	px_atpl360_desc->set_config(ATPL360_REG_PREDIST_COEF_TABLE_HI_2, px_coup_params->pus_equ_hi + 97, 97 << 1);
	px_atpl360_desc->set_config(ATPL360_REG_PREDIST_COEF_TABLE_VLO, px_coup_params->pus_equ_vlo, 97 << 1);
	px_atpl360_desc->set_config(ATPL360_REG_PREDIST_COEF_TABLE_VLO_2, px_coup_params->pus_equ_vlo + 97, 97 << 1);
#endif
}

/**
 * \brief Configure Coupling and TX parameters for PRIME
 *
 * \param px_atpl360_desc Pointer to PL360 controller despriptor
 * \param uc_channel PRIME channel. 1 to 8 (Single Channel) or 9 to 15 (Double Channel)
 */
void pl360_prime_coup_tx_config(atpl360_descriptor_t *px_atpl360_desc, uint8_t uc_channel)
{
	/* Single and Double Channel supported */
	if ((uc_channel >= SINGLE_CHN_1) && (uc_channel <= SINGLE_CHN_8)) {
		_configure_tx_coup_params_1chn(px_atpl360_desc, uc_channel);
	} else if ((uc_channel >= DOUBLE_CHN_1_2) && (uc_channel <= DOUBLE_CHN_7_8)) {
		_configure_tx_coup_params_2chn(px_atpl360_desc, uc_channel - SINGLE_CHN_8);
	}
}
