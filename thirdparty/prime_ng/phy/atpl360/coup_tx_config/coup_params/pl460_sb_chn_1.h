/**
 * \file
 *
 * \brief PL460 Single Branch PRIME CENELEC-A (Channel 1) Coupling and TX
 * configuration.
 *
 * Copyright (C) 2020 Atmel Corporation. All rights reserved.
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

#ifndef PL460_SB_CHN_1_H_INCLUDED
#define PL460_SB_CHN_1_H_INCLUDED

/**************************************************************************************************/
/* Coupling and TX parameters for PRIME CENELEC-A (Channel 1). ************************************/
/* Recommended to use PL460 with CEN-A (Single Branch). *******************************************/
/* IMPORTANT!!! The given values were obtained from calibration with MCHP EKs. ********************/
/* For customer HW designs, calibration values should be checked with MCHP PHY Calibration Tool. **/
/**************************************************************************************************/

/* ATPL360_REG_MAX_RMS_TABLE_HI , ATPL360_REG_MAX_RMS_TABLE_VLO. */
/* Target RMS_CALC in HI / VLO mode for dynamic gain (ATPL360_REG_CFG_AUTODETECT_IMPEDANCE = 1 or 2). */
/* 8 values, corresponding to first 8 TX attenuation levels (1 dB step). */
#define MAX_RMS_HI_VALUES_CHN_1              {2581, 2301, 2053, 1831, 1633, 1456, 1297, 1157}

#define MAX_RMS_VLO_VALUES_CHN_1             {4965, 4547, 4130, 3719, 3329, 2973, 2650, 2362}

/* ATPL360_REG_THRESHOLDS_TABLE_HI, ATPL360_REG_THRESHOLDS_TABLE_VLO. */
/* Thresholds to change impedance mode (ATPL360_REG_CFG_AUTODETECT_IMPEDANCE = 1) from HI / VLO mode. */
/* HI first 8 values (1 per TX level): thresholds to change from HI to LO (0 to disable). */
/* HI next 8 values (1 per TX level): thresholds to change from HI to VLO. When RMS_CALC is below threshold, impedance mode changes to VLO (0 to disable). */
/* VLO first 8 values (1 per TX level): thresholds to change from VLO to LO (0 to disable). */
/* VLO next 8 values (1 per TX level): thresholds to change from VLO to HI. When RMS_CALC is above threshold, impedance mode changes to HI (>=100000 to disable). */
#define TH_HI_VALUES_CHN_1                   {0, 0, 0, 0, 0, 0, 0, 0, 2115, 1886, 1683, 1502, 1339, 1194, 1064, 948}

#define TH_VLO_VALUES_CHN_1                  {0, 0, 0, 0, 0, 0, 0, 0, 8010, 7179, 6390, 5723, 5105, 4547, 4061, 3615}

/* ATPL360_REG_PREDIST_COEF_TABLE_HI, ATPL360_REG_PREDIST_COEF_TABLE_VLO. Equalization values for HI / VLO mode. */
/* Specific gain for each carrier to equalize transmission and compensate HW filter frequency response. */
#define PREDIST_COEF_HI_VALUES_CHN_1         {0x756E, 0x7396, 0x730A, 0x72EB, 0x72B2, 0x7433, 0x755E, 0x75D7, 0x769E, 0x76A4, 0x77C3, 0x7851, 0x7864, 0x78A0, \
					      0x78BA, 0x7918, 0x79B6, 0x79E9, 0x7ACC, 0x7B06, 0x7B30, 0x7B27, 0x7C1E, 0x7B96, 0x7A76, 0x7B12, 0x7AFD, 0x7C40, \
					      0x7C5E, 0x7B48, 0x7B8A, 0x7C64, 0x7C42, 0x7BCD, 0x7AFD, 0x7A5F, 0x7A03, 0x7A9D, 0x7A1A, 0x7A4A, 0x79FC, 0x7984, \
					      0x7A0D, 0x79CC, 0x792E, 0x780D, 0x7676, 0x75E4, 0x747A, 0x7251, 0x707E, 0x6E96, 0x6E30, 0x6D44, 0x6DBD, 0x6C9A, \
					      0x6C3C, 0x6CF8, 0x6CA4, 0x6CDF, 0x6C59, 0x6B2C, 0x6CB9, 0x6C1F, 0x6B6D, 0x6BF5, 0x6AF0, 0x6A55, 0x6955, 0x674F, \
					      0x6841, 0x685D, 0x670F, 0x6904, 0x6967, 0x6B01, 0x6C31, 0x6C2A, 0x6D82, 0x6F58, 0x6E62, 0x6F18, 0x6EE7, 0x7069, \
					      0x717B, 0x7120, 0x7170, 0x72FB, 0x7491, 0x75B3, 0x75A2, 0x7664, 0x784A, 0x7A52, 0x7B51, 0x7D5A, 0x7FFF}

#define PREDIST_COEF_VLO_VALUES_CHN_1        {0x7FFF, 0x7F2B, 0x7E38, 0x7CD3, 0x7B38, 0x7972, 0x77D6, 0x7654, 0x74AE, 0x7288, 0x70C0, 0x6E9A, 0x6D24, 0x6B80, \
					      0x6A2F, 0x6852, 0x674E, 0x65DA, 0x652E, 0x637E, 0x6292, 0x6142, 0x60CC, 0x5FF8, 0x5F6D, 0x5EC2, 0x5E6F, 0x5E55, \
					      0x5E43, 0x5E02, 0x5E5B, 0x5EB3, 0x5F4A, 0x5FD7, 0x604C, 0x60FC, 0x61F3, 0x6297, 0x63A9, 0x643D, 0x654A, 0x6634, \
					      0x675C, 0x6824, 0x6910, 0x69A4, 0x6A73, 0x6B6F, 0x6C15, 0x6CCD, 0x6D64, 0x6E4B, 0x6ED3, 0x6F44, 0x6F85, 0x70A1, \
					      0x70AF, 0x71B2, 0x7149, 0x71F3, 0x7203, 0x7279, 0x71FB, 0x72B4, 0x7281, 0x72A4, 0x7262, 0x72BD, 0x7295, 0x72CC, \
					      0x729E, 0x7288, 0x7244, 0x7279, 0x726C, 0x7230, 0x71B9, 0x70D8, 0x7045, 0x7052, 0x6F8D, 0x6F3D, 0x6EB0, 0x6E6A, \
					      0x6E76, 0x6E1C, 0x6D7A, 0x6D84, 0x6D50, 0x6D45, 0x6CF2, 0x6CA9, 0x6C92, 0x6CBA, 0x6C69, 0x6C27, 0x6C02}

/* ATPL360_REG_GAIN_TABLE_HI, ATPL360_REG_GAIN_TABLE_VLO. Gain values for HI / VLO mode {GAIN_INI, GAIN_MIN, GAIN_MAX}. */
#define IFFT_GAIN_HI_INI_CHN_1               81
#define IFFT_GAIN_HI_MIN_CHN_1               40
#define IFFT_GAIN_HI_MAX_CHN_1               128

#define IFFT_GAIN_VLO_INI_CHN_1              256
#define IFFT_GAIN_VLO_MIN_CHN_1              128
#define IFFT_GAIN_VLO_MAX_CHN_1              281

/* ATPL360_REG_DACC_TABLE_CFG: Configuration for PRIME Channel 1 with PL460. */
#define DACC_CFG_TABLE_CHN_1                 {0x00000000, 0x00000000, 0x00000100, 0x00000100, 0x00000000, 0x00000000, 0x9F7800FF, 0x1A1A1A1A, \
					      0x00000000, 0x00000000, 0x00000005, 0x00000355, 0x00000000, 0x001020F0, 0x00000355, 0x00000000, 0x001020FF}

/* ATPL360_REG_NUM_TX_LEVELS: Default value 8 */
#define NUM_TX_LEVELS_CHN_1                  8

/* ATPL360_REG_PLC_IC_DRIVER_CFG: PLC IC Driver used (PL460 Single Branch) */
#define PLC_IC_DRV_CFG_CHN_1                 0x05

#endif /* PL460_SB_CHN_1_H_INCLUDED */
