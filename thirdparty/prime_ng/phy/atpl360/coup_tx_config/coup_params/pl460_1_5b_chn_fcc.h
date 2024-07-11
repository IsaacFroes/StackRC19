/**
 * \file
 *
 * \brief PL460 1.5 Branch PRIME FCC (Channel 3-8) / CENELEC-BCD (Channel 2)
 * Coupling and TX configuration.
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

#ifndef PL460_1_5B_CHN_FCC_H_INCLUDED
#define PL460_1_5B_CHN_FCC_H_INCLUDED

/**********************************************************************************************************/
/* Coupling and TX parameters for PRIME FCC (Channel 3 - 8) and CENELEC-BCD (Channel 2). ******************/
/* Each channel needs its own calibration *****************************************************************/
/* Recommended to use PL460 with FCC-1.5B (1.5 Branch). ***************************************************/
/* PL460 with FCC-1.5B designed for FCC band. Channel 2 (CENELEC-BCD) supported to support all channels. **/
/* IMPORTANT!!! The given values were obtained from calibration with MCHP EKs. ****************************/
/* For customer HW designs, calibration values should be checked with MCHP PHY Calibration Tool. **********/
/**********************************************************************************************************/

/* ATPL360_REG_MAX_RMS_TABLE_HI , ATPL360_REG_MAX_RMS_TABLE_VLO. */
/* Target RMS_CALC in HI / VLO mode for dynamic gain (ATPL360_REG_CFG_AUTODETECT_IMPEDANCE = 1 or 2). */
/* 8 values, corresponding to first 8 TX attenuation levels (1 dB step). */
#define MAX_RMS_HI_VALUES_CHN_2              {737, 654, 581, 517, 460, 409, 364, 323}
#define MAX_RMS_HI_VALUES_CHN_3              {800, 697, 608, 532, 466, 409, 359, 315}
#define MAX_RMS_HI_VALUES_CHN_4              {1311, 1171, 1047, 936, 837, 748, 669, 597}
#define MAX_RMS_HI_VALUES_CHN_5              {1831, 1629, 1453, 1297, 1159, 1035, 925, 827}
#define MAX_RMS_HI_VALUES_CHN_6              {1707, 1520, 1360, 1219, 1096, 987, 890, 803}
#define MAX_RMS_HI_VALUES_CHN_7              {1248, 1125, 1016, 917, 830, 752, 682, 618}
#define MAX_RMS_HI_VALUES_CHN_8              {953, 868, 784, 706, 635, 573, 516, 466}

#define MAX_RMS_VLO_VALUES_CHN_2             {3492, 3124, 2766, 2430, 2122, 1853, 1622, 1425}
#define MAX_RMS_VLO_VALUES_CHN_3             {2459, 2265, 2069, 1868, 1676, 1495, 1330, 1183}
#define MAX_RMS_VLO_VALUES_CHN_4             {3124, 2868, 2609, 2358, 2114, 1882, 1673, 1486}
#define MAX_RMS_VLO_VALUES_CHN_5             {3344, 3048, 2749, 2456, 2188, 1948, 1734, 1543}
#define MAX_RMS_VLO_VALUES_CHN_6             {3074, 2814, 2546, 2283, 2042, 1825, 1632, 1459}
#define MAX_RMS_VLO_VALUES_CHN_7             {2553, 2339, 2129, 1924, 1723, 1542, 1379, 1233}
#define MAX_RMS_VLO_VALUES_CHN_8             {1838, 1668, 1500, 1345, 1203, 1076, 963, 861}

/* ATPL360_REG_THRESHOLDS_TABLE_HI, ATPL360_REG_THRESHOLDS_TABLE_VLO. */
/* Thresholds to change impedance mode (ATPL360_REG_CFG_AUTODETECT_IMPEDANCE = 1) from HI / VLO mode. */
/* HI first 8 values (1 per TX level): thresholds to change from HI to LO (0 to disable). */
/* HI next 8 values (1 per TX level): thresholds to change from HI to VLO. When RMS_CALC is below threshold, impedance mode changes to VLO (0 to disable). */
/* VLO first 8 values (1 per TX level): thresholds to change from VLO to LO (0 to disable). */
/* VLO next 8 values (1 per TX level): thresholds to change from VLO to HI. When RMS_CALC is above threshold, impedance mode changes to HI (>=100000 to disable). */
#define TH_HI_VALUES_CHN_2                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define TH_HI_VALUES_CHN_3                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define TH_HI_VALUES_CHN_4                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define TH_HI_VALUES_CHN_5                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define TH_HI_VALUES_CHN_6                   {0, 0, 0, 0, 0, 0, 0, 0, 1485, 1321, 1181, 1060, 951, 857, 774, 697}
#define TH_HI_VALUES_CHN_7                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define TH_HI_VALUES_CHN_8                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

#define TH_VLO_VALUES_CHN_2                  {0, 0, 0, 0, 0, 0, 0, 0, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000}
#define TH_VLO_VALUES_CHN_3                  {0, 0, 0, 0, 0, 0, 0, 0, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000}
#define TH_VLO_VALUES_CHN_4                  {0, 0, 0, 0, 0, 0, 0, 0, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000}
#define TH_VLO_VALUES_CHN_5                  {0, 0, 0, 0, 0, 0, 0, 0, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000}
#define TH_VLO_VALUES_CHN_6                  {0, 0, 0, 0, 0, 0, 0, 0, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000}
#define TH_VLO_VALUES_CHN_7                  {0, 0, 0, 0, 0, 0, 0, 0, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000}
#define TH_VLO_VALUES_CHN_8                  {0, 0, 0, 0, 0, 0, 0, 0, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000}

//#define TH_HI_VALUES_CHN_2                   {0, 0, 0, 0, 0, 0, 0, 0, 668, 595, 529, 469, 415, 370, 329, 292}
//#define TH_HI_VALUES_CHN_3                   {0, 0, 0, 0, 0, 0, 0, 0, 722, 630, 549, 480, 421, 369, 324, 284}
//#define TH_HI_VALUES_CHN_4                   {0, 0, 0, 0, 0, 0, 0, 0, 1045, 934, 836, 747, 668, 597, 533, 476}
//#define TH_HI_VALUES_CHN_5                   {0, 0, 0, 0, 0, 0, 0, 0, 1647, 1467, 1307, 1167, 1043, 931, 833, 744}
//#define TH_HI_VALUES_CHN_6                   {0, 0, 0, 0, 0, 0, 0, 0, 1503, 1346, 1209, 1087, 979, 883, 798, 720}
//#define TH_HI_VALUES_CHN_7                   {0, 0, 0, 0, 0, 0, 0, 0, 1122, 1010, 910, 823, 745, 674, 611, 554}
//#define TH_HI_VALUES_CHN_8                   {0, 0, 0, 0, 0, 0, 0, 0, 858, 781, 705, 635, 571, 514, 464, 419}

//#define TH_VLO_VALUES_CHN_2                  {0, 0, 0, 0, 0, 0, 0, 0, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000}
//#define TH_VLO_VALUES_CHN_3                  {0, 0, 0, 0, 0, 0, 0, 0, 10915, 9772, 8738, 7814, 6985, 6241, 5577, 4987}
//#define TH_VLO_VALUES_CHN_4                  {0, 0, 0, 0, 0, 0, 0, 0, 7256, 6510, 5840, 5242, 4706, 4226, 3796, 3413}
//#define TH_VLO_VALUES_CHN_5                  {0, 0, 0, 0, 0, 0, 0, 0, 4772, 4264, 3807, 3399, 3034, 2707, 2416, 2155}
//#define TH_VLO_VALUES_CHN_6                  {0, 0, 0, 0, 0, 0, 0, 0, 5790, 5300, 4800, 4318, 3847, 3420, 3038, 2700}
//#define TH_VLO_VALUES_CHN_7                  {0, 0, 0, 0, 0, 0, 0, 0, 8820, 7785, 6874, 6074, 5369, 4750, 4203, 3722}
//#define TH_VLO_VALUES_CHN_8                  {0, 0, 0, 0, 0, 0, 0, 0, 9963, 8968, 8057, 7239, 6499, 5835, 5234, 4693}

/* ATPL360_REG_PREDIST_COEF_TABLE_HI, ATPL360_REG_PREDIST_COEF_TABLE_VLO. Equalization values for HI / VLO mode. */
/* Specific gain for each carrier to equalize transmission and compensate HW filter frequency response. */
#define PREDIST_COEF_HI_VALUES_CHN_2         {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF}

#define PREDIST_COEF_VLO_VALUES_CHN_2        {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF}

#define PREDIST_COEF_HI_VALUES_CHN_3         {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF}

#define PREDIST_COEF_VLO_VALUES_CHN_3        {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF}

#define PREDIST_COEF_HI_VALUES_CHN_4         {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF}

#define PREDIST_COEF_VLO_VALUES_CHN_4        {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF}

#define PREDIST_COEF_HI_VALUES_CHN_5         {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF}

#define PREDIST_COEF_VLO_VALUES_CHN_5        {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF}

#define PREDIST_COEF_HI_VALUES_CHN_6         {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF}

#define PREDIST_COEF_VLO_VALUES_CHN_6        {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF}

#define PREDIST_COEF_HI_VALUES_CHN_7         {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF}

#define PREDIST_COEF_VLO_VALUES_CHN_7        {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF}

#define PREDIST_COEF_HI_VALUES_CHN_8         {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF}

#define PREDIST_COEF_VLO_VALUES_CHN_8        {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF}

/* ATPL360_REG_GAIN_TABLE_HI, ATPL360_REG_GAIN_TABLE_VLO. Gain values for HI / VLO mode {GAIN_INI, GAIN_MIN, GAIN_MAX}. */
#define IFFT_GAIN_HI_INI_CHN_2               120
#define IFFT_GAIN_HI_MIN_CHN_2               120
#define IFFT_GAIN_HI_MAX_CHN_2               120

#define IFFT_GAIN_VLO_INI_CHN_2              256
#define IFFT_GAIN_VLO_MIN_CHN_2              256
#define IFFT_GAIN_VLO_MAX_CHN_2              256

#define IFFT_GAIN_HI_INI_CHN_3               120
#define IFFT_GAIN_HI_MIN_CHN_3               120
#define IFFT_GAIN_HI_MAX_CHN_3               120

#define IFFT_GAIN_VLO_INI_CHN_3              287
#define IFFT_GAIN_VLO_MIN_CHN_3              287
#define IFFT_GAIN_VLO_MAX_CHN_3              287

#define IFFT_GAIN_HI_INI_CHN_4               85
#define IFFT_GAIN_HI_MIN_CHN_4               85
#define IFFT_GAIN_HI_MAX_CHN_4               85

#define IFFT_GAIN_VLO_INI_CHN_4              287
#define IFFT_GAIN_VLO_MIN_CHN_4              287
#define IFFT_GAIN_VLO_MAX_CHN_4              287

#define IFFT_GAIN_HI_INI_CHN_5               120
#define IFFT_GAIN_HI_MIN_CHN_5               120
#define IFFT_GAIN_HI_MAX_CHN_5               120

#define IFFT_GAIN_VLO_INI_CHN_5              256
#define IFFT_GAIN_VLO_MIN_CHN_5              256
#define IFFT_GAIN_VLO_MAX_CHN_5              256

#define IFFT_GAIN_HI_INI_CHN_6               120
#define IFFT_GAIN_HI_MIN_CHN_6               30
#define IFFT_GAIN_HI_MAX_CHN_6               256

#define IFFT_GAIN_VLO_INI_CHN_6              256
#define IFFT_GAIN_VLO_MIN_CHN_6              128
#define IFFT_GAIN_VLO_MAX_CHN_6              287

#define IFFT_GAIN_HI_INI_CHN_7               169
#define IFFT_GAIN_HI_MIN_CHN_7               169
#define IFFT_GAIN_HI_MAX_CHN_7               169

#define IFFT_GAIN_VLO_INI_CHN_7              287
#define IFFT_GAIN_VLO_MIN_CHN_7              287
#define IFFT_GAIN_VLO_MAX_CHN_7              287

#define IFFT_GAIN_HI_INI_CHN_8               240
#define IFFT_GAIN_HI_MIN_CHN_8               240
#define IFFT_GAIN_HI_MAX_CHN_8               240

#define IFFT_GAIN_VLO_INI_CHN_8              256
#define IFFT_GAIN_VLO_MIN_CHN_8              256
#define IFFT_GAIN_VLO_MAX_CHN_8              256

/* ATPL360_REG_DACC_TABLE_CFG: Configuration for PRIME Channel 2 - 8 with PL460. */
#define DACC_CFG_TABLE_CHN_FCC               {0x00000000, 0x00000000, 0x00000100, 0x00000100, 0x00000000, 0x00000000, 0xFFFF00FF, 0x1B1B1B1B, \
					      0x00000000, 0x00000000, 0x00000006, 0x00000355, 0x00000000, 0x001020F0, 0x00000355, 0x00000000, 0x001020FF}

/* ATPL360_REG_NUM_TX_LEVELS: Default value 8 */
#define NUM_TX_LEVELS_CHN_FCC                8

/* ATPL360_REG_PLC_IC_DRIVER_CFG: PLC IC Driver used (PL460 1.5 Branch) */
#define PLC_IC_DRV_CFG_CHN_FCC               0x07

#endif /* PL460_1_5B_CHN_FCC_H_INCLUDED */