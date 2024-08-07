/**
 * \file
 *
 * \brief MAC_PIB: PRIME MAC information base
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

#ifndef MAC_PIB_H_INCLUDE
#define MAC_PIB_H_INCLUDE

/* System includes */
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "conf_prime_stack.h"

/* @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/* @endcond */

/**
 * \weakgroup prime_mac_group
 * @{
 */

/** \brief PHY statistical PIB attributes */
/* @{ */
#define PIB_PHY_STATS_CRC_INCORRECT             0x00A0
#define PIB_PHY_STATS_CRC_FAIL_COUNT            0x00A1
#define PIB_PHY_STATS_TX_DROP_COUNT             0x00A2
#define PIB_PHY_STATS_RX_DROP_COUNT             0x00A3
#define PIB_PHY_STATS_RX_TOTAL_COUNT            0x00A4
#define PIB_PHY_STATS_BLK_AVG_EVM               0x00A5
#define PIB_PHY_EMA_SMOOTHING                   0x00A8
#define PIB_PHY_RF_STATS_CRC_INCORRECT          0x101A
#define PIB_PHY_RF_STATS_TX_DROP_COUNT          0x101B
#define PIB_PHY_RF_STATS_RX_DROP_COUNT          0x101C
#define PIB_PHY_RF_STATS_RX_TOTAL_COUNT         0x101D

/* @} */

/** \brief PHY implementation PIB attributes */
/* @{ */
#define PIB_PHY_TX_QUEUE_LEN                    0x00B0
#define PIB_PHY_RX_QUEUE_LEN                    0x00B1
#define PIB_PHY_TX_PROCESSING_DELAY             0x00B2
#define PIB_PHY_RX_PROCESSING_DELAY             0x00B3
#define PIB_PHY_AGC_MIN_GAIN                    0x00B4
#define PIB_PHY_AGC_STEP_VALUE                  0x00B5
#define PIB_PHY_AGC_STEP_NUMBER                 0x00B6
/* @} */

/** \brief PHY PIB attributes of RF */
/* @{ */
#define PIB_MAX_PHY_PACKET_SIZE                 0x1000
#define PIB_TURNAROUND_TIME                     0x1001
#define PIB_PHY_RF_CHANNEL                      0x1010
#define PIB_PHY_TX_POWER                        0x1011
#define PIB_PHY_FSK_FEC_ENABLED                 0x1012
#define PIB_PHY_FSK_FEC_INTERLEAVING_RSC        0x1013
#define PIB_PHY_FSK_FEC_SCHEME                  0x1014
#define PIB_PHY_FSK_PREAMBLE_LENGTH             0x1015
#define PIB_PHY_SUN_FSK_SFD                     0x1016
#define PIB_PHY_FSK_SCRAMBLE_PSDU               0x1017
#define PIB_PHY_CCA_DURATION                    0x1018
#define PIB_PHY_CCA_THRESHOLD                   0x1019
/* @} */

/** \brief MAC variable PIB attributes */
/* @{ */
#define PIB_MAC_VERSION                         0x0001  /* v1.4 */
#define PIB_MAC_MIN_SWITCH_SEARCH_TIME          0x0010
#define PIB_MAC_MAX_PROMOTION_PDU               0x0011
#define PIB_MAC_PROMOTION_PDU_TX_PERIOD         0x0012
#define PIB_MAC_BEACONS_PER_FRAME               0x0013  /* v1.3 */
#define PIB_MAC_SCP_MAX_TX_ATTEMPTS             0x0014
#define PIB_MAC_CTL_RE_TX_TIMER                 0x0015  /* v1.3 */
#define PIB_MAC_MIN_CTL_RE_TX_TIMER             0x0015  /* v1.4 */
#define PIB_MAC_TRAFFIC_BAND_TIMEOUT            0x0016  /* v1.4 */
#define PIB_MAC_SCP_CH_SENSE_COUNT              0x0017
#define PIB_MAC_MAX_CTL_RE_TX                   0x0018  /* v1.3 */
#define PIB_MAC_CTL_MSG_FAIL_TIME               0x0018  /* v1.4 */
#define PIB_MAC_EMA_SMOOTHING                   0x0019
#define PIB_MAC_MIN_BAND_SEARCH_TIME            0x001A  /* v1.4 */
#define PIB_MAC_PROMOTION_MAX_TX_PERIOD         0x001B  /* v1.4 */
#define PIB_MAC_PROMOTION_MIN_TX_PERIOD         0x001C  /* v1.4 */
#define PIB_MAC_SAR_SIZE                        0x001D  /* v1.4 */
#define PIB_MAC_MAX_BAND_SEARCH_TIME            0x001E  /* v1.4 */
#define PIB_MAC_EUI_48                          0x001F  /* v1.4 */
#define PIB_MAC_CSMA_R1                         0x0034  /* v1.4 */
#define PIB_MAC_CSMA_R2                         0x0035  /* v1.4 */
#define PIB_MAC_CSMA_DELAY                      0x0038  /* v1.4 */
#define PIB_MAC_CSMA_R1_ROBUST                  0x003B  /* v1.4 */
#define PIB_MAC_CSMA_R2_ROBUST                  0x003C  /* v1.4 */
#define PIB_MAC_CSMA_DELAY_ROBUST               0x003D  /* v1.4 */
#define PIB_MAC_ALV_TIME_MODE                   0x003E  /* v1.4 */
#define PIB_MAC_ACTION_ROBUSTNESS_MGMT          0x004A  /* v1.4 */
#define PIB_MAC_UPDATED_RM_TIMEOUT              0x004B  /* v1.4 */
#define PIB_MAC_ALV_HOP_REPETITIONS             0x004C  /* v1.4 */
#define PIB_MAC_PHY_CHN_CHANGE                  0x004D  /* v1.4 */
#define PIB_MAC_HOPPING_NUMBER_RF_CHANNELS      0x0090  /* v1.4 */
#define PIB_MAC_HOPPING_SEQUENCE_LENGTH         0x0091  /* v1.4 */
#define PIB_MAC_HOPPING_SEQUENCE_POSITION       0x0092  /* v1.4 Non used*/
#define PIB_MAC_HOPPING_BCN_SEQUENCE_LENGTH     0x0093  /* v1.4 */
#define PIB_MAC_HOPPING_BCN_SEQUENCE_POSITION   0x0094  /* v1.4 */
#define PIB_MAC_MIN_BE                          0x0098  /* v1.4 */
#define PIB_MAC_MAX_BE                          0x0099  /* v1.4 */
#define PIB_MAC_MAX_CSMA_BACKOFFS               0x009A  /* v1.4 */
#define PIB_MAC_HOPPING_PROMOTION_MAX_TX_PERIOD 0x009B  /* v1.4 */
#define PIB_MAC_HOPPING_PROMOTION_MIN_TX_PERIOD 0x009C  /* v1.4 */
#define PIB_MAC_HOPPING_INIT_CHANNEL_LIST       0x009D  /* v1.4 */
#define PIB_MAC_HOPPING_INIT_BCN_CHANNEL_LIST   0x009E  /* v1.4 */

/* @} */

/** \brief MAC functional PIB attributes */
/* @{ */
#define PIB_MAC_LNID                            0x0020
#define PIB_MAC_LSID                            0x0021
#define PIB_MAC_SID                             0x0022
#define PIB_MAC_SNA                             0x0023
#define PIB_MAC_STATE                           0x0024
#define PIB_MAC_SCP_LENGTH                      0x0025
#define PIB_MAC_NODE_HIERARCHY_LEVEL            0x0026
#define PIB_MAC_BEACON_SLOT_COUNT               0x0027  /* v1.3 */
#define PIB_MAC_BEACON_RX_SLOT                  0x0028  /* v1.3 */
#define PIB_MAC_BEACON_TX_SLOT                  0x0029  /* v1.3 */
#define PIB_MAC_BEACON_RX_FREQUENCY             0x002A
#define PIB_MAC_BEACON_TX_FREQUENCY             0x002B
#define PIB_MAC_MAC_CAPABILITES                 0x002C
#define PIB_MAC_FRAME_LENGTH                    0x002D  /* v1.4 */
#define PIB_MAC_CFP_LENGTH                      0x002E  /* v1.4 */
#define PIB_MAC_GUARD_TIME                      0x002F  /* v1.4 */
#define PIB_MAC_BC_MODE                         0x0030  /* v1.4 */
#define PIB_MAC_BEACON_RX_QLTY                  0x0032  /* v1.4 */
#define PIB_MAC_BEACON_TX_QLTY                  0x0033  /* v1.4 */
#define PIB_MAC_BEACON_RX_POS                   0x0039  /* v1.4 */
#define PIB_MAC_BEACON_TX_POS                   0x003A  /* v1.4 */
/* @} */

/** \brief MAC statistical PIB attributes */
/* @{ */
#define PIB_MAC_TX_DATAPKT_COUNT                0x0040
#define PIB_MAC_RX_DATAPKT_COUNT                0x0041
#define PIB_MAC_TX_CTRLPKT_COUNT                0x0042
#define PIB_MAC_RX_CTRLPKT_COUNT                0x0043
#define PIB_MAC_CSMA_FAIL_COUNT                 0x0044
#define PIB_MAC_CSMA_CH_BUSY_COUNT              0x0045
#define PIB_MAC_RF_CSMA_FAIL_COUNT              0x0046
#define PIB_MAC_RF_CSMA_CH_BUSY_COUNT           0x0047
/* @} */

/** \brief MAC list PIB attributes */
/* @{ */
#define PIB_MAC_LIST_REGISTER_DEVICES           0x0050
#define PIB_MAC_LIST_ACTIVE_CONN                0x0051
#define PIB_MAC_LIST_MCAST_ENTRIES              0x0052
#define PIB_MAC_LIST_SWITCH_TABLE_1_3           0x0053  /* v1.3 */
#define PIB_MAC_LIST_SWITCH_TABLE_1_4           0x005A  /* v1.4 */
#define PIB_MAC_LIST_DIRECT_CONN                0x0054
#define PIB_MAC_LIST_DIRECT_TABLE               0x0055
#define PIB_MAC_LIST_AVAILABLE_SWITCHES         0x0056
#define PIB_MAC_LIST_PHY_COMM_1_3               0x0057  /* v1.3 */
#define PIB_MAC_LIST_PHY_COMM_1_4               0x0059  /* v1.4 */
#define PIB_MAC_LIST_ACTIVE_CONN_EX             0x0058
#define PIB_MAC_LIST_SWITCHES_MP                0x2000  /* v1.4 */
#define PIB_MAC_LIST_REGISTER_DEVICES_MP        0x2050  /* v1.4 */
#define PIB_MAC_LIST_AVAILABLE_SWITCHES_MP      0x2056  /* v1.4 */
#define PIB_MAC_LIST_PHY_COMM_MP                0x2059  /* v1.4 */
/* @} */

/** \brief MAC security PIB attributes */
/* @{ */
#define PIB_MAC_SEC_DUK                         0x005B  /* v1.4 */
#define PIB_MAC_UPDATE_KEYS_TIME                0x005C  /* v1.4 */
/* @} */

/** \brief MAC action PIB attributes */
/* @{ */
#define PIB_MAC_ACTION_TX_DATA                  0x0060
#define PIB_MAC_ACTION_CONN_CLOSE               0x0061
#define PIB_MAC_ACTION_REG_REJECT               0x0062
#define PIB_MAC_ACTION_PRO_REJECT               0x0063
#define PIB_MAC_ACTION_UNREGISTER               0x0064
#define PIB_MAC_ACTION_PROMOTE                  0x0065
#define PIB_MAC_ACTION_DEMOTE                   0x0066
#define PIB_MAC_ACTION_REJECT                   0x0067
#define PIB_MAC_ACTION_ALIVE_TIME               0x0068
#define PIB_MAC_ACTION_PRM                      0x0069  /* v1.3 */
#define PIB_MAC_ACTION_BROADCAST_DATA_BURST     0x006A
#define PIB_MAC_ACTION_MGMT_CON                 0x006B
#define PIB_MAC_ACTION_MGMT_MUL                 0x006C
#define PIB_MAC_ACTION_UNREGISTER_BN            0x006D
#define PIB_MAC_ACTION_CONN_CLOSE_BN            0x006E
#define PIB_MAC_ACTION_SEGMENTED_432            0x006F
#define PIB_MAC_ACTION_APPEMU_DATA_BURST        0x0080
#define PIB_MAC_ACTION_MGMT_DATA_BURST          0x0081  /* v1.4 */
#define PIB_MAC_ACTION_PRO_BCN                  0x0082  /* v1.4 */
#define PIB_MAC_ACTION_PROMOTE_DS               0x0083  /* v1.4 */
/* @} */

/** \brief Management Plane firmware upgrade PIB attributes */
/* @{ */
#define PIB_FU_APP_FWDL_RUNNING                 0x0070
#define PIB_FU_APP_FWDL_RX_PKT_COUNT            0x0071
/* @} */

/** \brief MAC application PIB attributes */
/* @{ */
#define PIB_MAC_APP_FW_VERSION                  0x0075
#define PIB_MAC_APP_VENDOR_ID                   0x0076
#define PIB_MAC_APP_PRODUCT_ID                  0x0077
#define PIB_MAC_APP_LIST_ZC_STATUS              0x0078
/* @} */

/** \brief Proprietary MAC certification PIB attributes */
/* @{ */
#define PIB_MAC_ACTION_CFP_LENGTH               0x810D
#define PIB_MAC_ACTION_BCN_SLOT_COUNT           0x810E  /* v1.3 */
#define PIB_MAC_ALV_MIN_LEVEL                   0x810F  /* v1.4 */
#define PIB_MAC_ACTION_FRAME_LENGTH             0x8110  /* v1.4 */
#define PIB_CERTIFICATION_MODE                  0x8120
#define PIB_CERTIFICATION_SEND_MSG              0x8121
#define PIB_MAC_ACTION_ARQ_WIN_SIZE             0x8124
#define PIB_CERT_MIN_LEVEL_TO_REG               0x8130  /* v1.4 */
#define PIB_BCN_SLOTS_BUSY                      0x8131  /* v1.3 */
/* @} */

/** \brief Proprietary MAC manufacturing test process (MTP) PIB attributes */
/* @{ */
#define PIB_MTP_PHY_TX_TIME                     0x8085
#define PIB_MTP_PHY_RMS_CALC_CORRECTED          0x8086
#define PIB_MTP_PHY_EXECUTE_CALIBRATION         0x8087
#define PIB_MTP_PHY_RX_PARAMS                   0x8088
#define PIB_MTP_PHY_TX_PARAMS                   0x8089
#define PIB_MTP_PHY_CONTINUOUS_TX               0x808A
#define PIB_MTP_PHY_ENABLE                      0x808E
#define PIB_MTP_MAC_EUI_48                      0x8100
#define PIB_MTP_MAC_WRITE_SNA                   0x8123
/* @} */

/** \brief Proprietary Management Plane firmware upgrade PIB attributes */
/* @{ */
#define PIB_FU_LIST                             0x8350
/* @} */

/** \brief Other vendor specific PIB attributes */
/* @{ */
#define PIB_PHY_SW_VERSION                      0x8080
#define PIB_PHY_SW_RF_VERSION                   0x9080  /* v1.4 */
#define PIB_PHY_ZCT                             0x8081
#define PIB_PHY_HOST_VERSION                    0x8082
#define PIB_PHY_TX_CHANNEL                      0x8090
#define PIB_PHY_TXRX_CHANNEL_LIST               0x8092
#define PIB_PHY_TXRX_DOUBLE_CHANNEL_LIST        0x8093
#define PIB_MAC_PLC_STATE                       0x8101
#define PIB_MAC_SERVICE_STATE                   0x8102
#define PIB_MAC_REG_RSS                         0x8103
#define PIB_PHY_SNIFFER_ENABLED                 0x8106
#define PIB_MAC_INTERNAL_SW_VERSION             0x8126
#define PIB_MAC_ACTION_MGMT_MUL_SEND_DATA       0x8132  /* v1.4 */
#define PIB_MAC_ACTION_CFG_BCN_TX_SCHEME        0x8133  /* v1.4 */
#define PIB_MAC_ACTION_ALV_TYPE                 0x8134  /* v1.4 */
#define PIB_MAC_CHN_SCANNING_MODE               0x8135  /* v1.4 */
#define PIB_MAC_ACTION_CFG_BCN_SWITCH_RATE      0x8136  /* v1.4 */
#define PIB_MAC_ACTION_CFG_SEC_PROF             0x8137  /* v1.4 */
#define PIB_MAC_SEC_DUK_BN                      0x8140  /* v1.4 */
#define PIB_MAC_SEC_PROFILE_USED                0x8141  /* v1.4 */
#define PIB_MAC_SEC_OLD_SWK_TIME                0x8142  /* v1.4 */
#define PIB_MAC_WHITELIST                       0x8150
#define PIB_MAC_WHITELIST_ENABLED               0x8151
#define PIB_MAC_ACTION_CLEAR_NWK_STRUCTURE      0x8152
#define PIB_432_CON_STATE                       0x8200
#define PIB_CL_INTERNAL_SW_VERSION              0x8201
#define PIB_432_LIST_NODES                      0x8250
#define PIB_PHY_DRV_AUTODETECT_BRANCH           0x8301
#define PIB_PHY_DRV_IMPEDANCE                   0x8302
#define PIB_PHY_DRV_ATTENUATION                 0x8303
/* @} */

/* @} */

/* @cond 0 */
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/* @endcond */
#endif /* MAC_PIB_H_INCLUDE */
