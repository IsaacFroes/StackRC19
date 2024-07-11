/* ---------------------------------------------------------------------------- */
/*                Microchip Microcontroller Software Support                    */
/*                       SAM Software Package License                           */
/* ---------------------------------------------------------------------------- */
/* Copyright (c) 2022, Microchip Technology Inc.                    */
/*                                                                              */
/* All rights reserved.                                                         */
/*                                                                              */
/* Redistribution and use in source and binary forms, with or without           */
/* modification, are permitted provided that the following condition is met:    */
/*                                                                              */
/* - Redistributions of source code must retain the above copyright notice,     */
/* this list of conditions and the disclaimer below.                            */
/*                                                                              */
/* Microchip's name may not be used to endorse or promote products derived from */
/* this software without specific prior written permission.                     */
/*                                                                              */
/* DISCLAIMER:  THIS SOFTWARE IS PROVIDED BY MICROCHIP "AS IS" AND ANY EXPRESS  */
/* OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES */
/* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT    */
/* ARE DISCLAIMED. IN NO EVENT SHALL MICROCHIP BE LIABLE FOR ANY DIRECT,        */
/* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES           */
/* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; */
/* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND  */
/* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   */
/* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF     */
/* THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.            */
/* ---------------------------------------------------------------------------- */

#ifndef _PIC32CX_PWM_INSTANCE_
#define _PIC32CX_PWM_INSTANCE_

/* ========== Register definition for PWM peripheral ========== */
#if (defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
  #define REG_PWM_CLK                       (0x4801C000U) /**< \brief (PWM) PWM Clock Register */
  #define REG_PWM_ENA                       (0x4801C004U) /**< \brief (PWM) PWM Enable Register */
  #define REG_PWM_DIS                       (0x4801C008U) /**< \brief (PWM) PWM Disable Register */
  #define REG_PWM_SR                        (0x4801C00CU) /**< \brief (PWM) PWM Status Register */
  #define REG_PWM_IER1                      (0x4801C010U) /**< \brief (PWM) PWM Interrupt Enable Register 1 */
  #define REG_PWM_IDR1                      (0x4801C014U) /**< \brief (PWM) PWM Interrupt Disable Register 1 */
  #define REG_PWM_IMR1                      (0x4801C018U) /**< \brief (PWM) PWM Interrupt Mask Register 1 */
  #define REG_PWM_ISR1                      (0x4801C01CU) /**< \brief (PWM) PWM Interrupt Status Register 1 */
  #define REG_PWM_SCM                       (0x4801C020U) /**< \brief (PWM) PWM Sync Channels Mode Register */
  #define REG_PWM_SCUC                      (0x4801C028U) /**< \brief (PWM) PWM Sync Channels Update Control Register */
  #define REG_PWM_SCUP                      (0x4801C02CU) /**< \brief (PWM) PWM Sync Channels Update Period Register */
  #define REG_PWM_SCUPUPD                   (0x4801C030U) /**< \brief (PWM) PWM Sync Channels Update Period Update Register */
  #define REG_PWM_IER2                      (0x4801C034U) /**< \brief (PWM) PWM Interrupt Enable Register 2 */
  #define REG_PWM_IDR2                      (0x4801C038U) /**< \brief (PWM) PWM Interrupt Disable Register 2 */
  #define REG_PWM_IMR2                      (0x4801C03CU) /**< \brief (PWM) PWM Interrupt Mask Register 2 */
  #define REG_PWM_ISR2                      (0x4801C040U) /**< \brief (PWM) PWM Interrupt Status Register 2 */
  #define REG_PWM_OOV                       (0x4801C044U) /**< \brief (PWM) PWM Output Override Value Register */
  #define REG_PWM_OS                        (0x4801C048U) /**< \brief (PWM) PWM Output Selection Register */
  #define REG_PWM_OSS                       (0x4801C04CU) /**< \brief (PWM) PWM Output Selection Set Register */
  #define REG_PWM_OSC                       (0x4801C050U) /**< \brief (PWM) PWM Output Selection Clear Register */
  #define REG_PWM_OSSUPD                    (0x4801C054U) /**< \brief (PWM) PWM Output Selection Set Update Register */
  #define REG_PWM_OSCUPD                    (0x4801C058U) /**< \brief (PWM) PWM Output Selection Clear Update Register */
  #define REG_PWM_FMR                       (0x4801C05CU) /**< \brief (PWM) PWM Fault Mode Register */
  #define REG_PWM_FSR                       (0x4801C060U) /**< \brief (PWM) PWM Fault Status Register */
  #define REG_PWM_FCR                       (0x4801C064U) /**< \brief (PWM) PWM Fault Clear Register */
  #define REG_PWM_FPV1                      (0x4801C068U) /**< \brief (PWM) PWM Fault Protection Value Register 1 */
  #define REG_PWM_FPE                       (0x4801C06CU) /**< \brief (PWM) PWM Fault Protection Enable Register */
  #define REG_PWM_ELMR0                     (0x4801C07CU) /**< \brief (PWM) PWM Event Line 0 Mode Register */
  #define REG_PWM_SSPR                      (0x4801C0A0U) /**< \brief (PWM) PWM Spread Spectrum Register */
  #define REG_PWM_SSPUP                     (0x4801C0A4U) /**< \brief (PWM) PWM Spread Spectrum Update Register */
  #define REG_PWM_SMMR                      (0x4801C0B0U) /**< \brief (PWM) PWM Stepper Motor Mode Register */
  #define REG_PWM_FPV2                      (0x4801C0C0U) /**< \brief (PWM) PWM Fault Protection Value 2 Register */
  #define REG_PWM_WPCR                      (0x4801C0E4U) /**< \brief (PWM) PWM Write Protection Control Register */
  #define REG_PWM_WPSR                      (0x4801C0E8U) /**< \brief (PWM) PWM Write Protection Status Register */
  #define REG_PWM_TPR                       (0x4801C108U) /**< \brief (PWM) Transmit Pointer Register */
  #define REG_PWM_TCR                       (0x4801C10CU) /**< \brief (PWM) Transmit Counter Register */
  #define REG_PWM_TNPR                      (0x4801C118U) /**< \brief (PWM) Transmit Next Pointer Register */
  #define REG_PWM_TNCR                      (0x4801C11CU) /**< \brief (PWM) Transmit Next Counter Register */
  #define REG_PWM_PTCR                      (0x4801C120U) /**< \brief (PWM) Transfer Control Register */
  #define REG_PWM_PTSR                      (0x4801C124U) /**< \brief (PWM) Transfer Status Register */
  #define REG_PWM_PWPMR                     (0x4801C128U) /**< \brief (PWM) Write Protection Mode Register */
  #define REG_PWM_CMPV0                     (0x4801C130U) /**< \brief (PWM) PWM Comparison 0 Value Register */
  #define REG_PWM_CMPVUPD0                  (0x4801C134U) /**< \brief (PWM) PWM Comparison 0 Value Update Register */
  #define REG_PWM_CMPM0                     (0x4801C138U) /**< \brief (PWM) PWM Comparison 0 Mode Register */
  #define REG_PWM_CMPMUPD0                  (0x4801C13CU) /**< \brief (PWM) PWM Comparison 0 Mode Update Register */
  #define REG_PWM_CMPV1                     (0x4801C140U) /**< \brief (PWM) PWM Comparison 1 Value Register */
  #define REG_PWM_CMPVUPD1                  (0x4801C144U) /**< \brief (PWM) PWM Comparison 1 Value Update Register */
  #define REG_PWM_CMPM1                     (0x4801C148U) /**< \brief (PWM) PWM Comparison 1 Mode Register */
  #define REG_PWM_CMPMUPD1                  (0x4801C14CU) /**< \brief (PWM) PWM Comparison 1 Mode Update Register */
  #define REG_PWM_CMPV2                     (0x4801C150U) /**< \brief (PWM) PWM Comparison 2 Value Register */
  #define REG_PWM_CMPVUPD2                  (0x4801C154U) /**< \brief (PWM) PWM Comparison 2 Value Update Register */
  #define REG_PWM_CMPM2                     (0x4801C158U) /**< \brief (PWM) PWM Comparison 2 Mode Register */
  #define REG_PWM_CMPMUPD2                  (0x4801C15CU) /**< \brief (PWM) PWM Comparison 2 Mode Update Register */
  #define REG_PWM_CMPV3                     (0x4801C160U) /**< \brief (PWM) PWM Comparison 3 Value Register */
  #define REG_PWM_CMPVUPD3                  (0x4801C164U) /**< \brief (PWM) PWM Comparison 3 Value Update Register */
  #define REG_PWM_CMPM3                     (0x4801C168U) /**< \brief (PWM) PWM Comparison 3 Mode Register */
  #define REG_PWM_CMPMUPD3                  (0x4801C16CU) /**< \brief (PWM) PWM Comparison 3 Mode Update Register */
  #define REG_PWM_CMPV4                     (0x4801C170U) /**< \brief (PWM) PWM Comparison 4 Value Register */
  #define REG_PWM_CMPVUPD4                  (0x4801C174U) /**< \brief (PWM) PWM Comparison 4 Value Update Register */
  #define REG_PWM_CMPM4                     (0x4801C178U) /**< \brief (PWM) PWM Comparison 4 Mode Register */
  #define REG_PWM_CMPMUPD4                  (0x4801C17CU) /**< \brief (PWM) PWM Comparison 4 Mode Update Register */
  #define REG_PWM_CMPV5                     (0x4801C180U) /**< \brief (PWM) PWM Comparison 5 Value Register */
  #define REG_PWM_CMPVUPD5                  (0x4801C184U) /**< \brief (PWM) PWM Comparison 5 Value Update Register */
  #define REG_PWM_CMPM5                     (0x4801C188U) /**< \brief (PWM) PWM Comparison 5 Mode Register */
  #define REG_PWM_CMPMUPD5                  (0x4801C18CU) /**< \brief (PWM) PWM Comparison 5 Mode Update Register */
  #define REG_PWM_CMPV6                     (0x4801C190U) /**< \brief (PWM) PWM Comparison 6 Value Register */
  #define REG_PWM_CMPVUPD6                  (0x4801C194U) /**< \brief (PWM) PWM Comparison 6 Value Update Register */
  #define REG_PWM_CMPM6                     (0x4801C198U) /**< \brief (PWM) PWM Comparison 6 Mode Register */
  #define REG_PWM_CMPMUPD6                  (0x4801C19CU) /**< \brief (PWM) PWM Comparison 6 Mode Update Register */
  #define REG_PWM_CMPV7                     (0x4801C1A0U) /**< \brief (PWM) PWM Comparison 7 Value Register */
  #define REG_PWM_CMPVUPD7                  (0x4801C1A4U) /**< \brief (PWM) PWM Comparison 7 Value Update Register */
  #define REG_PWM_CMPM7                     (0x4801C1A8U) /**< \brief (PWM) PWM Comparison 7 Mode Register */
  #define REG_PWM_CMPMUPD7                  (0x4801C1ACU) /**< \brief (PWM) PWM Comparison 7 Mode Update Register */
  #define REG_PWM_CMR0                      (0x4801C200U) /**< \brief (PWM) PWM Channel Mode Register (ch_num = 0) */
  #define REG_PWM_CDTY0                     (0x4801C204U) /**< \brief (PWM) PWM Channel Duty Cycle Register (ch_num = 0) */
  #define REG_PWM_CDTYUPD0                  (0x4801C208U) /**< \brief (PWM) PWM Channel Duty Cycle Update Register (ch_num = 0) */
  #define REG_PWM_CPRD0                     (0x4801C20CU) /**< \brief (PWM) PWM Channel Period Register (ch_num = 0) */
  #define REG_PWM_CPRDUPD0                  (0x4801C210U) /**< \brief (PWM) PWM Channel Period Update Register (ch_num = 0) */
  #define REG_PWM_CCNT0                     (0x4801C214U) /**< \brief (PWM) PWM Channel Counter Register (ch_num = 0) */
  #define REG_PWM_DT0                       (0x4801C218U) /**< \brief (PWM) PWM Channel Dead Time Register (ch_num = 0) */
  #define REG_PWM_DTUPD0                    (0x4801C21CU) /**< \brief (PWM) PWM Channel Dead Time Update Register (ch_num = 0) */
  #define REG_PWM_CMR1                      (0x4801C220U) /**< \brief (PWM) PWM Channel Mode Register (ch_num = 1) */
  #define REG_PWM_CDTY1                     (0x4801C224U) /**< \brief (PWM) PWM Channel Duty Cycle Register (ch_num = 1) */
  #define REG_PWM_CDTYUPD1                  (0x4801C228U) /**< \brief (PWM) PWM Channel Duty Cycle Update Register (ch_num = 1) */
  #define REG_PWM_CPRD1                     (0x4801C22CU) /**< \brief (PWM) PWM Channel Period Register (ch_num = 1) */
  #define REG_PWM_CPRDUPD1                  (0x4801C230U) /**< \brief (PWM) PWM Channel Period Update Register (ch_num = 1) */
  #define REG_PWM_CCNT1                     (0x4801C234U) /**< \brief (PWM) PWM Channel Counter Register (ch_num = 1) */
  #define REG_PWM_DT1                       (0x4801C238U) /**< \brief (PWM) PWM Channel Dead Time Register (ch_num = 1) */
  #define REG_PWM_DTUPD1                    (0x4801C23CU) /**< \brief (PWM) PWM Channel Dead Time Update Register (ch_num = 1) */
  #define REG_PWM_CMR2                      (0x4801C240U) /**< \brief (PWM) PWM Channel Mode Register (ch_num = 2) */
  #define REG_PWM_CDTY2                     (0x4801C244U) /**< \brief (PWM) PWM Channel Duty Cycle Register (ch_num = 2) */
  #define REG_PWM_CDTYUPD2                  (0x4801C248U) /**< \brief (PWM) PWM Channel Duty Cycle Update Register (ch_num = 2) */
  #define REG_PWM_CPRD2                     (0x4801C24CU) /**< \brief (PWM) PWM Channel Period Register (ch_num = 2) */
  #define REG_PWM_CPRDUPD2                  (0x4801C250U) /**< \brief (PWM) PWM Channel Period Update Register (ch_num = 2) */
  #define REG_PWM_CCNT2                     (0x4801C254U) /**< \brief (PWM) PWM Channel Counter Register (ch_num = 2) */
  #define REG_PWM_DT2                       (0x4801C258U) /**< \brief (PWM) PWM Channel Dead Time Register (ch_num = 2) */
  #define REG_PWM_DTUPD2                    (0x4801C25CU) /**< \brief (PWM) PWM Channel Dead Time Update Register (ch_num = 2) */
  #define REG_PWM_CMUPD0                    (0x4801C400U) /**< \brief (PWM) PWM Channel Mode Update Register (ch_num = 0) */
  #define REG_PWM_CMUPD1                    (0x4801C420U) /**< \brief (PWM) PWM Channel Mode Update Register (ch_num = 1) */
  #define REG_PWM_ETRG1                     (0x4801C42CU) /**< \brief (PWM) PWM External Trigger Register 1 */
  #define REG_PWM_LEBR1                     (0x4801C430U) /**< \brief (PWM) PWM Leading-Edge Blanking Register 1 */
  #define REG_PWM_CMUPD2                    (0x4801C440U) /**< \brief (PWM) PWM Channel Mode Update Register (ch_num = 2) */
  #define REG_PWM_ETRG2                     (0x4801C44CU) /**< \brief (PWM) PWM External Trigger Register 2 */
  #define REG_PWM_LEBR2                     (0x4801C450U) /**< \brief (PWM) PWM Leading-Edge Blanking Register 2 */
#else
  #define REG_PWM_CLK      (*(__IO uint32_t*)0x4801C000U) /**< \brief (PWM) PWM Clock Register */
  #define REG_PWM_ENA      (*(__O  uint32_t*)0x4801C004U) /**< \brief (PWM) PWM Enable Register */
  #define REG_PWM_DIS      (*(__O  uint32_t*)0x4801C008U) /**< \brief (PWM) PWM Disable Register */
  #define REG_PWM_SR       (*(__I  uint32_t*)0x4801C00CU) /**< \brief (PWM) PWM Status Register */
  #define REG_PWM_IER1     (*(__O  uint32_t*)0x4801C010U) /**< \brief (PWM) PWM Interrupt Enable Register 1 */
  #define REG_PWM_IDR1     (*(__O  uint32_t*)0x4801C014U) /**< \brief (PWM) PWM Interrupt Disable Register 1 */
  #define REG_PWM_IMR1     (*(__I  uint32_t*)0x4801C018U) /**< \brief (PWM) PWM Interrupt Mask Register 1 */
  #define REG_PWM_ISR1     (*(__I  uint32_t*)0x4801C01CU) /**< \brief (PWM) PWM Interrupt Status Register 1 */
  #define REG_PWM_SCM      (*(__IO uint32_t*)0x4801C020U) /**< \brief (PWM) PWM Sync Channels Mode Register */
  #define REG_PWM_SCUC     (*(__IO uint32_t*)0x4801C028U) /**< \brief (PWM) PWM Sync Channels Update Control Register */
  #define REG_PWM_SCUP     (*(__IO uint32_t*)0x4801C02CU) /**< \brief (PWM) PWM Sync Channels Update Period Register */
  #define REG_PWM_SCUPUPD  (*(__O  uint32_t*)0x4801C030U) /**< \brief (PWM) PWM Sync Channels Update Period Update Register */
  #define REG_PWM_IER2     (*(__O  uint32_t*)0x4801C034U) /**< \brief (PWM) PWM Interrupt Enable Register 2 */
  #define REG_PWM_IDR2     (*(__O  uint32_t*)0x4801C038U) /**< \brief (PWM) PWM Interrupt Disable Register 2 */
  #define REG_PWM_IMR2     (*(__I  uint32_t*)0x4801C03CU) /**< \brief (PWM) PWM Interrupt Mask Register 2 */
  #define REG_PWM_ISR2     (*(__I  uint32_t*)0x4801C040U) /**< \brief (PWM) PWM Interrupt Status Register 2 */
  #define REG_PWM_OOV      (*(__IO uint32_t*)0x4801C044U) /**< \brief (PWM) PWM Output Override Value Register */
  #define REG_PWM_OS       (*(__IO uint32_t*)0x4801C048U) /**< \brief (PWM) PWM Output Selection Register */
  #define REG_PWM_OSS      (*(__O  uint32_t*)0x4801C04CU) /**< \brief (PWM) PWM Output Selection Set Register */
  #define REG_PWM_OSC      (*(__O  uint32_t*)0x4801C050U) /**< \brief (PWM) PWM Output Selection Clear Register */
  #define REG_PWM_OSSUPD   (*(__O  uint32_t*)0x4801C054U) /**< \brief (PWM) PWM Output Selection Set Update Register */
  #define REG_PWM_OSCUPD   (*(__O  uint32_t*)0x4801C058U) /**< \brief (PWM) PWM Output Selection Clear Update Register */
  #define REG_PWM_FMR      (*(__IO uint32_t*)0x4801C05CU) /**< \brief (PWM) PWM Fault Mode Register */
  #define REG_PWM_FSR      (*(__I  uint32_t*)0x4801C060U) /**< \brief (PWM) PWM Fault Status Register */
  #define REG_PWM_FCR      (*(__O  uint32_t*)0x4801C064U) /**< \brief (PWM) PWM Fault Clear Register */
  #define REG_PWM_FPV1     (*(__IO uint32_t*)0x4801C068U) /**< \brief (PWM) PWM Fault Protection Value Register 1 */
  #define REG_PWM_FPE      (*(__IO uint32_t*)0x4801C06CU) /**< \brief (PWM) PWM Fault Protection Enable Register */
  #define REG_PWM_ELMR0    (*(__IO uint32_t*)0x4801C07CU) /**< \brief (PWM) PWM Event Line 0 Mode Register */
  #define REG_PWM_SSPR     (*(__IO uint32_t*)0x4801C0A0U) /**< \brief (PWM) PWM Spread Spectrum Register */
  #define REG_PWM_SSPUP    (*(__O  uint32_t*)0x4801C0A4U) /**< \brief (PWM) PWM Spread Spectrum Update Register */
  #define REG_PWM_SMMR     (*(__IO uint32_t*)0x4801C0B0U) /**< \brief (PWM) PWM Stepper Motor Mode Register */
  #define REG_PWM_FPV2     (*(__IO uint32_t*)0x4801C0C0U) /**< \brief (PWM) PWM Fault Protection Value 2 Register */
  #define REG_PWM_WPCR     (*(__O  uint32_t*)0x4801C0E4U) /**< \brief (PWM) PWM Write Protection Control Register */
  #define REG_PWM_WPSR     (*(__I  uint32_t*)0x4801C0E8U) /**< \brief (PWM) PWM Write Protection Status Register */
  #define REG_PWM_TPR      (*(__IO uint32_t*)0x4801C108U) /**< \brief (PWM) Transmit Pointer Register */
  #define REG_PWM_TCR      (*(__IO uint32_t*)0x4801C10CU) /**< \brief (PWM) Transmit Counter Register */
  #define REG_PWM_TNPR     (*(__IO uint32_t*)0x4801C118U) /**< \brief (PWM) Transmit Next Pointer Register */
  #define REG_PWM_TNCR     (*(__IO uint32_t*)0x4801C11CU) /**< \brief (PWM) Transmit Next Counter Register */
  #define REG_PWM_PTCR     (*(__O  uint32_t*)0x4801C120U) /**< \brief (PWM) Transfer Control Register */
  #define REG_PWM_PTSR     (*(__I  uint32_t*)0x4801C124U) /**< \brief (PWM) Transfer Status Register */
  #define REG_PWM_PWPMR    (*(__IO uint32_t*)0x4801C128U) /**< \brief (PWM) Write Protection Mode Register */
  #define REG_PWM_CMPV0    (*(__IO uint32_t*)0x4801C130U) /**< \brief (PWM) PWM Comparison 0 Value Register */
  #define REG_PWM_CMPVUPD0 (*(__O  uint32_t*)0x4801C134U) /**< \brief (PWM) PWM Comparison 0 Value Update Register */
  #define REG_PWM_CMPM0    (*(__IO uint32_t*)0x4801C138U) /**< \brief (PWM) PWM Comparison 0 Mode Register */
  #define REG_PWM_CMPMUPD0 (*(__O  uint32_t*)0x4801C13CU) /**< \brief (PWM) PWM Comparison 0 Mode Update Register */
  #define REG_PWM_CMPV1    (*(__IO uint32_t*)0x4801C140U) /**< \brief (PWM) PWM Comparison 1 Value Register */
  #define REG_PWM_CMPVUPD1 (*(__O  uint32_t*)0x4801C144U) /**< \brief (PWM) PWM Comparison 1 Value Update Register */
  #define REG_PWM_CMPM1    (*(__IO uint32_t*)0x4801C148U) /**< \brief (PWM) PWM Comparison 1 Mode Register */
  #define REG_PWM_CMPMUPD1 (*(__O  uint32_t*)0x4801C14CU) /**< \brief (PWM) PWM Comparison 1 Mode Update Register */
  #define REG_PWM_CMPV2    (*(__IO uint32_t*)0x4801C150U) /**< \brief (PWM) PWM Comparison 2 Value Register */
  #define REG_PWM_CMPVUPD2 (*(__O  uint32_t*)0x4801C154U) /**< \brief (PWM) PWM Comparison 2 Value Update Register */
  #define REG_PWM_CMPM2    (*(__IO uint32_t*)0x4801C158U) /**< \brief (PWM) PWM Comparison 2 Mode Register */
  #define REG_PWM_CMPMUPD2 (*(__O  uint32_t*)0x4801C15CU) /**< \brief (PWM) PWM Comparison 2 Mode Update Register */
  #define REG_PWM_CMPV3    (*(__IO uint32_t*)0x4801C160U) /**< \brief (PWM) PWM Comparison 3 Value Register */
  #define REG_PWM_CMPVUPD3 (*(__O  uint32_t*)0x4801C164U) /**< \brief (PWM) PWM Comparison 3 Value Update Register */
  #define REG_PWM_CMPM3    (*(__IO uint32_t*)0x4801C168U) /**< \brief (PWM) PWM Comparison 3 Mode Register */
  #define REG_PWM_CMPMUPD3 (*(__O  uint32_t*)0x4801C16CU) /**< \brief (PWM) PWM Comparison 3 Mode Update Register */
  #define REG_PWM_CMPV4    (*(__IO uint32_t*)0x4801C170U) /**< \brief (PWM) PWM Comparison 4 Value Register */
  #define REG_PWM_CMPVUPD4 (*(__O  uint32_t*)0x4801C174U) /**< \brief (PWM) PWM Comparison 4 Value Update Register */
  #define REG_PWM_CMPM4    (*(__IO uint32_t*)0x4801C178U) /**< \brief (PWM) PWM Comparison 4 Mode Register */
  #define REG_PWM_CMPMUPD4 (*(__O  uint32_t*)0x4801C17CU) /**< \brief (PWM) PWM Comparison 4 Mode Update Register */
  #define REG_PWM_CMPV5    (*(__IO uint32_t*)0x4801C180U) /**< \brief (PWM) PWM Comparison 5 Value Register */
  #define REG_PWM_CMPVUPD5 (*(__O  uint32_t*)0x4801C184U) /**< \brief (PWM) PWM Comparison 5 Value Update Register */
  #define REG_PWM_CMPM5    (*(__IO uint32_t*)0x4801C188U) /**< \brief (PWM) PWM Comparison 5 Mode Register */
  #define REG_PWM_CMPMUPD5 (*(__O  uint32_t*)0x4801C18CU) /**< \brief (PWM) PWM Comparison 5 Mode Update Register */
  #define REG_PWM_CMPV6    (*(__IO uint32_t*)0x4801C190U) /**< \brief (PWM) PWM Comparison 6 Value Register */
  #define REG_PWM_CMPVUPD6 (*(__O  uint32_t*)0x4801C194U) /**< \brief (PWM) PWM Comparison 6 Value Update Register */
  #define REG_PWM_CMPM6    (*(__IO uint32_t*)0x4801C198U) /**< \brief (PWM) PWM Comparison 6 Mode Register */
  #define REG_PWM_CMPMUPD6 (*(__O  uint32_t*)0x4801C19CU) /**< \brief (PWM) PWM Comparison 6 Mode Update Register */
  #define REG_PWM_CMPV7    (*(__IO uint32_t*)0x4801C1A0U) /**< \brief (PWM) PWM Comparison 7 Value Register */
  #define REG_PWM_CMPVUPD7 (*(__O  uint32_t*)0x4801C1A4U) /**< \brief (PWM) PWM Comparison 7 Value Update Register */
  #define REG_PWM_CMPM7    (*(__IO uint32_t*)0x4801C1A8U) /**< \brief (PWM) PWM Comparison 7 Mode Register */
  #define REG_PWM_CMPMUPD7 (*(__O  uint32_t*)0x4801C1ACU) /**< \brief (PWM) PWM Comparison 7 Mode Update Register */
  #define REG_PWM_CMR0     (*(__IO uint32_t*)0x4801C200U) /**< \brief (PWM) PWM Channel Mode Register (ch_num = 0) */
  #define REG_PWM_CDTY0    (*(__IO uint32_t*)0x4801C204U) /**< \brief (PWM) PWM Channel Duty Cycle Register (ch_num = 0) */
  #define REG_PWM_CDTYUPD0 (*(__O  uint32_t*)0x4801C208U) /**< \brief (PWM) PWM Channel Duty Cycle Update Register (ch_num = 0) */
  #define REG_PWM_CPRD0    (*(__IO uint32_t*)0x4801C20CU) /**< \brief (PWM) PWM Channel Period Register (ch_num = 0) */
  #define REG_PWM_CPRDUPD0 (*(__O  uint32_t*)0x4801C210U) /**< \brief (PWM) PWM Channel Period Update Register (ch_num = 0) */
  #define REG_PWM_CCNT0    (*(__I  uint32_t*)0x4801C214U) /**< \brief (PWM) PWM Channel Counter Register (ch_num = 0) */
  #define REG_PWM_DT0      (*(__IO uint32_t*)0x4801C218U) /**< \brief (PWM) PWM Channel Dead Time Register (ch_num = 0) */
  #define REG_PWM_DTUPD0   (*(__O  uint32_t*)0x4801C21CU) /**< \brief (PWM) PWM Channel Dead Time Update Register (ch_num = 0) */
  #define REG_PWM_CMR1     (*(__IO uint32_t*)0x4801C220U) /**< \brief (PWM) PWM Channel Mode Register (ch_num = 1) */
  #define REG_PWM_CDTY1    (*(__IO uint32_t*)0x4801C224U) /**< \brief (PWM) PWM Channel Duty Cycle Register (ch_num = 1) */
  #define REG_PWM_CDTYUPD1 (*(__O  uint32_t*)0x4801C228U) /**< \brief (PWM) PWM Channel Duty Cycle Update Register (ch_num = 1) */
  #define REG_PWM_CPRD1    (*(__IO uint32_t*)0x4801C22CU) /**< \brief (PWM) PWM Channel Period Register (ch_num = 1) */
  #define REG_PWM_CPRDUPD1 (*(__O  uint32_t*)0x4801C230U) /**< \brief (PWM) PWM Channel Period Update Register (ch_num = 1) */
  #define REG_PWM_CCNT1    (*(__I  uint32_t*)0x4801C234U) /**< \brief (PWM) PWM Channel Counter Register (ch_num = 1) */
  #define REG_PWM_DT1      (*(__IO uint32_t*)0x4801C238U) /**< \brief (PWM) PWM Channel Dead Time Register (ch_num = 1) */
  #define REG_PWM_DTUPD1   (*(__O  uint32_t*)0x4801C23CU) /**< \brief (PWM) PWM Channel Dead Time Update Register (ch_num = 1) */
  #define REG_PWM_CMR2     (*(__IO uint32_t*)0x4801C240U) /**< \brief (PWM) PWM Channel Mode Register (ch_num = 2) */
  #define REG_PWM_CDTY2    (*(__IO uint32_t*)0x4801C244U) /**< \brief (PWM) PWM Channel Duty Cycle Register (ch_num = 2) */
  #define REG_PWM_CDTYUPD2 (*(__O  uint32_t*)0x4801C248U) /**< \brief (PWM) PWM Channel Duty Cycle Update Register (ch_num = 2) */
  #define REG_PWM_CPRD2    (*(__IO uint32_t*)0x4801C24CU) /**< \brief (PWM) PWM Channel Period Register (ch_num = 2) */
  #define REG_PWM_CPRDUPD2 (*(__O  uint32_t*)0x4801C250U) /**< \brief (PWM) PWM Channel Period Update Register (ch_num = 2) */
  #define REG_PWM_CCNT2    (*(__I  uint32_t*)0x4801C254U) /**< \brief (PWM) PWM Channel Counter Register (ch_num = 2) */
  #define REG_PWM_DT2      (*(__IO uint32_t*)0x4801C258U) /**< \brief (PWM) PWM Channel Dead Time Register (ch_num = 2) */
  #define REG_PWM_DTUPD2   (*(__O  uint32_t*)0x4801C25CU) /**< \brief (PWM) PWM Channel Dead Time Update Register (ch_num = 2) */
  #define REG_PWM_CMUPD0   (*(__O  uint32_t*)0x4801C400U) /**< \brief (PWM) PWM Channel Mode Update Register (ch_num = 0) */
  #define REG_PWM_CMUPD1   (*(__O  uint32_t*)0x4801C420U) /**< \brief (PWM) PWM Channel Mode Update Register (ch_num = 1) */
  #define REG_PWM_ETRG1    (*(__IO uint32_t*)0x4801C42CU) /**< \brief (PWM) PWM External Trigger Register 1 */
  #define REG_PWM_LEBR1    (*(__IO uint32_t*)0x4801C430U) /**< \brief (PWM) PWM Leading-Edge Blanking Register 1 */
  #define REG_PWM_CMUPD2   (*(__O  uint32_t*)0x4801C440U) /**< \brief (PWM) PWM Channel Mode Update Register (ch_num = 2) */
  #define REG_PWM_ETRG2    (*(__IO uint32_t*)0x4801C44CU) /**< \brief (PWM) PWM External Trigger Register 2 */
  #define REG_PWM_LEBR2    (*(__IO uint32_t*)0x4801C450U) /**< \brief (PWM) PWM Leading-Edge Blanking Register 2 */
#endif /* (defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

#endif /* _PIC32CX_PWM_INSTANCE_ */
