/**
 * \file
 *
 * \brief MAIN : Base Node modem application for PRIMEv1.3 (freeRTOS)
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

#include <board.h>
#include <sysclk.h>
#include <string.h>
#include <stdint.h>
#include <stdio_serial.h>
#include <conf_app_example.h>
#include <conf_board.h>

#include "asf.h"
#include "conf_oss.h"

/* From module: PLC Universal Serial Interface */
#include "hal.h"
#include "hal_private.h"

#include "app.h"

/**
 *  \mainpage Microchip PRIME Modem Application for Base Node for PRIMEv1.4
 *
 *  \section Purpose
 *
 *  This application provides the base node with modem capabilities.
 *
 *  \section Requirements
 *
 *  This package should be used in boards with dedicated PLC hardware.
 *
 *  \section Description
 *
 *  This application will configure the PRIME stack and its serial interface to
 * use PHY, MAC and IEC_432 layers as Base Node.
 *
 *  \section Usage
 *
 *  -# Build the program and download it into the evaluation board.
 *  -# The application will start PRIME standard as Base Node mode.
 *	-# The application is configured to serialize several protocols with
 * these settings:
 *    - Uart0 Serial port
 *	  - 115200 bauds
 *    - TX buffer: 1024 bytes, Rx buffer: 1024 bytes
 *    - Serialized protocols in use:
 *              - Prime Management
 *
 */

#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- Microchip PRIME Modem Application v1.3 for Base Node[OSS-FRTOS]--\r\n" \
	"-- "BOARD_NAME " --\r\n" \
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

#define SHOW_MESSAGE_TIME   (500)

/** Default empty dummy function*/
void dummy_user_app(void);
void user_app(void);

#ifdef __GNUC__
void vUserAppInitTask(void) __attribute__ ((weak, alias("dummy_user_app")));

#endif

#ifdef __ICCARM__
#pragma weak vUserAppInitTask=dummy_user_app
#endif

/** System status signaling timer */
static xTimerHandle xAppSignalTimer = NULL;
/** PLC activity signaling timer */
static xTimerHandle xPLCActivityTimer = NULL;

/** Define HAL API interface */
extern const hal_api_t hal_api;

#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE
#if (BOARD != ATPL360AMB) && (BOARD != ATPL360MB)
/** DATA activity signaling timer */
static xTimerHandle xDataActivityTimer = NULL;

/* icon blink status */
static uint8_t uc_blink_status;
#endif
#endif

/** WDT configuration (time in microseconds) */
#define WATCHDOG_TIME                     5000000

/**
 * \brief Configure the external SDRAM IS42S16400J_7B2LI.
 */
#if ((BOARD == PL360BN) || (BOARD == SAME70_XPLAINED)) && defined(CONF_BOARD_SDRAMC)
static void _prv_setup_sdram()
{
	volatile uint32_t i;
	volatile uint16_t *pSdram = (uint16_t *)(BOARD_SDRAM_ADDR);
	uint32_t ul_clk = sysclk_get_peripheral_hz();

	/* Enable peripheral clocks*/
	sysclk_enable_peripheral_clock(ID_SDRAMC);
	sysclk_enable_peripheral_clock(ID_PIOA);
	sysclk_enable_peripheral_clock(ID_PIOB);
	sysclk_enable_peripheral_clock(ID_PIOC);
	sysclk_enable_peripheral_clock(ID_PIOD);
	sysclk_enable_peripheral_clock(ID_PIOE);
	MATRIX->CCFG_SMCNFCS = CCFG_SMCNFCS_SDRAMEN;

	/* Prevent setting the SDRAMC entering to sleep mode */
	sleepmgr_lock_mode(SLEEPMGR_ACTIVE);

	/* SDRAM device configuration */
	/* Step 1. */
	/* Set the features of SDRAM device into the Configuration Register */
#if (BOARD == PL360BN)
	SDRAMC->SDRAMC_CR =
		  SDRAMC_CR_NC_COL8      // 8 column bits
		| SDRAMC_CR_NR_ROW12     // 12 row bits (4K)
		| SDRAMC_CR_CAS_LATENCY3 // CAS Latency 3
		| SDRAMC_CR_NB_BANK4     // 4 banks
		| SDRAMC_CR_DBW          // 16 bit
		| SDRAMC_CR_TWR(4)       // 4 SDCK cycles minimum.
		| SDRAMC_CR_TRC_TRFC(10) // Command period (Ref to Ref / ACT to ACT) 63ns minimum. If SDCK=143MHz minimum TRFC=10
		| SDRAMC_CR_TRP(3)       // Command period (PRE to ACT) 15 ns min. If SDCK=143MHz minimum TRP=3
		| SDRAMC_CR_TRCD(3)      // Active Command to read/Write Command delay 15ns. If SDCK=143MHz minimum TRCD=3
		| SDRAMC_CR_TRAS(7)      // Command period (ACT to PRE)  42ns min. If SDCK=143MHz minimum TRAS=7
		| SDRAMC_CR_TXSR(11U);   // Exit self-refresh to active time  70ns Min. If SDCK=143MHz minimum TXSR=11
#elif (BOARD == SAME70_XPLAINED)
	SDRAMC->SDRAMC_CR =
		SDRAMC_CR_NC_COL8      | /* 8 column bits. */
		SDRAMC_CR_NR_ROW11     | /* 11 row bits    (2K). */
		SDRAMC_CR_CAS_LATENCY2 | /* CAS Latency 2. */
		SDRAMC_CR_NB_BANK2     | /* SDRAM 2 bank. */
		SDRAMC_CR_DBW          | /* Data bus width 16 bits. */
		SDRAMC_CR_TWR(5)       | /* Write Recovery Delay. */
		SDRAMC_CR_TRC_TRFC(13) | /* Row Cycle Delay and Row Refresh Cycle. */
		SDRAMC_CR_TRP(5)       | /* Row Precharge Delay. */
		SDRAMC_CR_TRCD(5)      | /* Row to Column Delay. */
		SDRAMC_CR_TRAS(9)      | /* Active to Precharge Delay. */
		SDRAMC_CR_TXSR(15U)    ;   /* Exit from Self Refresh to Active Delay. */
#else
#error "Non valid SRAM defined"
#endif

	/* Step 2. */

	/* For low-power SDRAM, Temperature-Compensated Self Refresh (TCSR),
	 * Drive Strength (DS) and Partial Array Self Refresh (PASR) must be set
	 * in the Low-power Register. */
	SDRAMC->SDRAMC_LPR = 0;

	/* Step 3. */
	/* Program the memory device type into the Memory Device Register */
	SDRAMC->SDRAMC_MDR = SDRAMC_MDR_MD_SDRAM;

	/* Step 4. */

	/* A minimum pause of 200 µs is provided to precede any signal toggle.
	   (6 core cycles per iteration) */
	for (i = 0; i < (( ul_clk / 1000000) * 200 / 6); i++) {
		;
	}

	/* Step 5. */

	/* A NOP command is issued to the SDR-SDRAM. Program NOP command into
	 * Mode Register, and the application must set Mode to 1 in the Mode
	 * Register. Perform a write access to any SDR-SDRAM address to
	 * acknowledge this command. Now the clock which drives SDR-SDRAM
	 * device is enabled. */
	SDRAMC->SDRAMC_MR = SDRAMC_MR_MODE_NOP;
	*pSdram = 0x0;
	for (i = 0; i < 100000; i++) {
		;
	}

	/* Step 6. */

	/* An all banks precharge command is issued to the SDR-SDRAM. Program
	 * all banks precharge command into Mode Register, and the application
	 * must set Mode to 2 in the Mode Register. Perform a write access to
	 * any SDRSDRAM address to acknowledge this command. */
	SDRAMC->SDRAMC_MR = SDRAMC_MR_MODE_ALLBANKS_PRECHARGE;
	*pSdram = 0x0;

	/* Add some delays after precharge */
	for (i = 0; i < ((ul_clk / 1000000) * 200 / 6); i++) {
		;
	}

	/* Step 7. */

	/* Eight auto-refresh (CBR) cycles are provided. Program the auto
	 * refresh command (CBR) into Mode Register, and the application
	 * must set Mode to 4 in the Mode Register. Once in the idle state,
	 * eight AUTO REFRESH cycles must be performed. */
	for (i = 0; i < 8; i++) {
		SDRAMC->SDRAMC_MR = SDRAMC_MR_MODE_AUTO_REFRESH;
		*pSdram = 0;
	}
	for (i = 0; i < 100000; i++);

	/* Step 8. */

	/* A Mode Register Set (MRS) cycle is issued to program the parameters
	* of the SDRAM devices, in particular CAS latency and burst length. */
	SDRAMC->SDRAMC_MR = SDRAMC_MR_MODE_LOAD_MODEREG;
	*pSdram = 0;
	for (i = 0; i < 100000; i++);

	/* Step 9. */

	/* For low-power SDR-SDRAM initialization, an Extended Mode Register Set
	 * (EMRS) cycle is issued to program the SDR-SDRAM parameters (TCSR,
	 * PASR, DS). The write address must be chosen so that BA[1] is set to
	 * 1 and BA[0] is set to 0. */
	/* No Need*/

	/* Step 10. */

	/* The application must go into Normal Mode, setting Mode to 0 in the
	 * Mode Register and perform a write access at any location in the\
	 * SDRAM to acknowledge this command. */
	SDRAMC->SDRAMC_MR = SDRAMC_MR_MODE_NORMAL;
	*pSdram = 0x0;
	for (i = 0; i < 100000; i++);

	/* Step 11. */

	/* Write the refresh rate into the count field in the SDRAMC Refresh
	 * Timer register. Set Refresh timer to 15.625 us. */

	/* For IS42S16400J, 4096 refresh cycle every 64ms, every 15.625 . SDRAM_TR= ((64 x 10(^-3))/4096) xSDCK(MHz) */
#if (BOARD == PL360BN)
	SDRAMC->SDRAMC_TR = 2343;
#elif (BOARD == SAME70_XPLAINED)
	SDRAMC->SDRAMC_TR = 1562;
#endif
	SDRAMC->SDRAMC_CFR1 |= SDRAMC_CFR1_UNAL;        //Enable of unaligned addressing mode

        /**pSdram = 0;
	for (i = 0; i < 100000; i++);*/

	/* End of Initialization */

	/* Erase SDRAM. SDRAM is not available at early stages of the
	 * initialiation, we do it now. */
	uint32_t *p_sdram = (uint32_t *)BOARD_SDRAM_ADDR;

	p_sdram = (uint32_t *)BOARD_SDRAM_ADDR;
	while ((uint32_t)p_sdram < (uint32_t)(BOARD_SDRAM_ADDR + (BOARD_SDRAM_SIZE-1))) {
		*p_sdram =0;
		p_sdram++;
		__DMB();
	}
}

#endif

/**
 * \brief Configure the hardware.
 */
static void _prv_setup_hardware(void)
{
#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE
	status_code_t status;
#endif

#if !PIC32CX
	/* ASF function to setup clocking. */
	sysclk_init();
#else
	uint32_t rst_cause;

	/* Configure PMC to avoid Reset by WDT */
	rstc_disable_pmc_reset_on_watchdog_0_reset(RSTC);
	rstc_disable_pmc_reset_on_software_reset(RSTC);

	rst_cause = rstc_get_reset_cause(RSTC);

	/* Initialize the PIC32CX system */
	if ((rst_cause == RSTC_SR_RSTTYP_WDT0_RST) ||
        (rst_cause == RSTC_SR_RSTTYP_SOFT_RST)) {
		/* Restart only Core0 system */
		sysclk_restart_core0();
	} else {
		/* Initialize full clock system */
		sysclk_init();
	}
#endif

	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_SetPriorityGrouping(__NVIC_PRIO_BITS);

	/* Atmel library function to setup for the evaluation kit being used. */
	board_init();

#if (BOARD == PL360BN)
        /* Configure external 32khz clock */
        pmc_switch_sclk_to_32kxtal(0);
#endif

#ifdef CONF_BOARD_SDRAMC
	/* Enable Sleep manager */
	sleepmgr_init();

	/* Initialize SDRAM controller */
	_prv_setup_sdram();

#endif

	/* configure supply monitor */
#if !PIC32CX
	hal_setup_supply_monitor(CONTINUOUS_MONITORING, THRESHOLD_3V04);
#else
	hal_setup_supply_monitor(CONTINUOUS_MONITORING, SUPC_SM_THRESHOLD_2v68);
#endif


#if ((BOARD == PL360BN) || (BOARD == SAME70_XPLAINED))
	/* Initialize flash: 6 wait states for flash writing. */
	flash_init(FLASH_ACCESS_MODE_128, (6U));
#else
#if !PIC32CX
	/* Initialize flash wait states for writing. */
	flash_init(FLASH_ACCESS_MODE_128, CHIP_FLASH_WRITE_WAIT_STATE);
#endif
#endif

#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE
#if (BOARD == ATPL360AMB) || (BOARD == ATPL360MB)
	/* Initialize the c0216CiZ LCD glass component. */
	status = (status_code_t)c0216CiZ_init();
	if (status != STATUS_OK) {
		puts("-- LCD Initialization fails! --\r\n");
		while (1) {
		}
	}

	char puc_app[16];
	c0216CiZ_set_cursor(C0216CiZ_LINE_UP, 0);
	c0216CiZ_show((const char *)"Base Node");
	sprintf(puc_app, "PRIME Modem %d", PRIME_APP_VERSION);
	c0216CiZ_set_cursor(C0216CiZ_LINE_DOWN, 0);
	c0216CiZ_show((const char *)puc_app);
#elif (BOARD == PIC32CXMTSH_DB) || (BOARD == PIC32CXMTC_DB)
	/* Initialize the CL010 LCD glass component. */
	status = cl010_init();
	if (status != STATUS_OK) {
		puts("-- LCD Initialization fails! --\r\n");
		while (1) {
		}
	}

	cl010_clear_all();
	cl010_show_icon(CL010_ICON_MICROCHIP);
	/* PRIME Version, PRIME Application, PHY layer */
	cl010_show_numeric_string(CL010_LINE_DOWN, (const uint8_t *)"1450360");
	cl010_show_icon(CL010_ICON_DOT_4);
	cl010_show_icon(CL010_ICON_DOT_5);
	cl010_show_numeric_string(CL010_LINE_UP, (const uint8_t *)"80000000");
	cl010_show_icon(CL010_ICON_COMM_SIGNAL_LOW);
	cl010_show_icon(CL010_ICON_COMM_SIGNAL_MED);
	cl010_show_icon(CL010_ICON_COMM_SIGNAL_HIG);
#endif
#endif
}

#ifdef CONF_BOARD_UART_CONSOLE
/**
 * \brief Configure UART console.
 */
static void _configure_dbg_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONF_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}
#endif

#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE
#if (BOARD != ATPL360AMB) && (BOARD != ATPL360MB)

/**
 * \brief Function to blink a symbol or led.
 * \note Please see conf_app_example file in order to configure the signalling.
 *
 * \param icon_com  Pixel coordinate - COMx - of the icon.
 * \param icon_seg  Pixel coordinate - SEGy - of the icon.
 * \param status    Blink status
 *
 * \return blink status
 */
static uint8_t _blink_symbol(uint8_t icon_com, uint8_t icon_seg, uint8_t status)
{
	if (!status) {
#if (BOARD == PIC32CXMTSH_DB)  || (BOARD == PIC32CXMTC_DB)
		cl010_show_icon(icon_com, icon_seg);
#endif
		return true;
	} else {
#if (BOARD == PIC32CXMTSH_DB)  || (BOARD == PIC32CXMTC_DB)
		cl010_clear_icon(icon_com, icon_seg);
#endif
		return false;
	}
}

#endif
#endif

/**
 * \brief Task to update system status signalling
 *
 * \param pxTimer    Timer
 */
static void _app_signalling(xTimerHandle pxTimer)
{
	UNUSED(pxTimer);
#if (BOARD == PL360BN) || (BOARD == ATPL360AMB) || (BOARD == ATPL360MB)  || (BOARD == SAME70_XPLAINED) || (BOARD == PL360G55CF_EK) || (BOARD == PIC32CXMTG_EK)
	LED_Toggle(LED0);
#endif
#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE
#if (BOARD == PIC32CXMTSH_DB)  || (BOARD == PIC32CXMTC_DB)
	uc_blink_status = _blink_symbol(CL010_ICON_PHASE_1, uc_blink_status);
#endif
#endif
}

/**
 * \brief Task to PLC activity signalling
 *
 * \param pxTimer    Timer
 */
static void _PLC_activity(xTimerHandle pxTimer)
{
	UNUSED(pxTimer);
#if (BOARD == PL360BN) || (BOARD == ATPL360AMB) || (BOARD == ATPL360MB) || (BOARD == PL360G55CF_EK) || (BOARD == PIC32CXMTG_EK)
	LED_Off(LED1);
#endif
#if (BOARD == PIC32CXMTSH_DB)  || (BOARD == PIC32CXMTC_DB)
	cl010_clear_icon(CL010_ICON_PHASE_1);
#endif
}

#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE
#if (BOARD != ATPL360AMB) && (BOARD != ATPL360MB)

/**
 * \brief Task to PLC activity signalling
 *
 * \param pxTimer    Timer
 */
static void _Data_activity(xTimerHandle pxTimer)
{
	UNUSED(pxTimer);
#if (BOARD == PIC32CXMTSH_DB)  || (BOARD == PIC32CXMTC_DB)
	cl010_clear_icon(CL010_ICON_PHASE_2);
	cl010_clear_icon(CL010_ICON_PHASE_3);
#endif
}

#endif
#endif

/**
 * \brief Function to blink a led with PLC TX activity
 *
 */
static void _blink_plc_tx_activity_led(void)
{
	xTimerStart(xPLCActivityTimer, PLC_ACTIVITY_BLOCK_TIME);
#if (BOARD == PL360BN) || (BOARD == ATPL360AMB) || (BOARD == ATPL360MB) || (BOARD == PL360G55CF_EK) || (BOARD == PIC32CXMTG_EK)
	LED_On(LED1);
#endif
#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE
#if (BOARD == PIC32CXMTSH_DB)  || (BOARD == PIC32CXMTC_DB)
	xTimerStart(xDataActivityTimer, DATA_ACTIVITY_BLOCK_TIME);
	cl010_show_icon(CL010_ICON_PHASE_2);
#endif
#endif
}

/**
 * \brief Function to blink a led with PLC RX activity
 *
 */
static void _blink_plc_rx_activity_led(void)
{
	xTimerStart(xPLCActivityTimer, PLC_ACTIVITY_BLOCK_TIME);
#if (BOARD == PL360BN) || (BOARD == ATPL360AMB) || (BOARD == ATPL360MB) || (BOARD == PL360G55CF_EK) || (BOARD == PIC32CXMTG_EK)
	LED_On(LED1);
#endif
#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE
#if (BOARD != ATPL360AMB) && (BOARD != ATPL360MB)
	xTimerStart(xDataActivityTimer, DATA_ACTIVITY_BLOCK_TIME);
#if (BOARD == PIC32CXMTSH_DB)  || (BOARD == PIC32CXMTC_DB)
	cl010_show_icon(CL010_ICON_PHASE_3);
#endif
#endif
#endif
}

void dummy_user_app(void)
{
}

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName );

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	(void)pcTaskName;
	(void)pxTask;

	/* Run time stack overflow checking is performed if
	 * configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	 * function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for (;;) {
		while (1) {
		}
	}
}

/**
 * \brief Main code entry point.
 */
int main(void)
{
	/* Prepare the hardware */
	_prv_setup_hardware();

	/* Set up watchdog */
	hal_watchdog_setup(WATCHDOG_TIME);

	/* Configure console */
#if (BOARD != SAM4E_XPLAINED_PRO)
#ifdef CONF_BOARD_UART_CONSOLE
	_configure_dbg_console();
	puts(STRING_HEADER);
	printf("-- Application Layer Version ID:%d\r\n", (int)PRIME_APP_VERSION);
#endif
#else
#ifdef PIN_EBI_NLB
	/* / * Configure LB, enable SRAM access * / */
	pio_configure_pin(PIN_EBI_NLB, PIN_EBI_NLB_FLAGS);
#endif
#endif

	/* Create timer to example of app signalling */
	xAppSignalTimer = xTimerCreate((const signed char *const)"Signal T", APP_SIGNAL_TIMER_RATE, pdTRUE, NULL, _app_signalling);
	configASSERT(xAppSignalTimer);
	xTimerStart(xAppSignalTimer, APP_SIGNAL_BLOCK_TIME);

	/* Create timer to control PLC activity LED */
	xPLCActivityTimer = xTimerCreate((const signed char *const)"PLC Activity T", PLC_ACTIVITY_TIMER, pdFALSE, NULL, _PLC_activity);
#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE
#if (BOARD != ATPL360AMB) && (BOARD != ATPL360MB)
	/* Create timer to control DATA activity  */
	xDataActivityTimer = xTimerCreate((const signed char *const)"Data Activity T", DATA_ACTIVITY_TIMER, pdFALSE, NULL, _Data_activity);
#endif
#endif
	configASSERT(xPLCActivityTimer);

	/* Init Prime Stack OSS */
	vPrimeStackInitTask();

	/* Init HAL PLC signalling */
	hal_plc_set_tx_signalling_handler(_blink_plc_tx_activity_led);
	hal_plc_set_rx_signalling_handler(_blink_plc_rx_activity_led);

	/* Configure APP from user */
	vUserAppInitTask();

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following
	 * line will never be reached. If the following line does execute, then
	 * there was insufficient FreeRTOS heap memory available for the idle
	 * and/or
	 * timer tasks to be created. See the memory management section on the
	 * FreeRTOS web site for more details. */
	for (;;) {
	}
}
