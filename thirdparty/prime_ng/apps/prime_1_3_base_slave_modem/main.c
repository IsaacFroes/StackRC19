/**
 * \file
 *
 * \brief MAIN : Service Node modem application for PRIME_NG 1.3 (uC)
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
 *  \mainpage ATMEL PRIME 1.3 for Base Slave Node
 *
 *  \section Purpose
 *   this application implement the base slave functionlity for ticket 65 using
 *   a microcontroller based implementation.
 *
 *  \section Requirements
 *
 *  This package should be used with boards, on which there is
 * dedicated PLC hardware.
 *
 *  \section Description
 *
 *  This application will configure the PRIME stack and its serial interface to
 * use PHY, MAC and IEC_432 layers as Service Node. It will also implement a DLMS
 * application emulator that will interact with a base node with DLMS capabilities.
 *
 *  \section Usage
 *
 *  -# Build the program and download it into the evaluation board. Please
 *     refer to the
 *     <a href="http://www.atmel.com/dyn/resources/prod_documents/6421B.pdf">
 *     SAM-BA User Guide</a> or the
 *     <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *     GNU-Based Software Development</a>
 *     application note depending on the solutions that users choose.
 *     \endcode
 *  -# The application will start PRIME standard as Service Node mode.
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
#define STRING_HEADER "-- ATMEL PRIME_NG Application v1.3 for Base Slave Node[ucontroller]--\r\n" \
	"-- "BOARD_NAME " --\r\n" \
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

/** Enable swapping of PRIME location */
# define FU_ENABLE_SWAP 	0XFE45EC48
/* time to show messages in display */
# define SHOW_MESSAGE_TIME   (500)

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
/** DATA activity signaling timer */
static xTimerHandle xDataActivityTimer = NULL;

/* icon blink status */
static uint8_t uc_blink_status;
#endif

/** WDT configuration (time in microseconds) */
#define WATCHDOG_TIME                     5000000

/**
 * \brief Configure the external SDRAM IS42S16400J_7B2LI.
 */
#if ((BOARD == PL360BN) && defined(CONF_BOARD_SDRAMC))
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

	/* Step 2. */

	/* For low-power SDRAM, Temperature-Compensated Self Refresh (TCSR),
	   Drive Strength (DS) and Partial Array Self Refresh (PASR) must be set
	   in the Low-power Register. */
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
	   Mode Register, and the application must set Mode to 1 in the Mode
	   Register. Perform a write access to any SDR-SDRAM address to
	   acknowledge this command. Now the clock which drives SDR-SDRAM
	   device is enabled. */
	SDRAMC->SDRAMC_MR = SDRAMC_MR_MODE_NOP;
	*pSdram = 0x0;
	for (i = 0; i < 100000; i++){
		;
	}

	/* Step 6. */

	/* An all banks precharge command is issued to the SDR-SDRAM. Program
	   all banks precharge command into Mode Register, and the application
	   must set Mode to 2 in the Mode Register. Perform a write access to
	   any SDRSDRAM address to acknowledge this command. */
	SDRAMC->SDRAMC_MR = SDRAMC_MR_MODE_ALLBANKS_PRECHARGE;
	*pSdram = 0x0;

	/* Add some delays after precharge */
	for (i = 0; i < ((ul_clk / 1000000) * 200 / 6); i++) {
		;
	}

	/* Step 7. */
	/* Eight auto-refresh (CBR) cycles are provided. Program the auto
	   refresh command (CBR) into Mode Register, and the application
	   must set Mode to 4 in the Mode Register. Once in the idle state,
	   eight AUTO REFRESH cycles must be performed. */
	for (i = 0 ; i< 8; i++) {
		SDRAMC->SDRAMC_MR = SDRAMC_MR_MODE_AUTO_REFRESH;
		*pSdram = 0;
	}
	for (i = 0; i < 100000; i++);

	/* Step 8. */
	/* A Mode Register Set (MRS) cycle is issued to program the parameters
	   of the SDRAM devices, in particular CAS latency and burst length. */
	SDRAMC->SDRAMC_MR = SDRAMC_MR_MODE_LOAD_MODEREG;
	*pSdram = 0;
	for (i = 0; i < 100000; i++);

	/* Step 9. */

	/* For low-power SDR-SDRAM initialization, an Extended Mode Register Set
	   (EMRS) cycle is issued to program the SDR-SDRAM parameters (TCSR,
	   PASR, DS). The write address must be chosen so that BA[1] is set to
	   1 and BA[0] is set to 0. */
	/* No Need*/

	/* Step 10. */
	/* The application must go into Normal Mode, setting Mode to 0 in the
	   Mode Register and perform a write access at any location in the\
	   SDRAM to acknowledge this command. */
	SDRAMC->SDRAMC_MR = SDRAMC_MR_MODE_NORMAL;
	*pSdram = 0x0;
	for (i = 0; i < 100000; i++);

	/* Step 11. */

	/* Write the refresh rate into the count field in the SDRAMC Refresh
	   Timer register. Set Refresh timer to 15.625 us. */

	/* For IS42S16400J, 4096 refresh cycle every 64ms, every 15.625 . SDRAM_TR= ((64 x 10(^-3))/4096) xSDCK(MHz) */
        SDRAMC->SDRAMC_TR = 2343;
	SDRAMC->SDRAMC_CFR1 |= SDRAMC_CFR1_UNAL;        //Enable of unaligned addressing mode
	*pSdram = 0;
	for (i = 0; i < 100000; i++);

	/* End of Initialization */

	/* Erase SDRAM. SDRAM is not available at early stages of the
	initialiation, we do it now. */
	uint32_t  *p_sdram = (uint32_t *)BOARD_SDRAM_ADDR;

	p_sdram = (uint32_t *)BOARD_SDRAM_ADDR;
	while ((uint32_t)p_sdram < (uint32_t)(BOARD_SDRAM_ADDR + BOARD_SDRAM_SIZE)) {
		*p_sdram =0;
		p_sdram++;
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

	/* ASF function to setup clocking. */
	sysclk_init();

	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_SetPriorityGrouping(__NVIC_PRIO_BITS);

	/* Atmel library function to setup for the evaluation kit being used. */
	board_init();

#if ((BOARD == PL360BN) && defined(CONF_BOARD_SDRAMC))
    /* Configure external 32khz clock */
    pmc_switch_sclk_to_32kxtal(0);

	/* Disable watchdog */
	wdt_disable( WDT ) ;

    /* Enable Sleep manager */
    sleepmgr_init();

	/* Initialize SDRAM controller */
	_prv_setup_sdram();
#endif

	/* configure supply monitor */
	hal_setup_supply_monitor(CONTINUOUS_MONITORING, THRESHOLD_3V04);


#if (BOARD == PL360BN)
	/* Initialize flash: 6 wait states for flash writing. */
	flash_init(FLASH_ACCESS_MODE_128, (6U));
#else
	#if !PIC32CX
	/* Initialize flash wait states for writing. */
	flash_init(FLASH_ACCESS_MODE_128, CHIP_FLASH_WRITE_WAIT_STATE);
	#endif
#endif
}

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

/**
 * \brief Task to update system status signalling
 *
 * \param pxTimer    Timer
 */
static void _app_signalling(xTimerHandle pxTimer)
{
	UNUSED(pxTimer);
	LED_Toggle(LED0);
}

/**
 * \brief Task to PLC activity signalling
 *
 * \param pxTimer    Timer
 */
static void _PLC_activity(xTimerHandle pxTimer)
{
	UNUSED(pxTimer);
#if (BOARD == PL360BN)
	LED_Off(LED1);
#endif
}

/**
 * \brief Function to blink a led with PLC TX activity
 *
 */
static void _blink_plc_tx_activity_led(void)
{
	xTimerStart(xPLCActivityTimer, PLC_ACTIVITY_BLOCK_TIME);
#if (BOARD == PL360BN)
	LED_On(LED1);
#endif
}

/**
 * \brief Function to blink a led with PLC RX activity
 *
 */
static void _blink_plc_rx_activity_led(void)
{
	xTimerStart(xPLCActivityTimer, PLC_ACTIVITY_BLOCK_TIME);
#if (BOARD == PL360BN)
	LED_On(LED1);
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
	_configure_dbg_console();
	puts(STRING_HEADER);
	printf("-- Application Layer Version ID:%d\r\n",(int) PRIME_APP_VERSION);
#else
#ifdef PIN_EBI_NLB
	///* Configure LB, enable SRAM access */
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
	/* Create timer to control DATA activity  */
	xDataActivityTimer = xTimerCreate((const signed char *const)"Data Activity T", DATA_ACTIVITY_TIMER, pdFALSE, NULL, _Data_activity);
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
