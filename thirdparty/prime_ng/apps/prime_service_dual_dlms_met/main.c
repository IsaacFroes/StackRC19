/**
 * \file
 *
 * \brief MAIN : Service Node Metrology application for PRIMEv1.4 (uC)
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
#include <stdio_serial.h>
#include "app.h"
#include "app_emu.h"
#include "dlms_app.h"
#include "modem.h"
#include "conf_app_example.h"
#include "conf_board.h"
#include "conf_oss.h"
#include "asf.h"
#include "hal.h"
#include "hal_private.h"
#include "metering_task.h"
#include "conf_platform.h"

/* APP DC includes. */
#ifdef ENABLE_DIRECT_CON_EXAMPLE_APP
#include "app_dc.h"
#endif

/**
 *  \mainpage Microchip PRIME Metrology Application for Service Node
 *
 *  \section Purpose
 *
 *  This application provides the service node with DLMS and Metrology capabilities.
 *
 *  \section Requirements
 *
 *  This package should be used in boards with dedicated PLC hardware.
 *
 *  \section Description
 *
 *  This application will configure the PRIME stack and its serial interface to
 * use PHY, MAC and IEC_432 layers as Service Node. It will also implement a DLMS
 * application with Metrology that will interact with a base node with DLMS capabilities.
 *
 *  \section Usage
 *
 *  -# Build the program and download it into the evaluation board.
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
#define STRING_HEADER "-- Microchip PRIME Application DLMS_APP/Metrology for Service Node[uC]--\r\n"	\
	"-- "BOARD_NAME " --\r\n" \
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

/** Enable swapping of FU location */
# define FU_ENABLE_SWAP         0XFE45EC48
/** Enable swapping of stack */
# define STACK_ENABLE_SWAP      0XEF54CE84

/* time to show messages in display */
# define SHOW_MESSAGE_TIME   (500)

/** Pointer to the PRIME stack in internal flash */
uint32_t prime_api;
/** New stack pointer */
static uint32_t sul_new_prime_api;

/** Enable swapping of stack location */
static uint32_t volatile sul_fu_swap_en;
static uint32_t volatile sul_stack_swap_en;

/** PRIME regions configuration */
x_fu_region_cfg_t sx_prime_reg[PRIME_NUM_REGIONS];

/** Define HAL API interface */
extern const hal_api_t hal_api;

#define ID_TC_1MS               ID_TC3
#define TC_1MS                  TC1
#define TC_1MS_CHN              0
#define TC_1MS_IRQn             TC3_IRQn
#define TC_1MS_Handler          TC3_Handler

#define SIGNALLING_PERIOD_MS    250
#define SIGNALLING_APPEMU       750
#define SIGNALLING_LCD_MET      1000
static bool b_signalling_en;
static uint16_t us_signal_period;

#define APP_SIGNALLING_PERIOD_MS     (10)
static bool b_signalling_app;
static uint16_t us_signal_period_app;

static uint8_t uc_blink_tx;
static uint8_t uc_blink_rx;
#define TXRX_SHOW_TIME               150

#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE
#if (BOARD != ATPL360AMB) && (BOARD != ATPL360MB)
static uint8_t uc_blink_status;
#endif

#ifdef ENABLE_LCD_MET_VALUES
static disp_state_t suc_disp_state;
metering_info_t *spx_info;
static char spuc_disp_buff[16];
static uint16_t us_signal_period_lcd_met;
static bool b_signalling_lcd_met;
#endif

metering_info_t *spx_info;

static void _show_node_state(void);

#endif

/** WDT configuration (time in microseconds) */
#define WATCHDOG_TIME                     5000000

#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE
static void _init_display(void)
{
	status_code_t status;

#if (BOARD == ATPL360AMB) || (BOARD == ATPL360MB)
	/* Initialize the c0216CiZ LCD glass component. */
	status = (status_code_t)c0216CiZ_init();
	if (status != STATUS_OK) {
		puts("-- LCD Initialization fails! --\r\n");
		while (1) {
		}
	}

	char puc_app[16];
	sprintf(puc_app, "SN DLMS Dual %d", PRIME_APP_VERSION);
	c0216CiZ_set_cursor(C0216CiZ_LINE_DOWN, 0);
	c0216CiZ_show((const char *)puc_app);
#endif
}

#endif

/**
 * \brief Interrupt handler for Timer 3
 */
void TC_1MS_Handler(void)
{
	volatile uint32_t ul_dummy;
	/* Clear status bit to acknowledge interrupt */
	ul_dummy = tc_get_status(TC_1MS, TC_1MS_CHN);
	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/* update signalling ms */
	if (!us_signal_period--) {
		if (get_app_mode() == DLMS_APP_MODE) {
			us_signal_period = SIGNALLING_PERIOD_MS;
		} else {
			us_signal_period = SIGNALLING_APPEMU;
		}

		b_signalling_en = true;
	}

	if (!us_signal_period_app--) {
		us_signal_period_app = APP_SIGNALLING_PERIOD_MS;
		b_signalling_app = true;
	}

#ifdef ENABLE_LCD_MET_VALUES
	if (!us_signal_period_lcd_met--) {
		us_signal_period_lcd_met = SIGNALLING_LCD_MET;
		b_signalling_lcd_met = true;
	}
#endif

	metering_app_timers_update();

	if (uc_blink_tx) {
		if (!--uc_blink_tx) {
#if (BOARD == ATPL360AMB) || (BOARD == ATPL360MB)
			LED_Off(LED1);
#endif
		}
	}

	if (uc_blink_rx) {
		if (!--uc_blink_rx) {
#if (BOARD == ATPL360AMB) || (BOARD == ATPL360MB)
			LED_Off(LED1);
#endif
		}
	}
}

/**
 * \brief Init Timer interrupt (1ms)
 */
static void _init_timer_1ms(void)
{
	uint32_t ul_div, ul_tcclks;

	/* Configure PMC */
	pmc_enable_periph_clk(ID_TC_1MS);

	/* MCK = 102400000 -> tcclks = 2 : TCLK3 = MCK/32 = 3200000 = 0.3125us ->
	 * ul_div = 1ms/0.3125us = 3200 */
	ul_tcclks = 2;
	ul_div = 3200;

	tc_init(TC_1MS, TC_1MS_CHN, ul_tcclks | TC_CMR_CPCTRG);

	tc_write_rc(TC_1MS, TC_1MS_CHN, ul_div);

	/* Configure and enable interrupt on RC compare */
	NVIC_SetPriority((IRQn_Type)ID_TC_1MS, 0);
	NVIC_EnableIRQ((IRQn_Type)ID_TC_1MS);
	tc_enable_interrupt(TC_1MS, TC_1MS_CHN, TC_IER_CPCS);

	/** Start the timer. TC1, chanel 0 = TC3 */
	tc_start(TC_1MS, TC_1MS_CHN);
}

/**
 * \brief Configure the hardware.
 */
static void _prv_setup_hardware(void)
{
	/* ASF function to setup clocking. */
	sysclk_init();

	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_SetPriorityGrouping(__NVIC_PRIO_BITS);

#if BOARD == SAMG55_XPLAINED_PRO

	/* Select the crystal oscillator to be the source of the slow clock,
	 * as it provides a more accurate frequency
	 */
#ifdef CONF_BOARD_32K_XTAL
	supc_switch_sclk_to_32kxtal(SUPC, 0);
#endif
#endif

	/* Atmel library function to setup for the evaluation kit being used. */
	board_init();

	/* configure supply monitor */
	hal_setup_supply_monitor(CONTINUOUS_MONITORING, THRESHOLD_3V04);

#if !PIC32CX
	/* Initialize flash: 6 wait states for flash writing. */
	flash_init(FLASH_ACCESS_MODE_128, CHIP_FLASH_WRITE_WAIT_STATE);
#endif

#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE
	_init_display();
#endif
	uc_blink_tx = 0;
	uc_blink_rx = 0;
}

#ifdef CONF_BOARD_UART_CONSOLE
/**
 *  \brief Configure UART console.
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

/**
 * \brief Function to show node status in the display
 * \note Please see conf_app_example file in order to configure the signalling.
 *
 */
static void _show_node_state(void)
{
#if (BOARD == ATPL360AMB) || (BOARD == ATPL360MB)
	c0216CiZ_set_cursor(C0216CiZ_LINE_UP, 0);
#endif

	switch (dlms_app_node_state()) {
	case NODE_UNREGISTERED:
#if (BOARD == ATPL360AMB)  || (BOARD == ATPL360MB)
		c0216CiZ_show((const char *)"SN UNREGISTERED");
#endif
		break;

	case NODE_REGISTERED:
#if (BOARD == ATPL360AMB) || (BOARD == ATPL360MB)
		c0216CiZ_show((const char *)"SN REGISTERED   ");
#endif
		break;

	case NODE_CONNECTED_DLMSEMU:
#if (BOARD == ATPL360AMB) || (BOARD == ATPL360MB)
		c0216CiZ_show((const char *)"CONNECTED 4.32  ");
#endif
		break;
	}
}

#endif

#ifndef ENABLE_LCD_MET_VALUES
/**
 * \brief Task to signalling example.
 *
 */
static void _app_signalling(void)
{
#if (BOARD == ATPL360AMB) || (BOARD == ATPL360MB)
	LED_Toggle(LED0);
#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE
	_show_node_state();
#endif
#endif
}
#endif

/**
 * \brief Initial information in the display
 *
 */
static void _init_display_info(uint32_t ul_prime_ptr)
{
	if (ul_prime_ptr ==  PRIME_MAC13_FLASH_LOCATION) {
#if (BOARD == ATPL360AMB) || (BOARD == ATPL360MB)
		LED_On(LED1);
#endif
#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE
#if (BOARD == ATPL360AMB) || (BOARD == ATPL360MB)
		c0216CiZ_set_cursor(C0216CiZ_LINE_UP, 0);
		c0216CiZ_show((const char *)"STACK13 LOADED ");
#endif
#endif
		delay_ms(SHOW_MESSAGE_TIME);
	} else {        /*PRIME_MAC14_FLASH_LOCATION */
#if (BOARD == ATPL360AMB) || (BOARD == ATPL360MB)
		LED_On(LED1);
#endif
#ifdef EXAMPLE_LCD_SIGNALLING_ENABLE
#if (BOARD == ATPL360AMB) || (BOARD == ATPL360MB)
		c0216CiZ_set_cursor(C0216CiZ_LINE_UP, 0);
		c0216CiZ_show((const char *)"STACK14 LOADED ");
#endif
#endif
		delay_ms(SHOW_MESSAGE_TIME);
	}
}

/**
 * \brief return a valid prime api pointer.
 *
 * \retval valid stack pointer value
 */

static uint32_t _get_prime_ptr(uint8_t* puc_prime_version)
{
	x_prime_mode_info_cfg_t x_board_info;
	uint32_t ul_prime_api;

	hal_get_config_info(CONFIG_TYPE_MODE_PRIME, sizeof(x_board_info), (void *)&x_board_info);

	*puc_prime_version = x_board_info.prime_version;
	switch (x_board_info.prime_version) {
	case PRIME_1_3:
		ul_prime_api = PRIME_MAC13_FLASH_LOCATION;
		break;

	case PRIME_1_4:
		ul_prime_api = PRIME_MAC14_FLASH_LOCATION;
		break;

	default:
		*puc_prime_version = PRIME_1_4;
		/* set default prime pointer value location*/
		ul_prime_api = PRIME_MAC14_FLASH_LOCATION;
	}

	return ul_prime_api;
}

/**
 * \brief Function to blink a led with PLC TX activity
 *
 */
static void _blink_plc_tx_activity_led(void)
{
#if (BOARD == ATPL360AMB) || (BOARD == ATPL360MB)
	LED_On(LED1);
	uc_blink_tx = TXRX_SHOW_TIME;
#endif
}

/**
 * \brief Function to blink a led with PLC RX activity
 *
 */
static void _blink_plc_rx_activity_led(void)
{
#if (BOARD == ATPL360AMB) || (BOARD == ATPL360MB)
	LED_On(LED1);
	uc_blink_rx = TXRX_SHOW_TIME;
#endif
}

/**
 * \brief Swap FU pointer
 *
 */
static void _swap_fu_pointer(void)
{
	uint32_t *ptr_nvic_cpr0;
	uint32_t *ptr_nvic_ser0;
	uint32_t *ptr_nvic_cer0;
	uint32_t ul_tmp_value;

	/* Hold interrupt system */
	ptr_nvic_ser0 = (uint32_t *)NVIC_ISER0;
	ul_tmp_value = *ptr_nvic_ser0;
	ptr_nvic_cer0 = (uint32_t *)NVIC_ICER0;
	ptr_nvic_cpr0 = (uint32_t *)NVIC_ICPR0;
	*ptr_nvic_cer0 = 0xFFFFFFFF;
	*ptr_nvic_cpr0 = 0xFFFFFFFF;

	/* Swap firmware */
	if (hal_fu_swap()) {
		/* Trigger reset to launch bootloader */
		hal_reset_trigger(FU_RESET);
	}

	/* Restore interrupt system */
	*ptr_nvic_ser0 = ul_tmp_value;
}

/**
 * \brief Task to HAL FU results handle example.
 *
 * \param uc_result    FU result
 *
 */
static void _prime_fu_result_handler(hal_fu_result_t *uc_result)
{
	hal_fu_result_t uc_fu_res;

	uc_fu_res = *uc_result;
	switch (uc_fu_res) {
	case HAL_FU_SUCCESS:
		/* Update FU pointer */
		sul_fu_swap_en = FU_ENABLE_SWAP;
		break;

	case HAL_FU_CRC_ERROR:
		/* nothing to do. FU will restart automatically */
		break;

	case HAL_FU_CANCEL:
		/* nothing to do */
		break;

	case HAL_FU_FW_CONFIRM:
		/* nothing to do */
		break;

	case HAL_FU_FW_REVERT:
		/* Revert FU pointer */
		sul_fu_swap_en = FU_ENABLE_SWAP;
		break;

	case HAL_FU_ERROR:
		/* nothing to do */
		break;

	case HAL_FU_SIGNATURE_ERROR:
		/* nothing to do */
		break;

	case HAL_FU_IMAGE_ERROR:
		/* nothing to do */
		break;

	default:
		break;
	}
}

/**
 * \brief Task to handle HAL swap stack request
 *
 * \param uc_traffic   Detected traffic (1 = PRIME_1_3, 2 = PRIME_1_4)
 */
static void _prime_swap_stack_request(uint8_t uc_traffic)
{
	/* Compare current PRIME pointer with detected traffic */
	if (uc_traffic == PRIME_1_4) {
		sul_new_prime_api = PRIME_MAC14_FLASH_LOCATION;
		sul_stack_swap_en = STACK_ENABLE_SWAP;

	} else if (uc_traffic == PRIME_1_3) {
		sul_new_prime_api = PRIME_MAC13_FLASH_LOCATION;
		sul_stack_swap_en = STACK_ENABLE_SWAP;
	}
}

/**
 * \brief Task to HAL swap stack example.
 *
 */
static void _prime_swap_stack(uint8_t uc_prime_version)
{
	uint32_t *ptr_nvic_cpr0;
	uint32_t *ptr_nvic_ser0;
	uint32_t *ptr_nvic_cer0;
	uint32_t ul_tmp_value;

	/* Hold interrupt system */
	ptr_nvic_ser0 = (uint32_t *)NVIC_ISER0;
	ul_tmp_value = *ptr_nvic_ser0;

	/* Clear pending interrupts */
	ptr_nvic_cer0 = (uint32_t *)NVIC_ICER0;
	ptr_nvic_cpr0 = (uint32_t *)NVIC_ICPR0;
	*ptr_nvic_cer0 = 0xFFFFFFFF;
	*ptr_nvic_cpr0 = 0xFFFFFFFF;

	/* Reset PLC */
	hal_plc_reset();

	/* Set new PRIME stack location */
	prime_api = sul_new_prime_api;

	/* Restore interrupt system */
	*ptr_nvic_ser0 = ul_tmp_value;

	/* Initialize PRIME stack */
	prime_init((hal_api_t *)&hal_api);

	/* Configure APP from user to set up callbacks */
	vUserAppInitTask(uc_prime_version);
}

#ifdef ENABLE_LCD_MET_VALUES
static void display_met_value(void)
{
	if (b_signalling_en) {
		b_signalling_en = false;
		LED_Toggle(LED0);
	}

	if (b_signalling_lcd_met) {
		b_signalling_lcd_met = false;

		switch (suc_disp_state) {
		case disp_app:
			_show_node_state();
			suc_disp_state = disp_v;
			break;

		case disp_v:
			c0216CiZ_set_cursor(C0216CiZ_LINE_UP, 0);
			sprintf(spuc_disp_buff, "Vrms = %04u mV ", spx_info->Inst_V_rms_L1);
			c0216CiZ_show((const char *)spuc_disp_buff);
			suc_disp_state = disp_i;
			break;

		case disp_i:
			c0216CiZ_set_cursor(C0216CiZ_LINE_UP, 0);
			sprintf(spuc_disp_buff, "Irms = %04u mA  ", spx_info->Inst_I_rms_L1);
			c0216CiZ_show((const char *)spuc_disp_buff);
			suc_disp_state = disp_p;
			break;

		case disp_p:
			c0216CiZ_set_cursor(C0216CiZ_LINE_UP, 0);
			sprintf(spuc_disp_buff, "Prms = %04u W   ", spx_info->Inst_P_rms_L1 / 10);
			c0216CiZ_show((const char *)spuc_disp_buff);
			suc_disp_state = disp_app;
			break;
		}
	}
}
#endif

/**
 * \brief Main code entry point.
 */
int main(void)
{
	uint8_t uc_prime_version;

	/* Prepare the hardware */
	_prv_setup_hardware();

#if !defined( _PRIME_SIM_)
	/* configure IO_port to enable/ disable  APP EMU mode */
	ioport_set_pin_dir(PIN_APPDLMSEMU_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_level(PIN_APPDLMSEMU_GPIO, IOPORT_PIN_LEVEL_HIGH);
#endif

	/* Set up watchdog */
	hal_watchdog_setup(WATCHDOG_TIME);

#ifdef CONF_BOARD_UART_CONSOLE
	/* Configure console */
	_configure_dbg_console();
	puts(STRING_HEADER);
	printf("-- Application Layer Version ID:%d\r\n", (int)PRIME_APP_VERSION);
#endif

	/* Initialize hal layer */
	hal_init();

	/* Read PRIME ptr address */
	prime_api = _get_prime_ptr(&uc_prime_version);

	/* Initial information in display */
	_init_display_info(prime_api);

	/* Init HAL result callback for stack swap request */
	hal_swap_stack_set_callback(_prime_swap_stack_request);

	/* Init HAL FU result callback */
	hal_fu_result_set_callback(_prime_fu_result_handler);

	/* Configure PRIME image regions */
	sx_prime_reg[0].ul_address = PRIME_IMAGE_FLASH_LOCATION;
	sx_prime_reg[0].ul_size = PRIME_IMAGE_SIZE;

	/* Set which region is being used (needed in case of revert) */
	uint8_t uc_app_to_fu;
	uc_app_to_fu = (uint8_t)gpbr_read(GPBR4);
	if (uc_app_to_fu == PRIME_INVALID_APP) {
		sx_prime_reg[0].in_use = false;
	} else {
		sx_prime_reg[0].in_use = true;
	}

	hal_fu_config_regions(PRIME_NUM_REGIONS, sx_prime_reg);

	/* Init swap flags */
	sul_fu_swap_en = 0;
	sul_stack_swap_en = 0;

	/* Initialize PRIME stack */
	prime_init((hal_api_t *)&hal_api);

	/* Init signalling timer */
	_init_timer_1ms();

	/* init signalling tasks counters */
	us_signal_period = SIGNALLING_APPEMU;
	b_signalling_en = false;
	us_signal_period_app = APP_SIGNALLING_PERIOD_MS;
	b_signalling_app = false;
	us_signal_period_app = APP_SIGNALLING_PERIOD_MS;
	b_signalling_app = false;
#ifdef ENABLE_LCD_MET_VALUES
	us_signal_period_lcd_met = SIGNALLING_LCD_MET;
	b_signalling_lcd_met = false;
#endif

	/* Init HAL PLC signalling */
	hal_plc_set_tx_signalling_handler(_blink_plc_tx_activity_led);
	hal_plc_set_rx_signalling_handler(_blink_plc_rx_activity_led);

	/* Init MODEM application */
	modem_init();

	/* Configure APP from user */
	vUserAppInitTask(uc_prime_version);

	/* Init Metrology application */
	metering_app_init(&spx_info);

	while (1) {
		/* Restart watchdog */
		wdt_restart(WDT);

		/* Process HAL layer */
		hal_process();

		/* Process PRIME stack */
		prime_process();

		/* Process Metrology application */
		metering_app_process();

		/* Process MODEM application */
		modem_process();

		if (b_signalling_app) {
			b_signalling_app = false;

			if (get_app_mode() == APP_EMU_MODE) {
				/* Process APP EMU application */
				app_emu_process();
			}

			/* Process DLMS application */
			dlms_app_process();

#ifdef ENABLE_DIRECT_CON_EXAMPLE_APP
			/* Process APP EMU application */
			app_dc_process();
#endif
		}

#ifdef ENABLE_LCD_MET_VALUES
		display_met_value();
#else
		if (b_signalling_en) {
			b_signalling_en = false;
			_app_signalling();
		}
#endif

		/* Check if FU location must be swapped */
		if (sul_fu_swap_en == FU_ENABLE_SWAP) {
			sul_fu_swap_en = 0;
			_swap_fu_pointer();
		}

		/* Check if stack must be swapped */
		if (sul_stack_swap_en == STACK_ENABLE_SWAP) {
			sul_stack_swap_en = 0;
			_prime_swap_stack(uc_prime_version);
		}
	}
}
