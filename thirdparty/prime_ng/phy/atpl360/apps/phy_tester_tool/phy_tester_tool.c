/**
 * \file
 *
 * \brief PHY_TESTER_TOOL : ATMEL PLC Phy Tester Example
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

/**
 *  \mainpage ATMEL PLC Phy Tester Example
 *
 *  \section Purpose
 *
 *  The Phy Tester Tool example demonstrates how to use the PRIME PHY layer on
 * PLC boards.
 *
 *  \section Requirements
 *
 *  This package should be used with any PLC board on which there is PLC
 * hardware dedicated.
 *
 *  \section Description
 *
 *  This application will configure the PRIME PHY and its serial interface to
 * communicate with
 * ATMEL PLC Phy Tester Tool and test PLC transmission/reception processes.
 *
 *  \section Usage
 *
 *  The tool is ready for set up the device configuration and perform some
 * communications test.
 *
 */

/* Boards includes. */
#include "board.h"

/* Library includes. */
#include "asf.h"
#include "hal_private.h"
#include "serial_if.h"

/* Config includes */
#include "conf_app_example.h"

/* ATPL360 component */
static atpl360_descriptor_t sx_atpl360_desc;
static uint8_t suc_err_unexpected;
static uint8_t suc_err_critical;
static uint8_t suc_err_reset;
static uint8_t suc_err_none;
static uint8_t sb_exception_pend;

/* Define Phy Tester Tool serial interface */
#define SERIAL_ATPL230_USI_PROTOCOL                0x23

/* Define Time to led swapping */
#define COUNT_MS_SWAP_LED                          500

/* #define PL360_BIN_ADDR_FIXED */
#ifdef PL360_BIN_ADDR_FIXED
#  define PL360_BIN_ADDR  0x010E0000
#  define PL360_BIN_SIZE  (72 * 1024)
#endif

static uint32_t ul_count_ms = COUNT_MS_SWAP_LED;
static bool b_led_swap = false;
#if (BOARD == PIC32CXMTSH_DB)
#ifdef CONF_BOARD_LCD_EN
static bool b_led_swap_on = false;
#endif
#endif

#if (!PIC32CX)
#define ID_TC_1MS               ID_TC3
#define TC_1MS                  TC1
#define TC_1MS_CHN              0
#define TC_1MS_IRQn             TC3_IRQn
#define TC_1MS_Handler          TC3_Handler
#else
#define ID_TC_1MS               ID_TC1_CHANNEL0
#define TC_1MS                  TC1
#define TC_1MS_CHN              0
#define TC_1MS_IRQn             TC1_CHANNEL0_IRQn
#define TC_1MS_Handler          TC1_CHANNEL0_Handler
#endif

#ifdef CONF_BOARD_UART_CONSOLE
#define PRINT_DBG(x)            printf x
#else
#define PRINT_DBG(x)
#endif

#ifdef CONF_BOARD_UART_CONSOLE
#define STRING_EOL    "\r"
#define STRING_HEADER "-- Microchip PLC Phy Tester Tool Application --\r\n" \
	"-- "BOARD_NAME " --\r\n" \
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL
#endif

/* Static var to store last channel configuration */
static uint8_t suc_prime_channel;
static uint8_t suc_max_channel;

/** @brief	Interrupt handler for Timer 3
 *
 * Handler for Timer 3 */
void TC_1MS_Handler(void)
{
	/* Clear status bit to acknowledge interrupt */
	tc_get_status(TC_1MS, TC_1MS_CHN);

	/* update count ms */
	if (!ul_count_ms--) {
		ul_count_ms = COUNT_MS_SWAP_LED;
		b_led_swap = true;
	}
}

/** @brief	Init Timer interrupt (1ms)
 *
 * Initialize 1mSec timer 3 interrupt */
static void initTimer1ms(void)
{
	uint32_t ul_div, ul_tcclks;

	/* Configure PMC */
	pmc_enable_periph_clk(ID_TC_1MS);

	/* MCK = 120000000 -> tcclks = 2 : TCLK3 = MCK/32 = 3750000 = 0.266us ->
	 * ul_div = 1ms/0.2666us = 3750 */
	ul_tcclks = 2;
	ul_div = 3750;
#if (!PIC32CX)
	tc_init(TC_1MS, TC_1MS_CHN, ul_tcclks | TC_CMR_CPCTRG);
#else
	tc_init(TC_1MS, TC_1MS_CHN, ul_tcclks | TC_CMR_CPCTRG, 0);
#endif

	tc_write_rc(TC_1MS, TC_1MS_CHN, ul_div);

	/* Configure and enable interrupt on RC compare */
	NVIC_SetPriority((IRQn_Type)ID_TC_1MS, 0);
	NVIC_EnableIRQ((IRQn_Type)ID_TC_1MS);
	tc_enable_interrupt(TC_1MS, TC_1MS_CHN, TC_IER_CPCS);

	/** Start the timer. TC1, channel 0 = TC3 */
	tc_start(TC_1MS, TC_1MS_CHN);
}

#ifdef CONF_BOARD_UART_CONSOLE

/**
 *  Configure UART console.
 */
/* [main_console_configure] */
static void configure_dbg_console(void)
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

/**
 * \brief Configure the hardware.
 */
static void prvSetupHardware(void)
{
	/* ASF function to setup clocking. */
	sysclk_init();

	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_SetPriorityGrouping(__NVIC_PRIO_BITS);

	/* Atmel library function to setup for the evaluation kit being used. */
	board_init();

	/* Init HAL */
	hal_init();
}

/**
 * \brief Handler to receive data from ATPL360.
 */
static void _handler_serial_atpl360_event(uint8_t *px_serial_data, uint16_t us_len)
{
	x_usi_serial_cmd_params_t x_usi_msg;

	x_usi_msg.uc_protocol_type = PROTOCOL_PHY_TESTER;
	x_usi_msg.ptr_buf = px_serial_data;
	x_usi_msg.us_len = us_len;
	hal_usi_send_cmd(&x_usi_msg);
}

/**
 * \brief Handler to receive data from APP.
 */
static bool _handler_serial_app_event(uint8_t *px_serial_data, uint16_t us_len)
{
	uint8_t uc_serial_if_cmd;

	sx_atpl360_desc.send_addons_cmd(px_serial_data, us_len);

	/* Check if the command is a channel change to reconfigure coupling parameters. */
	uc_serial_if_cmd = *px_serial_data++;
	if (uc_serial_if_cmd == SERIAL_IF_PHY_COMMAND_SET_CFG) {
		/* Set config command */
		uint16_t us_id;
		uint8_t uc_id_len;
		uint8_t uc_value;

		us_id = ((uint16_t)*px_serial_data++) << 8;
		us_id += (uint16_t)*px_serial_data++;
		uc_id_len = *px_serial_data++;

		if ((us_id == (ATPL360_REG_MASK | ATPL360_REG_CHANNEL_CFG)) && (uc_id_len == 1)) {
			/* Set channel command */
			uc_value = *px_serial_data++;

			if ((uc_value != suc_prime_channel) && (uc_value >= SINGLE_CHN_1) && (uc_value <= suc_max_channel)) {
				/* Channel has changed: Configure Coupling and TX parameters depending on configured channel */
				suc_prime_channel = uc_value;
				pl360_prime_coup_tx_config(&sx_atpl360_desc, suc_prime_channel);
			}
		}
	}

	return true;
}

/**
 * \brief Set PL360 configuration
 */
static void _set_pl360_configuration(void)
{
	uint8_t uc_value;

	/* Restore/Initialize PRIME channel(s) */
	sx_atpl360_desc.set_config(ATPL360_REG_CHANNEL_CFG, &suc_prime_channel, 1);
	pl360_prime_coup_tx_config(&sx_atpl360_desc, suc_prime_channel);

	/* Only for PHY Test purposes: Disable AUTO mode and set VLO behavior by default in order to maximize signal level in anycase */
	uc_value = 0;
	sx_atpl360_desc.set_config(ATPL360_REG_CFG_AUTODETECT_IMPEDANCE, &uc_value, 1);
	uc_value = 2;
	sx_atpl360_desc.set_config(ATPL360_REG_CFG_IMPEDANCE, &uc_value, 1);
}

/**
 * \brief Handler to phy exceptions.
 */
static void _handler_exception_event(atpl360_exception_t exception)
{
	switch (exception) {
	case ATPL360_EXCEPTION_UNEXPECTED_SPI_STATUS:
		suc_err_unexpected++;
		break;

	case ATPL360_EXCEPTION_SPI_CRITICAL_ERROR:
		suc_err_critical++;
		break;

	case ATPL360_EXCEPTION_RESET:
		suc_err_reset++;
		break;

	default:
		suc_err_none++;
	}

	sb_exception_pend = true;
}

/**
 * \brief Get PL360 binary addressing.
 *
 * \param pul_address   Pointer to store the initial address of PL360 binary data
 *
 * \return Size of PL360 binary file
 */
static uint32_t _get_pl360_bin_addressing(uint32_t *pul_address)
{
	uint32_t ul_bin_addr;
	uint8_t *puc_bin_start;
	uint8_t *puc_bin_end;

#if defined (__CC_ARM)
	extern uint8_t atpl_bin_start[];
	extern uint8_t atpl_bin_end[];
	ul_bin_addr = (int)(atpl_bin_start - 1);
	puc_bin_start = atpl_bin_start - 1;
	puc_bin_end = atpl_bin_end;
#elif defined (__GNUC__)
	extern uint8_t atpl_bin_start;
	extern uint8_t atpl_bin_end;
	ul_bin_addr = (int)&atpl_bin_start;
	puc_bin_start = (int)&atpl_bin_start;
	puc_bin_end = (int)&atpl_bin_end;
#elif defined (__ICCARM__)
	#pragma section = "P_atpl_bin"
	extern uint8_t atpl_bin;
	ul_bin_addr = (int)&atpl_bin;
	puc_bin_start = __section_begin("P_atpl_bin");
	puc_bin_end = __section_end("P_atpl_bin");
#else
	#error This compiler is not supported for now.
#endif

	*pul_address = ul_bin_addr;
	return ((uint32_t)puc_bin_end - (uint32_t)puc_bin_start);
}

/**
 * \brief Main code entry point.
 */
int main( void )
{
	uint32_t ul_bin_addr;
	uint32_t ul_bin_size;
#ifdef CONF_BOARD_LCD_EN
	status_code_t status;
#endif

	atpl360_dev_callbacks_t x_atpl360_cbs;
	atpl360_hal_wrapper_t x_atpl360_hal_wrp;
	uint8_t uc_ret;
	uint8_t uc_max_num_channels;

	ul_count_ms = 500; /* count ms to blink led */

	/* Prepare the hardware */
	prvSetupHardware();

#ifdef CONF_BOARD_UART_CONSOLE
	/* UART debug */
	configure_dbg_console();
	puts(STRING_HEADER);
#endif

#ifdef CONF_BOARD_LCD_EN
#if BOARD == ATPL360ASB
	/* Initialize the vim878 LCD glass component. */
	status = vim878_init();
	if (status != STATUS_OK) {
		puts("-- LCD Initialization fails! --\r\n");
		while (1) {
		}
	}

	vim878_set_contrast(1);
	vim878_clear_all();
	vim878_show_text((const uint8_t *)"PHYTST");
#elif (BOARD == ATPL360AMB || BOARD == ATPL360MB)
	status = (status_code_t)c0216CiZ_init();
	if (status != STATUS_OK) {
		puts("-- LCD Initialization fails! --\r\n");
		while (1) {
		}
	}

	c0216CiZ_set_cursor(C0216CiZ_LINE_UP, 0);
	c0216CiZ_show((const char *)"ATPL360AMB PRIME");
	c0216CiZ_set_cursor(C0216CiZ_LINE_DOWN, 0);
	c0216CiZ_show((const char *)"Phy tester tool");
#elif BOARD == PIC32CXMTSH_DB
	/* Initialize the CL010 LCD glass component. */
	status = cl010_init();
	if (status != STATUS_OK) {
		puts("-- LCD Initialization fails! --\r\n");
		while (1) {
		}
	}

	cl010_clear_all();
	cl010_show_icon(CL010_ICON_MICROCHIP);
	cl010_show_numeric_string(CL010_LINE_DOWN, (const uint8_t *)"0000360");
	cl010_show_icon(CL010_ICON_DOT_5);
	cl010_show_icon(CL010_ICON_P_PLUS);
	cl010_show_icon(CL010_ICON_P_MINUS);
	b_led_swap_on = false;
#else
#error ERROR in board definition
#endif
#endif

	/* Init process timers */
	initTimer1ms();

	/* Init ATPL360 */
	x_atpl360_hal_wrp.plc_init = hal_plc_init;
	x_atpl360_hal_wrp.plc_reset = hal_plc_reset;
	x_atpl360_hal_wrp.plc_set_stby_mode = hal_plc_set_stby_mode;
	x_atpl360_hal_wrp.plc_set_handler = hal_plc_set_handler;
	x_atpl360_hal_wrp.plc_send_boot_cmd = hal_plc_send_boot_cmd;
	x_atpl360_hal_wrp.plc_write_read_cmd = hal_plc_send_wrrd_cmd;
	x_atpl360_hal_wrp.plc_enable_int = hal_plc_enable_interrupt;
	x_atpl360_hal_wrp.plc_delay = hal_plc_delay;
	x_atpl360_hal_wrp.plc_get_thw = hal_plc_get_thermal_warning;
	atpl360_init(&sx_atpl360_desc, &x_atpl360_hal_wrp);

	/* Callback configuration. Set NULL as Not used */
	x_atpl360_cbs.data_confirm = NULL;
	x_atpl360_cbs.data_indication = NULL;
	x_atpl360_cbs.exception_event = _handler_exception_event;
	x_atpl360_cbs.addons_event = _handler_serial_atpl360_event;
	x_atpl360_cbs.debug_mode_cb = NULL;
	x_atpl360_cbs.sleep_mode_cb = NULL;
	sx_atpl360_desc.set_callbacks(&x_atpl360_cbs);

	/* ATPL360 bin file addressing */
#ifndef PL360_BIN_ADDR_FIXED
	ul_bin_size = _get_pl360_bin_addressing(&ul_bin_addr);
#else
	ul_bin_addr = PL360_BIN_ADDR;
	ul_bin_size = PL360_BIN_SIZE;
#endif

	/* Enable ATPL360 */
	uc_ret = atpl360_enable(ul_bin_addr, ul_bin_size);
	if (uc_ret == ATPL360_ERROR) {
		PRINT_DBG(("\r\nmain: atpl360_enable call error!(%d)\r\n", uc_ret));
		while (1) {
			/* Reset watchdog */
#if (!PIC32CX)
			WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;
#else
			DWDT->WDT0_CR = WDT0_CR_KEY_PASSWD | WDT0_CR_WDRSTT;
			DWDT->WDT1_CR = WDT1_CR_KEY_PASSWD | WDT1_CR_WDRSTT;
#endif

#ifdef CONF_BOARD_LCD_EN
#if BOARD == ATPL360ASB
			vim878_show_text((const uint8_t *)"ERRORE");
#elif (BOARD == ATPL360AMB || BOARD == ATPL360MB)
			c0216CiZ_set_cursor(C0216CiZ_LINE_UP, 0);
			c0216CiZ_show((const char *)"ATPL360AMB ERROR");
#elif (BOARD == PIC32CXMTSH_DB)
			cl010_show_icon(CL010_ICON_PHASE_1);
			cl010_show_icon(CL010_ICON_PHASE_2);
			cl010_show_icon(CL010_ICON_PHASE_3);
			cl010_show_icon(CL010_ICON_PF);
#endif
#else
#if (BOARD == SAM4CMS_DB)
			LED_On(LED4);
#else
			LED_On(LED0);
#endif
#endif
			delay_ms(500);
		}
	}

	PRINT_DBG(("\r\nmain: atpl360_enable ok\r\n"));

	/* Set USI callback : Phy Tester Tool iface */
	hal_usi_set_callback(PROTOCOL_PHY_TESTER, _handler_serial_app_event, 0);

	/* Get maximum number of channels allowed from PHY layer */
	if (!sx_atpl360_desc.get_config(ATPL360_REG_MAX_NUM_CHANNELS, &uc_max_num_channels, 1, true)) {
		/* If get_config fails, set maximum number of channels to 1 (Single Channel) */
		uc_max_num_channels = 1;
	}

	if (uc_max_num_channels == 1) {
		suc_max_channel = SINGLE_CHN_8;
	} else {
		/* Double channel (channels 9 to 15) allowed */
		suc_max_channel = DOUBLE_CHN_7_8;
	}

	/* Initialize channel. Force to valid configuration */
	suc_prime_channel = min(max(CONF_APP_PRIME_CHANNEL_INI, SINGLE_CHN_1), suc_max_channel);

	while (1) {
		/* Reset watchdog */
#if (!PIC32CX)
		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;
#else
		DWDT->WDT0_CR = WDT0_CR_KEY_PASSWD | WDT0_CR_WDRSTT;
		DWDT->WDT1_CR = WDT1_CR_KEY_PASSWD | WDT1_CR_WDRSTT;
#endif

		/* Manage PL360 exceptions */
		if (sb_exception_pend) {
			sb_exception_pend = false;

			/* Set PL360 specific configuration from application */
			_set_pl360_configuration();
		}

		/* blink led 0 */
		if (b_led_swap) {
			b_led_swap = false;
#if (BOARD != PIC32CXMTSH_DB)
	#if (BOARD == SAM4CMS_DB)
			LED_Toggle(LED4);
	#else
			LED_Toggle(LED0);
	#endif
#else
#ifdef CONF_BOARD_LCD_EN
			if (b_led_swap_on) {
				cl010_clear_icon(CL010_ICON_PHASE_1);
				b_led_swap_on = false;
			} else {
				cl010_show_icon(CL010_ICON_PHASE_1);
				b_led_swap_on = true;
			}
#endif
#endif
		}

		/* HAL process (USI) */
		hal_process();

		/* Check ATPL360 pending events */
		atpl360_handle_events();
	}
}
