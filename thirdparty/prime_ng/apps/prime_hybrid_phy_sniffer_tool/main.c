/**
 *
 * \file
 *
 * \brief PRIME Hybrid PLC-RF sniffer tool
 *
 * Copyright (c) 2021 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */


/* Boards includes. */
#include "board.h"

/* Library includes. */
#include "asf.h"
#include "prime_hal_wrapper.h"
#include "sniffer_if.h"
#include "hal_private.h"

/* Config includes */
#include "conf_app_example.h"
#include "conf_pal.h"

/* Define Time for led swapping */
#define COUNT_US_SWAP_LED                          500000 /* 500 ms */

#ifdef CONF_BOARD_UART_CONSOLE
#define STRING_EOL    "\r"
#define STRING_HEADER "-- ATMEL PRIME v1.3 Sniffer Application --\r\n" \
	"-- "BOARD_NAME " --\r\n" \
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL
#endif

/** Define HAL API interface struct */
extern const hal_api_t hal_api;

/* Timer and LED variables */
static uint32_t sul_led0_swap_time_int;
static bool sb_led_swap;
#if (BOARD == PIC32CXMTSH_DB)
#ifdef CONF_BOARD_LCD_EN
static bool sb_led_swap_on;
#endif
#endif

/**
 * \brief Timer handler of periodic interrupt every 500 ms to swap LED0.
 *
 * \param ul_id Interrupt identifier
 */
static void _led0_swap_handler(uint32_t ul_id)
{
	uint32_t ul_timer_int_id;

	/* Program next periodic interrupt every 500 ms */
	sul_led0_swap_time_int += COUNT_US_SWAP_LED;
	prime_hal_timer_1us_set_int(sul_led0_swap_time_int, false, _led0_swap_handler, &ul_timer_int_id);

	/* Blink LED */
	sb_led_swap = true;

	UNUSED(ul_id);
}

/**
 * \brief Initialize timer of 500 ms, using timer of 1 us service
 */
static inline void _timer_led_swap_init(void)
{
	uint32_t ul_timer_int_id;
	uint32_t ul_time_us;

	/* Get current time in us */
	ul_time_us = prime_hal_timer_1us_get();

	/* Program first periodic interrupt every 500 milliseconds */
	sul_led0_swap_time_int = ul_time_us + COUNT_US_SWAP_LED;
	prime_hal_timer_1us_set_int(sul_led0_swap_time_int, false, _led0_swap_handler, &ul_timer_int_id);
	sb_led_swap = false;
}

/**
 * \brief Handler to receive USI data from sniffer tool.
 */
static bool _handler_sniffer_tool_cmd_event(uint8_t *puc_serial_data, uint16_t us_len)
{
	uint16_t us_rf_band_opmode, us_rf_channel;
	uint8_t uc_sniffer_if_cmd;
	uint8_t uc_plc_channel;

	/* Protection for invalid length */
	if (!us_len) {
		return true;
	}

	/* Process received message. Get command */
	uc_sniffer_if_cmd  = puc_serial_data[0];

	switch (uc_sniffer_if_cmd) {
	case SNIFFER_IF_PHY_COMMAND_SET_PLC_CHANNEL:
		/* Set PLC channel */
		uc_plc_channel = puc_serial_data[1];
		pal_set_cfg(PAL_ID_CFG_TXRX_CHANNEL, &uc_plc_channel, 1, 0);
		break;

	case SNIFFER_IF_PHY_COMMAND_SET_RF_BAND_OPM_CHANNEL:
		/* Set RF band, operating mode and channel */
		us_rf_band_opmode  = (puc_serial_data[1] << 8) | puc_serial_data[2];
		us_rf_channel  = (puc_serial_data[3] << 8) | puc_serial_data[4];
		pal_set_cfg(PAL_ID_RF_PHY_BAND_OPERATING_MODE, &us_rf_band_opmode, 2, 512);
		pal_set_cfg(PAL_ID_CFG_TXRX_CHANNEL, &us_rf_channel, 2, 512);
		break;

	default:
		/* Not supported */
		break;
	}

	return true;
}

#ifdef CONF_BOARD_UART_CONSOLE

/**
 * \brief Configure UART console.
 */
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
	prime_hal_config((hal_api_t *)&hal_api);
}

/**
 * \brief Main code entry point.
 */
int main( void )
{
	uint8_t uc_sniffer_enabled = 1;
	pal_callbacks_t x_pal_callbacks;
#ifdef CONF_BOARD_LCD_EN
	status_code_t status;
#endif

	/* Prepare the hardware */
	prvSetupHardware();

	/* Initialize timer to swap LED */
	_timer_led_swap_init();

#ifdef CONF_BOARD_UART_CONSOLE
	/* UART debug */
	configure_dbg_console();
	puts(STRING_HEADER);
#endif

#ifdef CONF_BOARD_LCD_EN
#if BOARD == PIC32CXMTSH_DB
	/* Initialize the CL010 LCD glass component. */
	status = cl010_init();
	if (status != STATUS_OK) {
		puts("-- LCD Initialization fails! --\r\n");
		while (1) {
		}
	}

	cl010_clear_all();
	cl010_show_icon(CL010_ICON_MICROCHIP);
	cl010_show_numeric_string(CL010_LINE_UP, (const uint8_t *)"51515151");
	cl010_show_numeric_string(CL010_LINE_DOWN, (const uint8_t *)"0000360");
	cl010_show_icon(CL010_ICON_DOT_5);
	cl010_show_icon(CL010_ICON_P_PLUS);
	cl010_show_icon(CL010_ICON_P_MINUS);
	sb_led_swap_on = false;
#else
#error ERROR in board definition
#endif
#endif

	/* Initialize PRIME PAL (sniffer is managed by PAL) */
	pal_init();

	/* PAL callbacks not used */
	x_pal_callbacks.data_confirm = NULL;
	x_pal_callbacks.data_indication = NULL;
	x_pal_callbacks.switch_rf_ch = NULL;
	pal_set_callbacks(&x_pal_callbacks);

	/* Enable Sniffer */
	pal_set_cfg(PAL_ID_PHY_SNIFFER_EN, &uc_sniffer_enabled, 1, 0);
	pal_set_cfg(PAL_ID_PHY_SNIFFER_EN, &uc_sniffer_enabled, 1, 512);

	/* Set Sniffer USI callback to receive configuration commands */
	prime_hal_usi_set_callback(PROTOCOL_SNIF_PRIME, _handler_sniffer_tool_cmd_event, PHY_SNIFFER_USI_PORT);

	while (1) {
		/* Reset watchdog */
#if (!PIC32CX)
		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;
#else
		DWDT->WDT0_CR = WDT0_CR_KEY_PASSWD | WDT0_CR_WDRSTT;
		DWDT->WDT1_CR = WDT1_CR_KEY_PASSWD | WDT1_CR_WDRSTT;
#endif

		/* blink led 0 */
		if (sb_led_swap) {
			sb_led_swap = false;
#if (BOARD != PIC32CXMTSH_DB)
	LED_Toggle(LED0);
#else
#ifdef CONF_BOARD_LCD_EN
#if (BOARD == PIC32CXMTSH_DB)
			if (sb_led_swap_on) {
				cl010_clear_icon(CL010_ICON_PHASE_1);
				sb_led_swap_on = false;
			} else {
				cl010_show_icon(CL010_ICON_PHASE_1);
				sb_led_swap_on = true;
			}
#endif
#endif
#endif
		}

		/* HAL process (USI) */
		hal_process();

		/* PAL process */
		pal_process();
	}
}
