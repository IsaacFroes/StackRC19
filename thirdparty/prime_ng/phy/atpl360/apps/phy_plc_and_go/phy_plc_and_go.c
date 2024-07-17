/**
 * \file
 *
 * \brief PHY_CHAT : PRIME Phy Getting Started Application. Main.
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

/* Atmel library includes. */
#include "asf.h"
#include "board.h"
#include "conf_app_example.h"
#include "app_console.h"
#include "app_phy_ctl.h"
#include "hal_private.h"

/* LED blink rate in ms */
#define COUNT_MS_SWAP_LED       500

/* 1 ms timer definitions. Timer used to manage leds */
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

/* String shown at initialization */
#define STRING_EOL    "\r"
#define STRING_HEADER "\r\n-- PRIME Phy Chat Application --" \
	"\r\n-- "BOARD_NAME " --" \
	"\r\n-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

/* LED management variables */
static uint32_t sul_count_ms = COUNT_MS_SWAP_LED;
uint32_t sul_ind_count_ms = 0;
static uint8_t suc_led_swap = 0;
static bool sb_ind_led_swap = false;
#if (BOARD == PIC32CXMTSH_DB) || (BOARD==PIC32CXMTC_DB)
#ifdef CONF_BOARD_LCD_EN
static bool b_led_swap_on = false;
#endif
#endif

/**
 *  Configure Serial Console.
 */
static void configure_dbg_console(void)
{
#ifdef CONF_BOARD_UDC_CONSOLE
	/* Configure console UDC (USB, SAMG55). */
	stdio_udc_init(UDP, (void *)hal_usb_udc_putchar, (void *)hal_usb_udc_getchar, (void *)hal_usb_udc_start);
	/* Wait to open terminal */
	while (!hal_usb_cdc_get_dtr()) {
		/* Reset watchdog */
		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;

		/* Blink led 0 */
		if (suc_led_swap == 2) {
			suc_led_swap = 0;
			LED_Toggle(LED0);
		}
	}
#else
	/* Configure console UART. */
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
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
#endif
}

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
 * \brief Initialize 1mSec timer 3 interrupt
 */
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

	/** Start the timer. TC1, chanel 0 = TC3 */
	tc_start(TC_1MS, TC_1MS_CHN);
}

/**
 * \brief Interrupt handler for Timer 3
 */
void TC_1MS_Handler(void)
{
	/* Clear status bit to acknowledge interrupt */
	tc_get_status(TC_1MS, TC_1MS_CHN);

	/* Update ms counter to blink led 0 */
	if (!sul_count_ms--) {
		sul_count_ms = COUNT_MS_SWAP_LED;
		suc_led_swap++;
	}

	/* Update ms counter to blink led 1 when a PLC message is received */
	if (sul_ind_count_ms) {
		if (!--sul_ind_count_ms) {
			sb_ind_led_swap = true;
		}
	}
}

#ifdef CONF_BOARD_LCD_EN

/**
 * \brief LCD initialization
 */
static void _lcd_init(void)
{
	status_code_t status;
#  if (BOARD == ATPL360MB)
	status = (status_code_t)c0216CiZ_init();
	if (status != STATUS_OK) {
		puts("-- LCD Initialization fails! --\r\n");
		while (1) {
		}
	}

	c0216CiZ_set_cursor(C0216CiZ_LINE_UP, 0);
	c0216CiZ_show((const char *)"ATPL360MB PRIME");
	c0216CiZ_set_cursor(C0216CiZ_LINE_DOWN, 0);
	c0216CiZ_show((const char *)"PRIME Chat App");
#elif BOARD == PIC32CXMTSH_DB || (BOARD==PIC32CXMTC_DB)
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
#  else
	/* Only ATPL360MB and PIC32CX boards support LCD */
#    error ERROR in board definition
#  endif
}

#endif

/**
 * \brief Main code entry point.
 */
int main(void)
{
	/* Prepare the hardware */
	prvSetupHardware();

	/* Initialize 1ms timer. Used to manage leds */
	initTimer1ms();

	/* Initialize UART debug */
	configure_dbg_console();
	puts(STRING_HEADER);

#ifdef CONF_BOARD_LCD_EN
	/* Initialize LCD */
	_lcd_init();
#endif

	/* Initialize PL360 PHY controller module */
	phy_ctl_pl360_init(CONF_PRIME_CHANNEL_INI);

	/* Initialize Chat App */
	app_console_init();

	while (1) {
		/* Reset watchdog */
#if (!PIC32CX)
		WDT->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT;
#else
		DWDT->WDT0_CR = WDT0_CR_KEY_PASSWD | WDT0_CR_WDRSTT;
		DWDT->WDT1_CR = WDT1_CR_KEY_PASSWD | WDT1_CR_WDRSTT;
#endif

		/* Blink led 0 */
		if (suc_led_swap) {
			suc_led_swap = 0;
#if (BOARD != PIC32CXMTSH_DB) || (BOARD!=PIC32CXMTC_DB)
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

		/* Turn off led 1 after receiving PLC message */
		if (sb_ind_led_swap) {
			sb_ind_led_swap = false;
#ifdef LED1_GPIO
			LED_Off(LED1);
#endif
#if (BOARD == PIC32CXMTSH_DB) || (BOARD==PIC32CXMTC_DB)
#ifdef CONF_BOARD_LCD_EN
			cl010_clear_icon(CL010_ICON_PHASE_2);  
#endif
#endif
		}

		/* PL360 PHY controller module process */
		phy_ctl_process();

		/* Chat App process */
		app_console_process();
	}
}
