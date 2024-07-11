/**
 * \file
 *
 * \brief PLC Bootloader application for ATMEL PRIME Service Node
 *
 * Copyright (c) 2014 Atmel Corporation. All rights reserved.
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
 *  \mainpage ATMEL PLC Bootloader Application for Service Node
 *
 *  \section Purpose
 *
 *  This application shows how to use the bootloader in a Service Node on PLC
 *	boards.
 *
 *  \section Requirements
 *
 *  This package should be used with PLC boards, on which there is dedicated PLC
 *	hardware.
 *
 *  \section Description
 *
 *  This application will check the firmware that must be used and will
 *	initialize the board with it.
 *
 */

#include "string.h"

/* Atmel boards includes. */
#include "board.h"

/* Atmel library includes. */
#include "asf.h"

/* Bootloader include. */
#include "conf_version.h"
#include "conf_boot_regions.h"

/* Prime Bootloader version (in fixed position in flash) */
#ifdef __GNUC__
const unsigned char PRBO_version[32] __attribute__((section(".frwvrs"))) = PRBO_VERSION;
#endif
#ifdef __ICCARM__
#pragma location = ".frwvrs"
__root const char PRBO_version[32] = PRBO_VERSION;
#endif

#define BOOT_USER_SIGNATURE_SIZE_8               BOOT_FLASH_PAGE_SIZE
#define BOOT_USER_SIGNATURE_SIZE_64              (BOOT_FLASH_PAGE_SIZE / sizeof(uint64_t))


/* Bootloader states */
enum {
	BOOT_IDLE,
	BOOT_COPIED_FU_TO_BUFF,
	BOOT_COPIED_APP_TO_FU,
	BOOT_COPIED_BUFF_TO_APP,
};

/* User signature */
static uint8_t spuc_user_sign_buf[BOOT_FLASH_PAGE_SIZE];
static uint8_t *spuc_boot_config;

/* Temporal buffer to store the flash pages content (in blocks of pages) */
static uint8_t spuc_page[BOOT_FLASH_16PAGE_SIZE];

/* Counter of page blocks */
static uint8_t suc_pages_counter;

/* Bootloader state */
static uint8_t suc_boot_state;

/**
 * \brief Update user signature.
 *
 * \param uc_pages_counter       Counter of page blocks
 * \param uc_boot_state          Bootloader state
 *
 */
static void _boot_update_user_signature(uint8_t uc_pages_counter, uint8_t uc_boot_state)
{
	/* Update values */
	spuc_boot_config[16] = uc_pages_counter;
	spuc_boot_config[17] = uc_boot_state;

#if (!PIC32CX)
	/* Erase the user signature */
	flash_erase_user_signature();
	/* Update user signature */
	flash_write_user_signature((void *)spuc_user_sign_buf, BOOT_USER_SIGNATURE_SIZE_64);
#else
	/* Erase the user signature */
	flash_erase_user_signature(BOOT_USER_SIGNATURE_BLOCK);
	/* Update the user signature */
	flash_write_user_signature((void *)spuc_user_sign_buf, BOOT_USER_SIGNATURE_BLOCK, BOOT_USER_SIGNATURE_PAGE, BOOT_USER_SIGNATURE_SIZE_64);
#endif
}

/**
 * \brief Delete page in flash.
 *
 * \param ul_page_addr        Page address in flash
 *
 * \return 1  Delete is OK
 * \return 0  Delete is NOK
 */
static uint8_t _boot_delete_page(uint32_t ul_page_addr)
{
	uint32_t ul_result;

	flash_unlock(ul_page_addr, ul_page_addr + BOOT_FLASH_16PAGE_SIZE - 1, 0, 0);
	ul_result = flash_erase_page(ul_page_addr, IFLASH_ERASE_PAGES_16);
	if (ul_result != FLASH_RC_OK) {
		return 0;
	}

	return 1;
}

/**
 * \brief This function copies a page from one region into another in the flash.
 *
 * \param ul_orig_address      Origin address
 * \param ul_dest_address      Destination address
 *
 * \return 1  Copy is OK
 * \return 0  Copy is NOK
 */
static uint8_t _boot_copy_page(uint32_t ul_orig_address, uint32_t ul_dest_address)
{
	uint32_t ul_result;

	memcpy(spuc_page, (uint8_t *)(ul_orig_address), BOOT_FLASH_16PAGE_SIZE);
	ul_result = flash_write(ul_dest_address, spuc_page, BOOT_FLASH_16PAGE_SIZE, 0);
	if (ul_result != FLASH_RC_OK) {
		return 0;
	}

	return 1;
}

/**
 * \brief Verify page of written flash against its data
 *        source in ram.
 *
 * \param puc_page_in_ram     Pointer to the data source in ram
 * \param puc_page_in_flash   Pointer to the page in flash
 * \param us_page_size        Size of the page
 *
 * \return 1  Comparison result is OK
 * \return 0  Comparison result is NOK
 */
static uint8_t _boot_verify_page(uint8_t *puc_page_in_ram, uint8_t *puc_page_in_flash, uint16_t us_page_size)
{
	uint16_t i = 0;

	while (i < us_page_size) {
		if (puc_page_in_ram[i] != puc_page_in_flash[i]) {
			return 0;
		}

		i++;
	}

	return 1;
}

/**
 * \brief This function swaps the two FW regions.
 *
 * \param ul_img_size             Image size
 * \param ul_fu_base_address      Address of FU region
 * \param ul_app_base_address     Address of application region
 *
 * \return 1  Swap is OK
 * \return 0  Swap is NOK
 */
static uint8_t _boot_swap_fw_version(uint32_t ul_img_size, uint32_t ul_fu_base_address, uint32_t ul_app_base_address)
{
	uint32_t ul_buffer_addr;
	uint32_t ul_page_offset = 0;
	uint8_t uc_pages_number;
	uint8_t i;
	uint8_t uc_init;

	/* Temporary buffer of 8 pages */
	ul_buffer_addr = BOOT_BUFFER_ADDR;

	/* Number of page blocks */
	uc_pages_number = ul_img_size / BOOT_FLASH_16PAGE_SIZE;
	if (ul_img_size % BOOT_FLASH_16PAGE_SIZE) {
		uc_pages_number++;
	}

	/* Start at the last counter position */
	uc_init = suc_pages_counter;
	for (i = uc_init; i < uc_pages_number; i++) {
		/* Set page offset */
		ul_page_offset = i * BOOT_FLASH_16PAGE_SIZE;

		/* Check state */
		if ((suc_boot_state == BOOT_IDLE) || (suc_boot_state == BOOT_COPIED_BUFF_TO_APP)) {
			/* Delete temporary buffer */
			if (!_boot_delete_page(ul_buffer_addr)) {
				return 0;
		    	}

			/* Copy FU into buffer */
			if (!_boot_copy_page(ul_fu_base_address + ul_page_offset, ul_buffer_addr)) {
				return 0;
		    	}

			/* Verify the written data */
			/* If not successfully verified, repeat the page writing process */
			if (!_boot_verify_page(spuc_page, (uint8_t *)(ul_buffer_addr), BOOT_FLASH_16PAGE_SIZE)) {
				i--;
				continue;
			}

			/* Set new state */
			suc_boot_state = BOOT_COPIED_FU_TO_BUFF;
			/* Update user signature */
			_boot_update_user_signature(suc_pages_counter, suc_boot_state);
		}

		/* Check state */
		if (suc_boot_state == BOOT_COPIED_FU_TO_BUFF) {
			/* Delete FU */
			if (!_boot_delete_page(ul_fu_base_address + ul_page_offset)) {
				return 0;
		    	}

			/* Copy application into FU */
			if (!_boot_copy_page(ul_app_base_address + ul_page_offset, ul_fu_base_address + ul_page_offset)) {
				return 0;
		    	}

			/* Verify the written data */
			/* If not successfully verified, repeat the page writing process */
			if (!_boot_verify_page(spuc_page, (uint8_t *)(ul_fu_base_address + ul_page_offset), BOOT_FLASH_16PAGE_SIZE)) {
				i--;
				continue;
			}

			/* Set new state */
			suc_boot_state = BOOT_COPIED_APP_TO_FU;
			/* Update user signature */
			_boot_update_user_signature(suc_pages_counter, suc_boot_state);
		}

		/* Check state */
		if (suc_boot_state == BOOT_COPIED_APP_TO_FU) {
			/* Delete application */
			if (!_boot_delete_page(ul_app_base_address + ul_page_offset)) {
				return 0;
		    	}

			/* Copy buffer into application */
			if (!_boot_copy_page(ul_buffer_addr, ul_app_base_address + ul_page_offset)) {
				return 0;
		    	}

			/* Verify the written data */
			/* If not successfully verified, repeat the page writing process */
			if (!_boot_verify_page(spuc_page, (uint8_t *)(ul_app_base_address + ul_page_offset), BOOT_FLASH_16PAGE_SIZE)) {
				i--;
				continue;
			}

			/* Set new state */
			suc_boot_state = BOOT_COPIED_BUFF_TO_APP;
			/* Increase counter */
			++suc_pages_counter;
			/* Update user signature */
			_boot_update_user_signature(suc_pages_counter, suc_boot_state);
		}
	}

	return 1;
}

/**
 * \brief Get status of swap command
 *
 * \param ul_img_size            Image size
 * \param ul_orig_addr           Address of origin
 * \param ul_dest_addr           Address of destination
 *
 * \return True if swap needed, otherwise false
 */
static bool _boot_is_swap_cmd(uint32_t ul_img_size, uint32_t ul_orig_addr, uint32_t ul_dest_addr)
{
	if ((ul_img_size == 0) || (ul_orig_addr == 0) || (ul_dest_addr == 0) ||
	    (ul_img_size == 0xFFFFFFFF) || (ul_orig_addr == 0xFFFFFFFF) || (ul_dest_addr == 0xFFFFFFFF)){
		return false;
	}

	if (ul_orig_addr == ul_dest_addr){
		return false;
	}

	if ((suc_boot_state == BOOT_IDLE) && (suc_pages_counter > 0)) {
		return false;
	}

	if (suc_boot_state > BOOT_COPIED_BUFF_TO_APP) {
		return false;
	}

	return true;
}

/*
 * Monitoring Rate for Supply Monitor
 */
#define CONTINUOUS_MONITORING                0x00000100
#define MONITOR_ONE_OUT_OF_32SLCK_CYCLES     0x00000200
#define MONITOR_ONE_OUT_OF_256SLCK_CYCLES    0x00000200
#define MONITOR_ONE_OUT_OF_2048SLCK_CYCLES   0x00000200

#define PLATFORM_PDD_MONITORING_RATE         CONTINUOUS_MONITORING

/*
 * Threshold for Supply Monitor
 */
#define THRESHOLD_3V40                   0x0000000F
#define THRESHOLD_3V28                   0x0000000E
#define THRESHOLD_3V16                   0x0000000D
#define THRESHOLD_3V04                   0x0000000C
#define THRESHOLD_2V92                   0x0000000B
#define THRESHOLD_2V80                   0x0000000A
#define THRESHOLD_2V68                   0x00000009
#define THRESHOLD_2V56                   0x00000008
#define THRESHOLD_2V44                   0x00000007
#define THRESHOLD_2V32                   0x00000006
#define THRESHOLD_2V20                   0x00000005
#define THRESHOLD_2V08                   0x00000004

#if !PIC32CX
#define PLATFORM_PDD_MONITOR_THRESHOLD   THRESHOLD_3V04
#else
#define PLATFORM_PDD_MONITOR_THRESHOLD   SUPC_SM_THRESHOLD_2v68
#endif


/**
 *  \brief Configure microcontroller supply monitor SUPC and browndown detector.
 */
static void _setup_supply_monitor(void)
{
#if (!PIC32CX)
	/* Enable sam4c brownout detector */
	supc_enable_brownout_detector(SUPC);
	/* enable sam4c browout detector reset */
	supc_enable_brownout_reset(SUPC);
#endif

#if (!SAMG)
	/* enable and configure configure voltage monitor  */
	supc_enable_voltage_regulator(SUPC);
#endif

	/* configure sampling */
	supc_set_monitor_sampling_period(SUPC, PLATFORM_PDD_MONITORING_RATE);
	/* Set Monitor Threshold */
	supc_set_monitor_threshold(SUPC, PLATFORM_PDD_MONITOR_THRESHOLD);
	/* enable reset monitor if voltage monitor is under threshold voltage */
	supc_enable_monitor_reset(SUPC);

	/* Wait 30ms to be sure that voltage is stable */
	delay_ms(30);
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
	__set_BASEPRI(0);

	/* Atmel library function to setup for the evaluation kit being used. */
	board_init();

	/* configure supply monitor */
	_setup_supply_monitor();

#if !PIC32CX
	/* Initialize flash: 6 wait states for flash writing. */
	flash_init(FLASH_ACCESS_MODE_128, 6);
#endif

#if PIC32CX
	/* Disable Write Protection */
	efc_disable_write_protection(SEFC0);

	/* Unlock Key Bus */
	efc_unlock_key_bus_transfer(SEFC0);

	/* Set User Signature Rights */
	efc_set_usr_rights(SEFC0, EEFC_USR_WRENUSB1 | EEFC_USR_RDENUSB1);
#endif
}

/**
 * \brief Main code entry point.
 */
int main(void)
{
	uint32_t ul_img_size;
	uint32_t ul_orig_addr;
	uint32_t ul_dest_addr;
	uint32_t ul_cfg_key;
	uint8_t i;

	/* Prepare the hardware */
	_prv_setup_hardware();

	/* Read the boot configuration from user signature */
#if (!PIC32CX)
	flash_read_user_signature((void *)spuc_user_sign_buf, BOOT_USER_SIGNATURE_SIZE_64);
#else
	flash_read_user_signature((void *)spuc_user_sign_buf, BOOT_USER_SIGNATURE_BLOCK, BOOT_USER_SIGNATURE_PAGE, BOOT_USER_SIGNATURE_SIZE_64);
#endif

	/* Get boot configuration */
	spuc_boot_config = spuc_user_sign_buf + BOOT_CONFIG_OFFSET_USER_SIGN;
	ul_cfg_key = (uint32_t)(spuc_boot_config[3] << 24) + (uint32_t)(spuc_boot_config[2] << 16) + (uint32_t)(spuc_boot_config[1] << 8) + (uint32_t)(spuc_boot_config[0]);

	/* Check configuration key */
	if (ul_cfg_key != BOOT_CONFIG_KEY) {
		/* Clear boot configuration */
		memset(spuc_boot_config, 0, 16);
		/* Set configuration key */
		spuc_boot_config[0] = (uint8_t)(BOOT_CONFIG_KEY);
		spuc_boot_config[1] = (uint8_t)((BOOT_CONFIG_KEY >> 8) & 0xFF);
		spuc_boot_config[2] = (uint8_t)((BOOT_CONFIG_KEY >> 16) & 0xFF);
		spuc_boot_config[3] = (uint8_t)((BOOT_CONFIG_KEY >> 24) & 0xFF);
		/* Update user signature */
		_boot_update_user_signature(0, BOOT_IDLE);
	} else {
		/* Get boot information */
		ul_img_size = (uint32_t)(spuc_boot_config[7] << 24) + (uint32_t)(spuc_boot_config[6] << 16) + (uint32_t)(spuc_boot_config[5] << 8) + (uint32_t)(spuc_boot_config[4]);
		ul_orig_addr = (uint32_t)(spuc_boot_config[11] << 24) + (uint32_t)(spuc_boot_config[10] << 16) + (uint32_t)(spuc_boot_config[9] << 8) + (uint32_t)(spuc_boot_config[8]);
		ul_dest_addr = (uint32_t)(spuc_boot_config[15] << 24) + (uint32_t)(spuc_boot_config[14] << 16) + (uint32_t)(spuc_boot_config[13] << 8) + (uint32_t)(spuc_boot_config[12]);
		suc_pages_counter = spuc_boot_config[16];
		suc_boot_state = spuc_boot_config[17];

		/* Check if swap fw is needed. If not, load defaults */
		if (_boot_is_swap_cmd(ul_img_size, ul_orig_addr, ul_dest_addr)) {
			/* change osc to external to optimize proc. time */
			sysclk_init();
			/* Swap fw */
			_boot_swap_fw_version(ul_img_size, ul_orig_addr, ul_dest_addr);
			/* Clear boot configuration (leave configuration key) */
			memset(&spuc_boot_config[4], 0, 16);
			/* Update user signature */
			_boot_update_user_signature(0, BOOT_IDLE);

			/* Return to slow RC */
			osc_enable(OSC_SLCK_32K_RC);
			osc_wait_ready(OSC_SLCK_32K_RC);
			pmc_switch_mck_to_sclk(SYSCLK_PRES_1);
			/* Switch clock to fast RC */
 #if SAMG
		  	osc_enable(OSC_MAINCK_16M_RC);
		  	osc_wait_ready(OSC_MAINCK_16M_RC);
 #else
			osc_enable(OSC_MAINCK_12M_RC);
	  		osc_wait_ready(OSC_MAINCK_12M_RC);
 #endif
			pmc_switch_mck_to_mainck(SYSCLK_PRES_1);
		}
	}

	 __disable_irq();
	/* Disable SysTick */
	SysTick->CTRL = 0;
	/* Disable IRQs & clear pending IRQs */
	for (i = 0; i < 8; i++) {
		NVIC->ICER[i] = 0xFFFFFFFF;
		NVIC->ICPR[i] = 0xFFFFFFFF;
	}

	/* Modify vector table location */
	__DSB();
	__ISB();
	/* set the stack pointer also to the start of the firmware app */
	__set_MSP(*(int *)(BOOT_FLASH_APP_FIRMWARE_START_ADDRESS));
	/* offset the start of the vector table (first 6 bits must be zero) */
	/* The register containing the offset, from 0x00000000, is at 0xE000ED08
	 **/
	SCB->VTOR = ((uint32_t)BOOT_FLASH_APP_FIRMWARE_START_ADDRESS & SCB_VTOR_TBLOFF_Msk);
	__DSB();
	__ISB();
	__enable_irq();
	/* * (int *) 0xE000ED08 = FIRMWARE_START_ADDRESS; */

	/* jump to the start of the firmware, casting the address as function
	 * pointer to the start of the firmware. We want to jump to the address
	 * of the reset */

	/* handler function, that is the value that is being pointed at position
	 * FIRMWARE_RESET_ADDRESS */
	void (*runFirmware)(void) = NULL;
	runFirmware = (void (*)(void))(*(uint32_t *)BOOT_FLASH_APP_FIRMWARE_RESET_ADDRESS);
	runFirmware();

	while (1) {
	}
}
