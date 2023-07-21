/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2023 by Rapid Silicon                                   *
 *   chernyee.kok@rapidsilicon.com                                         *
 ***************************************************************************/

#ifndef OPENOCD_PLD_GEMINI_H
#define OPENOCD_PLD_GEMINI_H

#include <jtag/jtag.h>
#include <target/target.h>
#include <target/target_type.h>
#include <target/riscv/riscv.h>

#define MAX_NUM_OF_CHIP_ID	5

struct device_t {
	char *name;
	intptr_t probe_addr;
	intptr_t debug_control;
	intptr_t spare_reg;
	intptr_t cfg_status;
	intptr_t fsbl_ubi_addr;
	uint32_t ram_size;
	intptr_t cbuffer;
	intptr_t read_counter;
	intptr_t writer_counter;
	uint32_t chip_id[MAX_NUM_OF_CHIP_ID];
};

struct target_info_t {
	struct target *target;
	struct jtag_tap *tap;
	struct device_t *device;
};

struct gemini_pld_device {
	struct target_info_t *target_info;
	uint32_t count;
};

struct gemini_stats {
	uint64_t total_packages_size;
	uint64_t data_sent;
	uint32_t cicular_buffer_full_count;
	uint8_t package_count;
	uint8_t log;
	uint32_t timeout_counter;
	uint32_t wait_time_us;
	uint64_t total_overall_us;
	uint64_t total_us;
};

/* bootrom/fsbl command and status definitions */
enum gemini_prg_tsk_cmd {
	GEMINI_PRG_TSK_CMD_IDLE = 0x0,
	// Bootrom Boot FSBL or FSBL DDR INIT
	GEMINI_PRG_TSK_CMD_BBF_FDI = 0x281,
	GEMINI_PRG_TSK_CMD_CFG_BITSTREAM_FPGA = 0x282,
	GEMINI_PRG_TSK_CMD_CFG_BITSTREAM_FLASH = 0x283,
	GEMINI_PRG_TSK_CMD_DEBUG_MODE = 0x284
};

enum gemini_prg_st {
	GEMINI_PRG_ST_PENDING = 0,
	GEMINI_PRG_ST_TASK_COMPLETE = 1,
	// Crypto errors
	// rs_crypto_status value +1 but no crypto success
	GEMINI_PRG_ST_ERR_NULL_POINTER = 2,
	GEMINI_PRG_ST_ERR_MEM_UNAVAILABLE = 3,
	GEMINI_PRG_ST_ERR_CRYPTO_INIT_ERROR = 4,
	GEMINI_PRG_ST_ERR_RSA_VERIFY_ERROR = 5,
	GEMINI_PRG_ST_ERR_ECDSA_VERIFY_ERROR = 6,
	GEMINI_PRG_ST_ERR_DECRYPTION_ERROR = 7,
	GEMINI_PRG_ST_ERR_HASH_CALC_ERROR = 8,
	GEMINI_PRG_ST_ERR_ECP_CONFIG_ERROR = 9,
	GEMINI_PRG_ST_ERR_INVALID_RSA_TYPE = 10,
	GEMINI_PRG_ST_ERR_INVALID_HASH_ALGO = 11,
	GEMINI_PRG_ST_ERR_INVALID_ENCRYPTION_ALGO = 12,
	GEMINI_PRG_ST_ERR_INVALID_SIGNING_ALGO = 13,
	GEMINI_PRG_ST_ERR_INVALID_CIPHER_TYPE = 14,
	GEMINI_PRG_ST_ERR_BOP_IMAGE_NOT_FOUND = 15,
	GEMINI_PRG_ST_ERR_INVALID_IMAGE_TYPE = 16,
	GEMINI_PRG_ST_ERR_KEY_WRITE_ERROR = 17,
	GEMINI_PRG_ST_ERR_KEY_READ_ERROR = 18,
	GEMINI_PRG_ST_ERR_KEY_LOCK_ERROR = 19,
	GEMINI_PRG_ST_ERR_INVALID_PUBLIC_KEY = 20,
	GEMINI_PRG_ST_ERR_NON_SECURE_IMAGE = 21,
	GEMINI_PRG_ST_ERR_CRC_CHECK_FAILED = 22,
	GEMINI_PRG_ST_ERR_INVALID_KEY_SLOT = 23,
	GEMINI_PRG_ST_ERR_INVALID_KEY_SRC = 24,
	GEMINI_PRG_ST_ERR_NO_ENC_KEY = 25,
	GEMINI_PRG_ST_ERR_NO_ACCESS = 26,
	GEMINI_PRG_ST_ERR_DATA_TRANSFER_ERROR = 27,
	GEMINI_PRG_ST_ERR_KEY_NOT_IN_MANIFEST = 28,
	GEMINI_PRG_ST_ERR_CRYPTO_xCB_ERROR = 29,
	GEMINI_PRG_ST_ERR_INVALID_KEYS = 30,
	// Other errors
	GEMINI_PRG_ST_ERR_INIT_SPI = 31,
	GEMINI_PRG_ST_ERR_INIT_DDR = 32,
	GEMINI_PRG_ST_ERR_FLASH_PROGRAM = 33,
	GEMINI_PRG_ST_ERR_FLASH_ERASE = 34
};

enum gemini_prg_fw_type {
	GEMINI_PRG_FW_TYPE_UKNOWN = 0,
	GEMINI_PRG_FW_TYPE_BOOTROM = 1,
	GEMINI_PRG_FW_TYPE_CFG_FSBL = 2,
};

#endif /* OPENOCD_PLD_GEMINI_H */
