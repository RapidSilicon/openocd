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

struct device_t {
	char *name;
	intptr_t debug_control;
	intptr_t spare_reg;
	intptr_t cfg_status;
	intptr_t fsbl_ubi_addr;
	uint32_t ram_size;
	intptr_t cbuffer;
	intptr_t read_counter;
	intptr_t write_counter;
};

struct target_info_t {
	struct target *target;
	struct jtag_tap *tap;
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
enum prg_reg_tsk_cmd {
	PRG_REG_TSK_CMD_IDLE = 0x0,
	// Bootrom Boot FSBL or FSBL DDR INIT
	PRG_REG_TSK_CMD_BBF_FDI = 0x281,
	PRG_REG_TSK_CMD_CFG_BITSTREAM_FPGA = 0x282,
	PRG_REG_TSK_CMD_CFG_BITSTREAM_FLASH = 0x283,
	PRG_REG_TSK_CMD_CFG_OTP = 0x284,
	PRG_REG_TSK_CMD_DEBUG_MODE = 0x2BF
};

enum prg_reg_st {
	PRG_REG_ST_PENDING = 0,
	PRG_REG_ST_TASK_COMPLETE = 1,
	// Crypto errors
	// rs_crypto_status value +1 but no crypto success
	PRG_REG_ST_ERR_NULL_POINTER = 2,
	PRG_REG_ST_ERR_MEM_UNAVAILABLE = 3,
	PRG_REG_ST_ERR_CRYPTO_INIT_ERROR = 4,
	PRG_REG_ST_ERR_RSA_VERIFY_ERROR = 5,
	PRG_REG_ST_ERR_ECDSA_VERIFY_ERROR = 6,
	PRG_REG_ST_ERR_DECRYPTION_ERROR = 7,
	PRG_REG_ST_ERR_HASH_CALC_ERROR = 8,
	PRG_REG_ST_ERR_ECP_CONFIG_ERROR = 9,
	PRG_REG_ST_ERR_INVALID_RSA_TYPE = 10,
	PRG_REG_ST_ERR_INVALID_HASH_ALGO = 11,
	PRG_REG_ST_ERR_INVALID_ENCRYPTION_ALGO = 12,
	PRG_REG_ST_ERR_INVALID_SIGNING_ALGO = 13,
	PRG_REG_ST_ERR_INVALID_CIPHER_TYPE = 14,
	PRG_REG_ST_ERR_BOP_IMAGE_NOT_FOUND = 15,
	PRG_REG_ST_ERR_INVALID_IMAGE_TYPE = 16,
	PRG_REG_ST_ERR_KEY_WRITE_ERROR = 17,
	PRG_REG_ST_ERR_KEY_READ_ERROR = 18,
	PRG_REG_ST_ERR_KEY_LOCK_ERROR = 19,
	PRG_REG_ST_ERR_INVALID_PUBLIC_KEY = 20,
	PRG_REG_ST_ERR_INVALID_ENC_KEY = 21,
	PRG_REG_ST_ERR_NON_SECURE_IMAGE = 22,
	PRG_REG_ST_ERR_CRC_CHECK_FAILED = 23,
	PRG_REG_ST_ERR_INVALID_KEY_SLOT = 24,
	PRG_REG_ST_ERR_INVALID_KEY_SRC = 25,
	PRG_REG_ST_ERR_NO_ENC_KEY = 26,
	PRG_REG_ST_ERR_NO_ACCESS = 27,
	PRG_REG_ST_ERR_DATA_TRANSFER_ERROR = 28,
	PRG_REG_ST_ERR_KEY_NOT_IN_MANIFEST = 29,
	PRG_REG_ST_ERR_CRYPTO_xCB_ERROR = 30,
	PRG_REG_ST_ERR_INVALID_KEYS = 31,
	PRG_REG_ST_ERR_NO_ACTIONS = 32,
	PRG_REG_ST_INTEGRITY_CHECK_FAILED = 33,
	PRG_REG_ST_CBUFFER_READ_ERROR = 34,
	PRG_REG_ST_INVALID_TARGET_DEVICE = 35,
	PRG_REG_ST_DECOMPRESSION_ERROR = 36,
	PRG_REG_ST_INVALID_ACTION_CMD = 37,
	PRG_REG_ST_INVALID_CMPR_ALGO = 38,
	PRG_REG_ST_CBUFFER_DATA_CONSUMPTION_MISMATCH = 39,
	PRG_REG_ST_ALL_ACTIONS_NOT_CONSUMED = 40,
	PRG_REG_ST_MALFORMED_ACTION = 41,
	PRG_REG_ST_LIFECYCLE_PROGRAMMING_ERROR = 42,
	// Other errors
	PRG_REG_ST_ERR_FLASH_PROGRAM = 43,
	PRG_REG_ST_ERR_FLASH_ERASE = 44,
	PRG_REG_ST_ERR_INIT_SPI = 45,
	PRG_REG_ST_ERR_INIT_DDR = 46,
	PRG_REG_ST_ERR_INVALID_CMD = 47
};

enum prg_reg_fw_type {
	PRG_REG_FW_TYPE_UKNOWN = 0,
	PRG_REG_FW_TYPE_BOOTROM = 1,
	PRG_REG_FW_TYPE_CFG_FSBL = 2,
	PRG_REG_FW_TYPE_MANF_FSBL = 3
};

#endif /* OPENOCD_PLD_GEMINI_H */
