/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2023 by Rapid Silicon                                   *
 *   chernyee.kok@rapidsilicon.com                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "gemini.h"
#include "gemini_bit.h"
#include "pld.h"
#include "helper/time_support.h"

//#define EMULATOR_BUILD

#ifdef EMULATOR_BUILD
#define GEMINI_SPARE_REG		0x8003fbfc //0x8003fffc
#define GEMINI_LOAD_ADDRESS		0x80020000
#define GEMINI_SRAM_SIZE		(127 * 1024)
#else
#define GEMINI_SPARE_REG		0xf10000f0
#define GEMINI_LOAD_ADDRESS		0x00000000
#define GEMINI_SRAM_SIZE		(255 * 1024)
#endif

#define GEMINI_IDCODE			0x1000563d
#define GEMINI_PRODUCT_ID		0x31303050
#define GEMINI_DEBUG_CONTROL	0xf1000028
#define GEMINI_SRAM_ADDRESS		0x80000000
#define GEMINI_ACPU				1
#define GEMINI_BCPU				0
#define GEMINI_COMMAND_POLLS	5
#define DM_SBCS					0x38
#define DM_SBADDRESS0			0x39
#define DM_SBDATA0				0x3c
#define DDR_INIT				1
#define DDR_NOT_INIT			0

enum gemini_prg_mode {
	GEMINI_PRG_MODE_FPGA,
	GEMINI_PRG_MODE_SPI_FLASH
};

static int gemini_sysbus_write_reg32(struct target * target, target_addr_t address, uint32_t value)
{
	struct riscv_info * ri = target->arch_info;

	LOG_DEBUG("[RS] Writing 0x%08x to 0x%08x via System Bus on ACPU", value, (uint32_t)address);

	if (ri->dmi_write(target, DM_SBCS, (2u << 17) | (7u << 12)) != ERROR_OK)
		return ERROR_FAIL;

	if (ri->dmi_write(target, DM_SBADDRESS0, address) != ERROR_OK)
		return ERROR_FAIL;

	// don't care about the return status since the dm is changed to bcpu after this
	ri->dmi_write(target, DM_SBDATA0, value);
	return ERROR_OK;
}

static int gemini_write_reg32(struct target * target, target_addr_t address, uint32_t width, uint32_t offset, uint32_t value)
{
	int retval = ERROR_OK;
	uint32_t tmp;

	if (target_halt(target) == ERROR_OK) 
	{
		if (target_read_u32(target, address, &tmp) == ERROR_OK)
		{
			uint32_t bitmask = (1u << width) - 1;

			if (target_write_u32(target, address, (tmp & ~(bitmask << offset)) | ((value & bitmask) << offset)) != ERROR_OK)
			{
				LOG_WARNING("[RS] Failed to write to address 0x%08x", (uint32_t)address);
				retval = ERROR_FAIL;
			}
		}
		else
		{
			LOG_WARNING("[RS] Failed to read from address 0x%08x", (uint32_t)address);
			retval = ERROR_FAIL;
		}
		target_resume(target, true, 0, true, false);
	}
	else
	{
		LOG_WARNING("[RS] Failed to halt target");
		retval = ERROR_FAIL;
	}

	return retval;
}

static int gemini_read_reg32(struct target * target, target_addr_t address, uint32_t *value)
{
	int retval = ERROR_OK;

	if (target_halt(target) == ERROR_OK) 
	{
		if (target_read_u32(target, address, value) != ERROR_OK)
		{
			LOG_WARNING("[RS] Failed to read from address 0x%08x", (uint32_t)address);
			retval = ERROR_FAIL;
		}
		target_resume(target, true, 0, true, false);
	}
	else
	{
		LOG_WARNING("[RS] Failed to halt target");
		retval = ERROR_FAIL;
	}

	return retval;
}

static int gemini_write_memory(struct target * target, target_addr_t address, uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int retval = ERROR_OK;

	if (target_halt(target) == ERROR_OK)
	{
		if (target_write_memory(target, address, size, count, buffer) != ERROR_OK)
		{
			LOG_WARNING("[RS] Failed to write to address 0x%08x", (uint32_t)address);
			retval = ERROR_FAIL;
		}
		target_resume(target, true, 0, true, false);
	}
	else
	{
		LOG_ERROR("[RS] Failed to halt target");
		return ERROR_FAIL;
	}

	return retval;
}

static int gemini_get_cpu_type(struct target * target, uint32_t *cpu_type)
{
	uint32_t debug_control;

	if (gemini_read_reg32(target, GEMINI_DEBUG_CONTROL, &debug_control) == ERROR_OK)
	{
		*cpu_type = debug_control & 0x1 ? GEMINI_ACPU : GEMINI_BCPU;
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

static int gemini_get_firmware_type(struct target * target, uint32_t *fw_type)
{
	uint32_t spare_reg;

	if (gemini_read_reg32(target, GEMINI_SPARE_REG, &spare_reg) == ERROR_OK)
	{
		spare_reg >>= 29;
		if (spare_reg == GEMINI_PRG_FW_TYPE_BOOTROM || spare_reg == GEMINI_PRG_FW_TYPE_CFG_FSBL)
		{
			*fw_type = spare_reg;
			return ERROR_OK;
		}
	}

	return ERROR_FAIL;
}

static int gemini_get_command_status(struct target * target, uint32_t *status)
{
	uint32_t spare_reg;

	if (gemini_read_reg32(target, GEMINI_SPARE_REG, &spare_reg) == ERROR_OK)
	{
		*status = (spare_reg >> 10) & 0x3f;
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

static int gemini_get_ddr_status(struct target * target, uint32_t *status)
{
	uint32_t spare_reg;

	if (gemini_read_reg32(target, GEMINI_SPARE_REG, &spare_reg) == ERROR_OK)
	{
		*status = (spare_reg & (1u << 16)) ? DDR_INIT : DDR_NOT_INIT;
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

static int gemini_check_target_device(struct gemini_pld_device *gemini_info, gemini_bit_file_t *bit_file)
{
	if (gemini_info->tap->idcode != GEMINI_IDCODE)
	{
		LOG_ERROR("[RS] Not gemini device IDCODE 0x%08x", gemini_info->tap->idcode);
		return ERROR_FAIL;
	}

	if (bit_file->ubi_header->product_id != GEMINI_PRODUCT_ID)
	{
		LOG_ERROR("[RS] Invalid gemini product id found in bitstream file 0x%x", bit_file->ubi_header->product_id);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int gemini_reset_bcpu(struct target * target)
{
	LOG_INFO("[RS] Reseting BCPU...");

	if (target->type->assert_reset(target) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to assert reset signal");
		return ERROR_FAIL;
	}

	sleep(1);

	if (target->type->deassert_reset(target) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to de-assert reset signal");
		return ERROR_FAIL;
	}

	LOG_INFO("[RS] Resetted BCPU successfully");
	return ERROR_OK;
}

static int gemini_switch_to_bcpu(struct target * target)
{
	uint32_t cpu_type;

	if (gemini_get_cpu_type(target, &cpu_type) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to get connected cpu type");
		return ERROR_FAIL;
	}

	if (cpu_type == GEMINI_BCPU)
	{
		LOG_INFO("[RS] Connected to BCPU");
		return ERROR_OK;
	}

	LOG_INFO("[RS] Perform switching from ACPU to BCPU...");

	if (gemini_sysbus_write_reg32(target, GEMINI_DEBUG_CONTROL, 0) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to write debug_control register");
		return ERROR_FAIL;
	}

	jtag_add_tlr();
	jtag_execute_queue();
	target->examined = false;

	if (target_examine_one(target) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to re-initialize the target");
		return ERROR_FAIL;
	}

	if (gemini_get_cpu_type(target, &cpu_type) != ERROR_OK || cpu_type == GEMINI_ACPU)
	{
		LOG_INFO("[RS] Failed to switch to BCPU");
		return ERROR_FAIL;
	}

	LOG_INFO("[RS] Switched to BCPU");

	return ERROR_OK;
}

static int gemini_poll_command_complete_and_status(struct target * target, uint32_t *status)
{
	int retval = ERROR_OK;
	uint32_t num_polls = 0;

	while (1)
	{
		LOG_DEBUG("[RS] Poll command status #%d...", ++num_polls);

		retval = gemini_get_command_status(target, status);
		if (retval != ERROR_OK)
			break;
		if (*status != GEMINI_PRG_ST_PENDING)
			break;
		if (num_polls >= GEMINI_COMMAND_POLLS)
		{
			LOG_ERROR("[RS] Timed out waiting for task to complete.");
			retval = ERROR_TIMEOUT_REACHED;
			break;
		}
		sleep(1);
	}

	// check the command status
	if (retval == ERROR_OK)
	{
		if (*status != GEMINI_PRG_ST_TASK_COMPLETE) {
			LOG_ERROR("[RS] Command completed with error %d", *status);
			retval = ERROR_FAIL;
		}
	}
	// clear command and status field
	gemini_write_reg32(target, GEMINI_SPARE_REG, 16, 0, GEMINI_PRG_TSK_CMD_IDLE);

	return retval;
}

static int gemini_load_fsbl(struct target *target, gemini_bit_file_t *bit_file)
{
	int retval = ERROR_OK;
	uint32_t fsbl_size = bit_file->ubi_header->fsbl_size;
	uint32_t size = 1;
	uint32_t status;
	uint32_t fw_type;

	if (gemini_get_firmware_type(target, &fw_type) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to determine the firmware type");
		return ERROR_FAIL;
	}

	if (fw_type == GEMINI_PRG_FW_TYPE_CFG_FSBL)
	{
		LOG_INFO("[RS] FSBL firmware type detected");
		return ERROR_OK;
	}

	LOG_INFO("[RS] Loading FSBL firmware...");

	if (fsbl_size == 0)
	{
		LOG_ERROR("[RS] FSBL BOP not exist in the bitstream");
		return ERROR_FAIL;
	}

	if (fsbl_size > GEMINI_SRAM_SIZE)
	{
		LOG_ERROR("[RS] FSBL bitstream size (%d bytes) is larger than available SRAM (%d bytes)", fsbl_size, GEMINI_SRAM_SIZE);
		return ERROR_FAIL;
	}

	if ((fsbl_size % 4) == 0)
		size = 4;
	else if ((fsbl_size % 2) == 0)
		size = 2;

	retval = gemini_write_memory(target, GEMINI_SRAM_ADDRESS, size, fsbl_size / size, (uint8_t *)bit_file->ubi_header);
	if (retval != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to write bitstream of %d bytes to SRAM at 0x%08x", fsbl_size, GEMINI_SRAM_ADDRESS);
		return ERROR_FAIL;
	}

	LOG_DEBUG("[RS] Wrote %d bytes to SRAM at 0x%08x", fsbl_size, GEMINI_SRAM_ADDRESS);

	if (gemini_write_reg32(target, GEMINI_SPARE_REG, 16, 0, GEMINI_PRG_TSK_CMD_BBF_FDI) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to write command 0x%x to spare_reg at 0x%08x", GEMINI_PRG_TSK_CMD_BBF_FDI, GEMINI_SPARE_REG);
		return ERROR_FAIL;
	}

	LOG_DEBUG("[RS] Wrote command 0x%x to spare_reg at 0x%08x", GEMINI_PRG_TSK_CMD_BBF_FDI, GEMINI_SPARE_REG);

	retval = gemini_poll_command_complete_and_status(target, &status);
	if (retval == ERROR_OK)
	{
		// double check if the firmware type is FSBL
		if (gemini_get_firmware_type(target, &fw_type) != ERROR_OK || fw_type != GEMINI_PRG_FW_TYPE_CFG_FSBL)
			retval = ERROR_FAIL;
	}

	if (retval != ERROR_OK)
	{
		if (retval == ERROR_TIMEOUT_REACHED)
			gemini_reset_bcpu(target); // reset target to known state when command timeout
		LOG_ERROR("[RS] Failed to load FSBL firmware");
	}
	else
		LOG_INFO("[RS] Loaded FSBL firmware of size %d bytes successfully.", fsbl_size);

	return retval;
}

static int gemini_init_ddr(struct target *target)
{
	int retval = ERROR_OK;
	uint32_t status;

	if (gemini_get_ddr_status(target, &status) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to determine the DDR memory status");
		return ERROR_FAIL;
	}

	if (status == DDR_INIT)
	{
		LOG_INFO("[RS] DDR memory is already initialized");
		return ERROR_OK;
	}

	LOG_INFO("[RS] Initializing DDR memory...");

	if (gemini_write_reg32(target, GEMINI_SPARE_REG, 16, 0, GEMINI_PRG_TSK_CMD_BBF_FDI) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to write command 0x%x to spare_reg at 0x%08x", GEMINI_PRG_TSK_CMD_BBF_FDI, GEMINI_SPARE_REG);
		return ERROR_FAIL;
	}

	retval = gemini_poll_command_complete_and_status(target, &status);
	if (retval != ERROR_OK)
	{
		if (retval == ERROR_TIMEOUT_REACHED)
			gemini_reset_bcpu(target); // reset target to known state when command timeout
		LOG_ERROR("[RS] Failed to initialize DDR memory");
	}
	else
		LOG_INFO("[RS] DDR memory is initialized successfully");

	return retval;
}

static int gemini_program_bitstream(struct target *target, gemini_bit_file_t *bit_file, enum gemini_prg_mode mode)
{
	int retval = ERROR_OK;
	uint32_t status;
	uint32_t filesize = (uint32_t)bit_file->filesize;
	uint32_t size = 1;
	uint32_t task_cmd = 0;

	LOG_INFO("[RS] Configuring Gemini FPGA fabric...");

	if ((filesize % 4) == 0)
		size = 4;
	else if ((filesize % 2) == 0)
		size = 2;

	if (gemini_write_memory(target, GEMINI_LOAD_ADDRESS, size, filesize / size, (uint8_t *)bit_file->ubi_header) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to write bitstream (%d bytes) to DDR memory at 0x%08x", filesize, GEMINI_LOAD_ADDRESS);
		return ERROR_FAIL;
	}

	LOG_DEBUG("[RS] Wrote %d bytes to DDR memory at 0x%08x", filesize, GEMINI_LOAD_ADDRESS);

	if (mode == GEMINI_PRG_MODE_SPI_FLASH)
		task_cmd = GEMINI_PRG_TSK_CMD_CFG_BITSTREAM_FLASH;
	else
		task_cmd = GEMINI_PRG_TSK_CMD_CFG_BITSTREAM_FPGA;

	if (gemini_write_reg32(target, GEMINI_SPARE_REG, 16, 0, task_cmd) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to write command %d to spare_reg at 0x%08x", task_cmd, GEMINI_SPARE_REG);
		return ERROR_FAIL;
	}

	LOG_DEBUG("[RS] Wrote command 0x%x to spare_reg at 0x%08x", task_cmd, GEMINI_SPARE_REG);

	retval = gemini_poll_command_complete_and_status(target, &status);
	if (retval != ERROR_OK)
	{
		if (retval == ERROR_TIMEOUT_REACHED)
			gemini_reset_bcpu(target); // reset target to known state when command timeout
		LOG_ERROR("[RS] Failed to program bitstream to the device");
	}
	else
		LOG_INFO("[RS] Gemini FPGA fabric is configured successfully");

	return retval;
}

static int gemini_program_device(struct pld_device *pld_device, const char *filename, enum gemini_prg_mode mode)
{
	struct gemini_pld_device *gemini_info = pld_device->driver_priv;
	gemini_bit_file_t bit_file;
	int retval = ERROR_PLD_FILE_LOAD_FAILED;

	if (gemini_read_bit_file(&bit_file, filename) != ERROR_OK)
		return ERROR_PLD_FILE_LOAD_FAILED;

	if (gemini_check_target_device(gemini_info, &bit_file) != ERROR_OK)
		goto err;

	if (gemini_switch_to_bcpu(gemini_info->target) != ERROR_OK)
		goto err;

	if (gemini_load_fsbl(gemini_info->target, &bit_file) != ERROR_OK)
		goto err;

	if (gemini_init_ddr(gemini_info->target) != ERROR_OK)
		goto err;

	if (gemini_program_bitstream(gemini_info->target, &bit_file, mode) != ERROR_OK)
		goto err;

	// everything is completed successfully
	retval = ERROR_OK;
err:
	gemini_free_bit_file(&bit_file);
	return retval;
}

static int gemini_load(struct pld_device *pld_device, const char *filename)
{
	return gemini_program_device(pld_device, filename, GEMINI_PRG_MODE_FPGA);
}

PLD_DEVICE_COMMAND_HANDLER(gemini_pld_device_command)
{
	struct gemini_pld_device *gemini_info;

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_target(CMD_ARGV[1]);
	if (!target) {
		command_print(CMD, "Target: %s does not exist", CMD_ARGV[1]);
		return ERROR_FAIL;
	}

	if (!target->tap) {
		command_print(CMD, "Target: %s is not on jtag chain", CMD_ARGV[1]);
		return ERROR_FAIL;
	}

	gemini_info = malloc(sizeof(struct gemini_pld_device));
	gemini_info->target = target;
	gemini_info->tap = target->tap;

	pld->driver_priv = gemini_info;

	return ERROR_OK;
}

COMMAND_HANDLER(gemini_handle_load_command)
{
	struct pld_device *device;
	uint32_t retval;
	unsigned int dev_id;
	enum gemini_prg_mode mode;

	if (CMD_ARGC < 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], dev_id);
	device = get_pld_device_by_num(dev_id);
	if (!device) {
		command_print(CMD, "pld device '#%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	if (!strcmp(CMD_ARGV[2], "fpga"))
		mode = GEMINI_PRG_MODE_FPGA;
	else if (!strcmp(CMD_ARGV[2], "flash"))
		mode = GEMINI_PRG_MODE_SPI_FLASH;
	else {
		command_print(CMD, "invalid mode '#%s'. supported modes are 'flash' and 'fpga'", CMD_ARGV[1]);
		return ERROR_FAIL;
	}

	retval = gemini_program_device(device, CMD_ARGV[1], mode);
	return retval;
}

static const struct command_registration gemini_exec_command_handlers[] = {
	{
		.name = "load",
		.mode = COMMAND_EXEC,
		.handler = gemini_handle_load_command,
		.help = "program/configure bitstream into spi flash or fpga fabric",
		.usage = "index filepath mode",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration gemini_command_handler[] = {
	{
		.name = "gemini",
		.mode = COMMAND_ANY,
		.help = "Rapid Silicon Gemini specific commands",
		.usage = "",
		.chain = gemini_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct pld_driver gemini_pld = {
	.name = "gemini",
	.commands = gemini_command_handler,
	.pld_device_command = &gemini_pld_device_command,
	.load = &gemini_load,
};
