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

#define LOCAL_BUILD
//#define PROTOTYPE_BUILD

#ifdef LOCAL_BUILD
#	define GEMINI_IDCODE			0x20000913
#else
#	define GEMINI_IDCODE			0x1000563d
#endif

#define GEMINI_PRODUCT_ID			0x31303050
#define GEMINI_BLOCK_SIZE			2048
#define GEMINI_NUM_OF_BLOCKS		4
#define GEMINI_TIMEOUT_COUNTER		500
#define GEMINI_WAIT_TIME_US			30000
#define GEMINI_BUFFER_ADDR(base, x)	(base + ((x % GEMINI_NUM_OF_BLOCKS) * GEMINI_BLOCK_SIZE))
#define GEMINI_ACPU					1
#define GEMINI_BCPU					0
#define DM_SBCS						0x38
#define DM_SBADDRESS0				0x39
#define DM_SBDATA0					0x3c
#define DDR_INIT					1
#define DDR_NOT_INIT				0
#define MSEC(miliseconds)			(miliseconds * 1000) // to microseconds
#define STATUS(x)					((x >> 10) & 0x3f)

enum gemini_prg_mode {
	GEMINI_PRG_MODE_FPGA,
	GEMINI_PRG_MODE_SPI_FLASH,
	GEMINI_PRG_MODE_OTP
};

struct device_t device_table[] =
{
#if defined(LOCAL_BUILD)
    {
		.name           = "gemini",
		.debug_control  = 0x80003028,
		.spare_reg      = 0x800030f0,
		.cfg_status     = 0x80003ff4,
		.fsbl_ubi_addr  = 0x80000000,
		.ram_size       = 261120, /* 255kb SRAM */
		.cbuffer        = 0x80000000,
		.read_counter   = 0x80003ffc,
		.write_counter  = 0x80003ff8,
	},
#elif defined(PROTOTYPE_BUILD)
    {
		.name           = "gemini",
		.debug_control  = 0xf1000028,
		.spare_reg      = 0x8003DDF4,
		.cfg_status     = 0xf10a0000,
		.fsbl_ubi_addr  = 0x80000000,
		.ram_size       = 131072, /* 128kb SRAM */
		.cbuffer        = 0x8003DDF8,
		.read_counter   = 0x8003FDFC,
		.write_counter  = 0x8003FDF8,
	},
#else
    {
		.name           = "gemini",
		.debug_control  = 0xf1000028,
		.spare_reg      = 0xf10000f0,
		.cfg_status     = 0xf10a0000,
		.fsbl_ubi_addr  = 0x80000000,
		.ram_size       = 261120, /* 255k SRAM */
		.cbuffer        = 0x8003DDF8,
		.read_counter   = 0x8003FDFC,
		.write_counter  = 0x8003FDF8,
	},
    {
		.name           = "virgo",
		.debug_control  = 0xa0110028,
		.spare_reg      = 0xa01100f0,
		.cfg_status     = 0xa0710000,
		.fsbl_ubi_addr  = 0xA0200000,
		.ram_size       = 65536, /* 64kb ILM */
		.cbuffer        = 0xA040DFF8,
		.read_counter   = 0xA040FFFC,
		.write_counter  = 0xA040FFF8,
	},
#endif
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

	if (width > 32 || offset > 32 || (width + offset) > 32)
		return ERROR_FAIL;

	if (target_halt(target) == ERROR_OK) 
	{
		if (width < 32)
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
		}
		else
		{
			if (target_write_u32(target, address, value) != ERROR_OK)
			{
				LOG_WARNING("[RS] Failed to write to address 0x%08x", (uint32_t)address);
				retval = ERROR_FAIL;
			}
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

static int gemini_get_device_type(char *typename, struct device_t **device)
{
	for (uint32_t i = 0; i < (sizeof(device_table) / sizeof(struct device_t)); ++i)
	{
		if (!strcmp(typename, device_table[i].name))
		{
			*device = &device_table[i];
			return ERROR_OK;
		}
	}
	return ERROR_FAIL;
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

static int gemini_get_cpu_type(struct target *target, struct device_t *device, uint32_t *cpu_type)
{
	uint32_t debug_control;

	if (gemini_read_reg32(target, device->debug_control, &debug_control) == ERROR_OK)
	{
		*cpu_type = debug_control & 0x1 ? GEMINI_ACPU : GEMINI_BCPU;
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

static int gemini_get_firmware_type(struct target * target, struct device_t *device, uint32_t *fw_type)
{
	uint32_t spare_reg;

	if (gemini_read_reg32(target, device->spare_reg, &spare_reg) == ERROR_OK)
	{
		spare_reg >>= 29;
		if (spare_reg == PRG_REG_FW_TYPE_BOOTROM || spare_reg == PRG_REG_FW_TYPE_CFG_FSBL)
		{
			*fw_type = spare_reg;
			return ERROR_OK;
		}
	}

	return ERROR_FAIL;
}

static int gemini_get_command_status(struct target *target, struct device_t *device, uint32_t *status)
{
	uint32_t spare_reg;

	if (gemini_read_reg32(target, device->spare_reg, &spare_reg) == ERROR_OK)
	{
		*status = STATUS(spare_reg);
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

static int gemini_check_target_device(struct target *target, gemini_bit_file_t *bit_file)
{
	if (target->tap->idcode != GEMINI_IDCODE)
	{
		LOG_ERROR("[RS] Unknown JTAG ID 0x%08x", target->tap->idcode);
		return ERROR_FAIL;
	}

	if (bit_file->ubi_header->product_id != GEMINI_PRODUCT_ID)
	{
		LOG_ERROR("[RS] Invalid Gemini Product ID in the bitstream file");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int gemini_get_config_status(struct target *target, struct device_t *device, uint32_t *cfg_done, uint32_t *cfg_error)
{
	uint32_t cfg_status = 0;

	if (gemini_read_reg32(target, device->cfg_status, &cfg_status) != ERROR_OK)
		return ERROR_FAIL;

	*cfg_done = (cfg_status >> 7) & 0x1;
	*cfg_error = (cfg_status >> 15) & 0x1;
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

static int gemini_switch_to_bcpu(struct target *target, struct device_t *device)
{
	uint32_t cpu_type;

	if (gemini_get_cpu_type(target, device, &cpu_type) != ERROR_OK)
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

	if (gemini_sysbus_write_reg32(target, device->debug_control, 0) != ERROR_OK)
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

	if (gemini_get_cpu_type(target, device, &cpu_type) != ERROR_OK || cpu_type == GEMINI_ACPU)
	{
		LOG_INFO("[RS] Failed to switch to BCPU");
		return ERROR_FAIL;
	}

	LOG_INFO("[RS] Switched to BCPU");

	return ERROR_OK;
}

static int gemini_poll_command_complete_and_status(struct target *target, struct device_t *device, uint32_t *status, uint32_t wait_time_us, uint32_t num_polls)
{
	int retval = ERROR_OK;

	while (1)
	{
		retval = gemini_get_command_status(target, device, status);
		if (retval != ERROR_OK)
			break;
		if (*status != PRG_REG_ST_PENDING)
			break;
		if (num_polls == 0) {
			LOG_ERROR("[RS] Timed out waiting for task to complete.");
			retval = ERROR_TIMEOUT_REACHED;
			break;
		}
		--num_polls;
		usleep(wait_time_us);
	}

	// check the command status
	if (retval == ERROR_OK)
	{
		if (*status != PRG_REG_ST_TASK_COMPLETE) {
			LOG_ERROR("[RS] Command error %d", *status);
			retval = ERROR_FAIL;
		}
	}
	// clear command and status field
	gemini_write_reg32(target, device->spare_reg, 16, 0, PRG_REG_TSK_CMD_IDLE);

	return retval;
}

static int gemini_load_fsbl(struct target *target, struct device_t *device, gemini_bit_file_t *bit_file)
{
	int retval = ERROR_OK;
	uint32_t fsbl_size = bit_file->ubi_header->fsbl_size;
	uint32_t size = 1;
	uint32_t status;
	uint32_t fw_type;

	if (gemini_get_firmware_type(target, device, &fw_type) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to determine the firmware type");
		return ERROR_FAIL;
	}

	if (fw_type == PRG_REG_FW_TYPE_CFG_FSBL)
	{
		LOG_INFO("[RS] FSBL firmware type detected");
		return ERROR_OK;
	}

	LOG_INFO("[RS] Loading FSBL firmware...");

	if (fsbl_size == 0)
	{
		LOG_ERROR("[RS] FSBL package not found in the bitstream");
		return ERROR_FAIL;
	}

	if (fsbl_size > device->ram_size)
	{
		LOG_ERROR("[RS] FSBL package (%d bytes) is larger than the available SRAM size (%d bytes)", fsbl_size, device->ram_size);
		return ERROR_FAIL;
	}

#ifdef LOCAL_BUILD
	// debug on local. to be removed
	if (fsbl_size > (1024 * 12))
		fsbl_size = 1024 * 12;
#endif

	if ((fsbl_size % 4) == 0)
		size = 4;
	else if ((fsbl_size % 2) == 0)
		size = 2;

	retval = gemini_write_memory(target, device->fsbl_ubi_addr, size, fsbl_size / size, (uint8_t *)bit_file->ubi_header);
	if (retval != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to write bitstream of %d bytes to SRAM at 0x%08" PRIxPTR, fsbl_size, device->fsbl_ubi_addr);
		return ERROR_FAIL;
	}

	if (gemini_write_reg32(target, device->spare_reg, 16, 0, PRG_REG_TSK_CMD_BBF_FDI) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to write command 0x%x to spare_reg at 0x%08" PRIxPTR, PRG_REG_TSK_CMD_BBF_FDI, device->spare_reg);
		return ERROR_FAIL;
	}

	retval = gemini_poll_command_complete_and_status(target, device, &status, MSEC(1000), 5);
	if (retval == ERROR_OK)
	{
		// double check if the firmware type is FSBL
		if (gemini_get_firmware_type(target, device, &fw_type) != ERROR_OK || fw_type != PRG_REG_FW_TYPE_CFG_FSBL)
			retval = ERROR_FAIL;
	}

	if (retval != ERROR_OK)
	{
		if (retval == ERROR_TIMEOUT_REACHED)
			gemini_reset_bcpu(target); // reset target to known state when command timeout
		LOG_ERROR("[RS] Failed to load FSBL firmware");
	}
	else
		LOG_INFO("[RS] Boot up FSBL firmware of size %d bytes successfully.", fsbl_size);

	return retval;
}

static void gemini_print_stats(struct gemini_stats *stats)
{
	uint64_t num_blocks = stats->data_sent / GEMINI_BLOCK_SIZE;
	uint64_t avg = num_blocks > 0 ? stats->total_us / num_blocks : 0;

	LOG_INFO("[RS] -- Statistic -----------------------------------------------------------");
	LOG_INFO("[RS]    1. total_packages_size        : %" PRIu64, stats->total_packages_size);
	LOG_INFO("[RS]    2. package_count              : %d" , stats->package_count);
	LOG_INFO("[RS]    3. data_sent                  : %" PRIu64, stats->data_sent);
	LOG_INFO("[RS]    4. cicular_buffer_full_count  : %d" , stats->cicular_buffer_full_count);
	LOG_INFO("[RS]    5. total_blocks               : %" PRIu64, num_blocks);
	LOG_INFO("[RS]    6. total_us                   : %" PRIu64 " (%f sec)", stats->total_us, stats->total_us / 1000000.0);
	LOG_INFO("[RS]    7. total_overall_us           : %" PRIu64 " (%f sec)", stats->total_overall_us, stats->total_overall_us / 1000000.0);
	LOG_INFO("[RS]    8. total_wait_us              : %" PRIu64 " (%f sec)", stats->total_overall_us - stats->total_us, (stats->total_overall_us - stats->total_us) / 1000000.0);
	LOG_INFO("[RS]    9. average us @ block         : %" PRIu64 " (%f sec)", avg, avg / 1000000.0);
	LOG_INFO("[RS]   10. transfer rate              : %.5f kbps", stats->data_sent / (1024.0 * (stats->total_overall_us / 1000000.0)));
}

static int gemini_reset_read_write_counters(struct target *target, struct device_t *device)
{
	if (gemini_write_reg32(target, device->write_counter, 32, 0, 0) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to reset write counter at 0x%08" PRIxPTR, device->write_counter);
		return ERROR_FAIL;
	}

	if (gemini_write_reg32(target, device->read_counter, 32, 0, 0) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to reset read counter at 0x%08" PRIxPTR, device->read_counter);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int gemini_stream_data_blocks(struct target *target, struct device_t *device, uint8_t *data, uint64_t data_size, uint32_t task_id, struct gemini_stats *stats)
{
	int retval = ERROR_OK;
	struct duration block_duration, bop_duration;
	uint32_t read_counter;
	uint32_t write_counter = 0;
	uint32_t timeout_counter = 0;
	uint32_t block_counter = data_size / GEMINI_BLOCK_SIZE;
	uint32_t spare_reg = 0;

	// reset read/write counters
	if (gemini_reset_read_write_counters(target, device) != ERROR_OK)
	{
		return ERROR_FAIL;
	}

	// write task cmd
	if (gemini_write_reg32(target, device->spare_reg, 16, 0, task_id) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to write command %d to spare_reg at 0x%08" PRIxPTR, task_id, device->spare_reg);
		return ERROR_FAIL;
	}

	duration_start(&bop_duration);

	while (block_counter > 0)
	{
		duration_start(&block_duration);

		if (target_halt(target) != ERROR_OK)
		{
			LOG_ERROR("[RS] Failed to halt the BPU core");
			retval = ERROR_FAIL;
			break;
		}

		if (target_read_u32(target, device->read_counter, &read_counter) != ERROR_OK)
		{
			LOG_ERROR("[RS] Failed to retrieve read counter at 0x%08" PRIxPTR, device->read_counter);
			retval = ERROR_FAIL;
			break;
		}

		if (read_counter > write_counter)
		{
			LOG_ERROR("[RS] Read counter (%d) cannot be greater than write counter (%d)", read_counter, write_counter);
			retval = ERROR_FAIL;
			break;
		}

		if ((write_counter - read_counter) > GEMINI_NUM_OF_BLOCKS)
		{
			LOG_ERROR("[RS] The available blocks (%d) is greater than %d", write_counter - read_counter, GEMINI_NUM_OF_BLOCKS);
			retval = ERROR_FAIL;
			break;
		}

		if ((write_counter - read_counter) < GEMINI_NUM_OF_BLOCKS)
		{
			uint32_t num_blocks = GEMINI_NUM_OF_BLOCKS - (write_counter - read_counter);

			for (uint32_t i = 0, num = num_blocks < block_counter ? num_blocks : block_counter; i < num; ++i)
			{
				if (target_write_memory(target, GEMINI_BUFFER_ADDR(device->cbuffer, write_counter), sizeof(uint32_t), GEMINI_BLOCK_SIZE / sizeof(uint32_t), data) != ERROR_OK)
				{
					LOG_ERROR("[RS] Failed to write a block to 0x%08" PRIxPTR " on the device", GEMINI_BUFFER_ADDR(device->cbuffer, write_counter));
					retval = ERROR_FAIL;
					break;
				}
				write_counter += 1;
				block_counter -= 1;
				stats->data_sent += GEMINI_BLOCK_SIZE;
				data += GEMINI_BLOCK_SIZE;
			}

			if (retval != ERROR_OK)
				break;

			if (target_write_u32(target, device->write_counter, write_counter) != ERROR_OK)
			{
				LOG_ERROR("[RS] Failed to increment write counter at 0x%08" PRIxPTR, device->write_counter);
				retval = ERROR_FAIL;
				break;
			}

			if (stats->log & 1) {
				float progress = ((float)stats->data_sent / (float)stats->total_packages_size) * 100.0;
				LOG_INFO("[RS] Progress %.2f%% (%"PRIu64"/%"PRIu64" bytes)", progress, stats->data_sent, stats->total_packages_size);
			}

			timeout_counter = 0;
		}
		else
		{
			// circular buffer is full
			stats->cicular_buffer_full_count += 1;
			++timeout_counter;

			// check cmd status
			if (target_read_u32(target, device->spare_reg, &spare_reg) == ERROR_OK && STATUS(spare_reg) != 0)
			{
				LOG_ERROR("[RS] Command error %d.", STATUS(spare_reg));
				retval = ERROR_FAIL;
				break;
			}

			if (timeout_counter >= stats->timeout_counter)
			{
				LOG_ERROR("[RS] Circular buffer timed out.");
				retval = ERROR_TIMEOUT_REACHED;
				break;
			}
		}

		if (target_resume(target, true, 0, true, false) != ERROR_OK)
		{
			LOG_ERROR("[RS] Failed to resume the BPU core");
			retval = ERROR_FAIL;
			break;
		}

		// take some statistics
		duration_measure(&block_duration);
		stats->total_us += (block_duration.elapsed.tv_usec + (block_duration.elapsed.tv_sec * 1000000));
		usleep(stats->wait_time_us);
	}

	duration_measure(&bop_duration);
	stats->total_overall_us += (bop_duration.elapsed.tv_usec + (bop_duration.elapsed.tv_sec * 1000000));

	if (target->halt_issued == true)
		target_resume(target, true, 0, true, false);

	return retval;
}

static int gemini_program_bitstream(struct target *target, struct device_t *device, gemini_bit_file_t *bit_file, unsigned int log_level)
{
	int retval = ERROR_OK;
	uint32_t cfg_done, cfg_error;
	uint32_t status;
	uint8_t *bop;
	struct gemini_stats option = { 0 };

	LOG_INFO("[RS] Configuring FPGA fabric...");

	if (gemini_get_total_packages_size(bit_file, GEMINI_BLOCK_SIZE, BOP_FPGA, &option.total_packages_size) != 0)
	{
		LOG_ERROR("[RS] Bitstream BOP size is not multiple of %d bytes", GEMINI_BLOCK_SIZE);
		return ERROR_FAIL;
	}

	if (option.total_packages_size == 0)
	{
		LOG_ERROR("[RS] Bitstream doens't contain any FPGA configuration BOP");
		return ERROR_FAIL;
	}

	option.timeout_counter = GEMINI_TIMEOUT_COUNTER;
	option.wait_time_us = GEMINI_WAIT_TIME_US;
	option.log = log_level;

	bop = gemini_get_first_bop(bit_file);
	while (bop != NULL)
	{
		if (gemini_is_bop_type(bop, BOP_FPGA) == true)
		{
			if ((retval = gemini_stream_data_blocks(target, device, bop, gemini_get_bop_size(bop), PRG_REG_TSK_CMD_CFG_BITSTREAM_FPGA, &option)) != ERROR_OK)
			{
				break;
			}

			if ((retval = gemini_poll_command_complete_and_status(target, device, &status, option.wait_time_us, option.timeout_counter)) != ERROR_OK)
			{
				break;
			}
			option.package_count += 1;
		}

		bop = gemini_get_next_bop(bit_file);
	}

	if (option.log & 2)
		gemini_print_stats(&option);

	if (retval != ERROR_OK)
	{
		if (retval == ERROR_TIMEOUT_REACHED)
			gemini_reset_bcpu(target); // reset target to known state when command timeout
		LOG_ERROR("[RS] Failed to program bitstream to the device");
	}
	else
	{
		// check configuration done and error status
		retval = gemini_get_config_status(target, device, &cfg_done, &cfg_error);
		if (retval == ERROR_OK)
		{
			if (cfg_done == 1 && cfg_error == 0)
				LOG_INFO("[RS] Configured FPGA fabric successfully");
			else
			{
				LOG_ERROR("[RS] FPGA fabric configuration error (cfg_done = %d, cfg_error = %d)", cfg_done, cfg_error);
				retval = ERROR_FAIL;
			}
		}
	}

	return retval;
}

static int gemini_program_flash(struct target *target, struct device_t *device, gemini_bit_file_t *bit_file, unsigned int log_level)
{
	int retval = ERROR_OK;
	uint32_t status;
	struct gemini_stats option = { 0 };
	uint64_t filesize = (uint64_t)bit_file->filesize;

	LOG_INFO("[RS] Programming SPI Flash...");

	if ((filesize % GEMINI_BLOCK_SIZE) != 0)
	{
		LOG_ERROR("[RS] Bitstream file size %" PRIu64 " is not multiple of %d bytes", filesize, GEMINI_BLOCK_SIZE);
		return ERROR_FAIL;
	}

	option.total_packages_size = filesize;
	option.package_count = 1;
	option.timeout_counter = GEMINI_TIMEOUT_COUNTER;
	option.wait_time_us = GEMINI_WAIT_TIME_US;
	option.log = log_level;

	retval = gemini_stream_data_blocks(target, device, (uint8_t *)bit_file->ubi_header, filesize, PRG_REG_TSK_CMD_CFG_BITSTREAM_FLASH, &option);
	if (retval == ERROR_OK)
	{
		retval = gemini_poll_command_complete_and_status(target, device, &status, option.wait_time_us, option.timeout_counter);
	}

	if (option.log & 2)
		gemini_print_stats(&option);

	if (retval != ERROR_OK)
	{
		if (retval == ERROR_TIMEOUT_REACHED)
			gemini_reset_bcpu(target); // reset target to known state when command timeout
		LOG_ERROR("[RS] Failed to program SPI flash");
	}
	else
	{
		LOG_INFO("[RS] Programmed SPI Flash successfully");
	}

	return retval;
}

static int gemini_program_otp(struct target *target, struct device_t *device, gemini_bit_file_t *bit_file, unsigned int log_level)
{
	int retval = ERROR_OK;
	uint32_t status;
	uint8_t *bop;
	struct gemini_stats option = { 0 };

	LOG_INFO("[RS] Programming OTP...");

	if (gemini_get_total_packages_size(bit_file, GEMINI_BLOCK_SIZE, BOP_KEY_CERT, &option.total_packages_size) != 0)
	{
		LOG_ERROR("[RS] Bitstream BOP size is not multiple of %d bytes", GEMINI_BLOCK_SIZE);
		return ERROR_FAIL;
	}

	if (option.total_packages_size == 0)
	{
		LOG_ERROR("[RS] Bitstream doens't contain any OTP configuration BOP");
		return ERROR_FAIL;
	}

	option.timeout_counter = GEMINI_TIMEOUT_COUNTER;
	option.wait_time_us = GEMINI_WAIT_TIME_US;
	option.log = log_level;

	bop = gemini_get_first_bop(bit_file);
	while (bop != NULL)
	{
		if (gemini_is_bop_type(bop, BOP_KEY_CERT) == true)
		{
			if ((retval = gemini_stream_data_blocks(target, device, bop, gemini_get_bop_size(bop), PRG_REG_TSK_CMD_CFG_OTP, &option)) != ERROR_OK)
			{
				break;
			}

			if ((retval = gemini_poll_command_complete_and_status(target, device, &status, option.wait_time_us, option.timeout_counter)) != ERROR_OK)
			{
				break;
			}
			option.package_count += 1;
		}

		bop = gemini_get_next_bop(bit_file);
	}

	if (retval != ERROR_OK)
	{
		if (retval == ERROR_TIMEOUT_REACHED)
			gemini_reset_bcpu(target); // reset target to known state when command timeout
		LOG_ERROR("[RS] Failed to program OTP");
	}
	else
	{
		LOG_INFO("[RS] Programmed OTP successfully");
	}

	return retval;
}

static int gemini_program_device(struct target *target, struct device_t *device, const char *filename, enum gemini_prg_mode mode, unsigned int progress_log)
{
	gemini_bit_file_t bit_file;
	int retval = ERROR_PLD_FILE_LOAD_FAILED;

	if (gemini_read_bit_file(&bit_file, filename) != ERROR_OK)
		return ERROR_PLD_FILE_LOAD_FAILED;

	if (gemini_check_target_device(target, &bit_file) != ERROR_OK)
		goto err;

	if (gemini_switch_to_bcpu(target, device) != ERROR_OK)
		goto err;

	if (gemini_load_fsbl(target, device, &bit_file) != ERROR_OK)
		goto err;

	if (mode == GEMINI_PRG_MODE_FPGA)
		if (gemini_program_bitstream(target, device, &bit_file, progress_log) != ERROR_OK)
			goto err;

	if (mode == GEMINI_PRG_MODE_SPI_FLASH)
		if (gemini_program_flash(target, device, &bit_file, progress_log) != ERROR_OK)
			goto err;

	if (mode == GEMINI_PRG_MODE_OTP)
		if (gemini_program_otp(target, device, &bit_file, progress_log) != ERROR_OK)
			goto err;

	// everything is completed successfully
	retval = ERROR_OK;
err:
	gemini_free_bit_file(&bit_file);
	return retval;
}

static int gemini_load(struct pld_device *pld_device, const char *filename)
{
	return ERROR_NOT_IMPLEMENTED;
}

static struct pld_device *gemini_get_pld_device_driver(void)
{
	struct pld_device *device = NULL;
	int i = 0;

	while (1)
	{
		// locate pld driver named 'gemini' by index start from 0 and by right only one gemini pld driver
		device = get_pld_device_by_num(i++);
		if (!device) break;
		if (strcmp(device->driver->name, "gemini") != 0) {
			continue;
		}
		break;
	}

	return device;
}

PLD_DEVICE_COMMAND_HANDLER(gemini_pld_device_command)
{
	struct gemini_pld_device *gemini_info;
	int ret = ERROR_OK;

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	// allocate memory for devices
	gemini_info = malloc(sizeof(struct gemini_pld_device));
	gemini_info->count = CMD_ARGC - 1;
	gemini_info->target_info = malloc(sizeof(struct target_info_t) * gemini_info->count);

	// enumerate all passed in targets
	for (uint32_t i = 0; i < gemini_info->count; ++i)
	{
		struct target *target = get_target(CMD_ARGV[i+1]);
		if (!target || !target->tap)
		{
			command_print(CMD, "Target: %s is invalid", CMD_ARGV[i+1]);
			ret = ERROR_FAIL;
			break;
		}

		gemini_info->target_info[i].target = target;
		gemini_info->target_info[i].tap = target->tap;
	}

	if (ret == ERROR_OK)
	{
		pld->driver_priv = gemini_info;
	}
	else
	{
		free(gemini_info->target_info);
		free(gemini_info);
	}

	return ret;
}

static char *get_cmdline_option(struct command_invocation *cmd, char *optname, char *defvalue)
{
	char *optval = defvalue;

	for (unsigned int i = 0; i < CMD_ARGC; ++i) {
		if (!strcmp(CMD_ARGV[i], optname)) {
			if ((i + 1) < CMD_ARGC)
				optval = (char *)CMD_ARGV[i+1];
			break;
		}
	}

	return optval;
}

COMMAND_HANDLER(gemini_handle_load_command)
{
	struct timeval start, end, duration;
	struct pld_device *pld_device;
	struct gemini_pld_device *gemini_device = NULL;
	struct device_t *device = NULL;
	int retval;
	unsigned int progress_log = 0;
	unsigned int index;
	enum gemini_prg_mode mode;

	gettimeofday(&start, NULL);

	if (CMD_ARGC < 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	pld_device = gemini_get_pld_device_driver();
	if (!pld_device) {
		command_print(CMD, "pld device driver 'gemini' not found");
		return ERROR_FAIL;
	}

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], index);
	gemini_device = (struct gemini_pld_device *)(pld_device->driver_priv);
	// user input is 1-based indexing. internally is 0-based indexing
	if (index == 0 || (index - 1) >= gemini_device->count) {
		command_print(CMD, "device index '#%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	if (!strcmp(CMD_ARGV[1], "fpga"))
		mode = GEMINI_PRG_MODE_FPGA;
	else if (!strcmp(CMD_ARGV[1], "flash"))
		mode = GEMINI_PRG_MODE_SPI_FLASH;
	else if (!strcmp(CMD_ARGV[1], "otp"))
		mode = GEMINI_PRG_MODE_OTP;
	else {
		command_print(CMD, "invalid mode '#%s'. supported modes are 'fpga', 'flash' and 'otp' only", CMD_ARGV[1]);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (gemini_get_device_type(get_cmdline_option(CMD, "-d", "gemini"), &device) != ERROR_OK)
	{
		command_print(CMD, "invalid device '%s'", get_cmdline_option(CMD, "-d", NULL));
		return ERROR_FAIL;
	}

	COMMAND_PARSE_NUMBER(uint, get_cmdline_option(CMD, "-p", "0"), progress_log);

	retval = gemini_program_device(gemini_device->target_info[index - 1].target, device, CMD_ARGV[2], mode, progress_log);
	if (retval != ERROR_OK) {
		command_print(CMD, "failed loading file %s to device %u", CMD_ARGV[2], index);
	} else {
		gettimeofday(&end, NULL);
		timeval_subtract(&duration, &end, &start);
		command_print(CMD, "loaded file %s to device %u in %jis %jius", CMD_ARGV[2], index,
			(intmax_t)duration.tv_sec, (intmax_t)duration.tv_usec);
	}

	return retval;
}

COMMAND_HANDLER(gemini_handle_get_cfg_status_command)
{
	struct pld_device *pld_device = NULL;
	struct gemini_pld_device *gemini_device = NULL;
	struct device_t *device = NULL;
	uint32_t cfg_done;
	uint32_t cfg_error;
	unsigned int index = 0;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	pld_device = gemini_get_pld_device_driver();
	if (!pld_device) {
		command_print(CMD, "pld device driver 'gemini' not found");
		return ERROR_FAIL;
	}

	// check if index is not out of bound
	gemini_device = (struct gemini_pld_device *)(pld_device->driver_priv);
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], index);
	// user input is 1-based indexing. internally is 0-based indexing
	if (index == 0 || (index - 1) >= gemini_device->count)
	{
		command_print(CMD, "device index '#%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	if (gemini_get_device_type(get_cmdline_option(CMD, "-d", "gemini"), &device) != ERROR_OK)
	{
		command_print(CMD, "invalid device '%s'", get_cmdline_option(CMD, "-d", NULL));
		return ERROR_FAIL;
	}

	if (gemini_get_config_status(gemini_device->target_info[index - 1].target, device, &cfg_done, &cfg_error) != ERROR_OK)
		return ERROR_FAIL;

	// print header
	command_print(CMD, "      | Index  | Device                                  | cfg_done | cfg_error");
	command_print(CMD, "------ -------- ----------------------------------------- ---------- ----------");

	// print cfg done and error status
	command_print(CMD, "Found   %-7d  %-40s  %-9d  %d", index, device->name, cfg_done, cfg_error);

	return ERROR_OK;
}

COMMAND_HANDLER(gemini_handle_list_device_command)
{
	struct pld_device *pld_device = NULL;
	struct gemini_pld_device *gemini_device = NULL;
	struct device_t *device = NULL;

	pld_device = gemini_get_pld_device_driver();
	if (!pld_device) {
		command_print(CMD, "pld device driver 'gemini' not found");
		return ERROR_FAIL;
	}

	if (gemini_get_device_type(get_cmdline_option(CMD, "-d", "gemini"), &device) != ERROR_OK)
	{
		command_print(CMD, "invalid device '%s'", get_cmdline_option(CMD, "-d", NULL));
		return ERROR_FAIL;
	}

	// print header
	command_print(CMD, "      | Index  | Device                                  | ID         | IRLen    | Flash Size      ");
	command_print(CMD, "------ -------- ----------------------------------------- ------------ ---------- -----------------");

	gemini_device = (struct gemini_pld_device *)(pld_device->driver_priv);
	for (uint32_t i = 0; i < gemini_device->count; ++i)
	{
		// print device details
		command_print(CMD, "Found   %-7d  %-40s  0x%-9x  %-9d  %d", i + 1, device->name, gemini_device->target_info[i].tap->idcode,
						gemini_device->target_info[i].tap->ir_length, 1024*16 /*place holder*/);
	}

	return ERROR_OK;
}

static const struct command_registration gemini_exec_command_handlers[] = {
	{
		.name = "load",
		.mode = COMMAND_EXEC,
		.handler = gemini_handle_load_command,
		.help = "program/configure bitstream into spi flash or fpga fabric",
		.usage = "index mode filepath -p <progress level>",
	},
	{
		.name = "status",
		.mode = COMMAND_EXEC,
		.handler = gemini_handle_get_cfg_status_command,
		.help = "get fpga configration done and error status",
		.usage = "index",
	},
	{
		.name = "list",
		.mode = COMMAND_EXEC,
		.handler = gemini_handle_list_device_command,
		.help = "list all devices",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration gemini_command_handler[] = {
	{
		.name = "gemini",
		.mode = COMMAND_ANY,
		.help = "Rapid Silicon device specific commands",
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
