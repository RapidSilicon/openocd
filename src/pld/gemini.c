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

//#define LOCAL_BUILD

#ifdef LOCAL_BUILD
#	define GEMINI_IDCODE		0x20000913
#	define GEMINI_DEBUG_CONTROL	0x80003028
#	define GEMINI_SPARE_REG		0x800030f0
#	define GEMINI_CFG_STATUS	0x80003ff4
#	define GEMINI_BUFFER_BASE	0x80000000
#	define GEMINI_WRITE_COUNTER	0x80003ff8
#	define GEMINI_READ_COUNTER	0x80003ffc
#else
#	define GEMINI_IDCODE		0x1000563d
#	define GEMINI_DEBUG_CONTROL	0xf1000028
#	define GEMINI_SPARE_REG		0xf10000f0
#	define GEMINI_CFG_STATUS	0xf10a0000
#	define GEMINI_BUFFER_BASE	0x8001DC00
#	define GEMINI_WRITE_COUNTER	0x8001FC00
#	define GEMINI_READ_COUNTER	0x8001FC04
#endif

#define GEMINI_PRODUCT_ID		0x31303050
#define GEMINI_SRAM_SIZE		261120 		// 255kb
#define GEMINI_SRAM_ADDRESS		0x80000000
#define GEMINI_BLOCK_SIZE		2048
#define GEMINI_NUM_OF_BLOCKS	4
#define GEMINI_WAIT_TIME_USEC	15000
#define GEMINI_TIMEOUT_COUNTER	10
#define GEMINI_BUFFER_ADDR(x)	(GEMINI_BUFFER_BASE + ((x % GEMINI_NUM_OF_BLOCKS) * GEMINI_BLOCK_SIZE))
#define GEMINI_ACPU				1
#define GEMINI_BCPU				0
#define GEMINI_COMMAND_POLLS	10
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

static int gemini_get_config_status(struct target * target, uint32_t *cfg_done, uint32_t *cfg_error)
{
	uint32_t cfg_status;

	if (gemini_read_reg32(target, GEMINI_CFG_STATUS, &cfg_status) != ERROR_OK)
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
		LOG_DEBUG("[RS] Poll command status #%d...", num_polls);

		retval = gemini_get_command_status(target, status);
		if (retval != ERROR_OK)
			break;
		if (*status != GEMINI_PRG_ST_PENDING)
			break;
		if (++num_polls >= GEMINI_COMMAND_POLLS)
		{
			LOG_ERROR("[RS] Timed out waiting for task to complete.");
			retval = ERROR_TIMEOUT_REACHED;
			break;
		}
		usleep(GEMINI_WAIT_TIME_USEC);
	}

	// check the command status
	if (retval == ERROR_OK)
	{
		if (*status != GEMINI_PRG_ST_TASK_COMPLETE) {
			LOG_ERROR("[RS] Command error %d", *status);
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

#ifdef LOCAL_BUILD
	// debug on local. to be removed
	if (fsbl_size > (1024 * 12))
		fsbl_size = 1024 * 12;
#endif

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

static void gemini_print_stats(struct gemini_stats *stats)
{
	uint64_t num_blocks = stats->data_sent / GEMINI_BLOCK_SIZE;
	uint64_t avg = stats->total_us / num_blocks;

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

static int gemini_reset_read_write_counters(struct target *target)
{
	if (gemini_write_reg32(target, GEMINI_WRITE_COUNTER, 32, 0, 0) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to reset write counter at 0x%08x", GEMINI_WRITE_COUNTER);
		return ERROR_FAIL;
	}

	if (gemini_write_reg32(target, GEMINI_READ_COUNTER, 32, 0, 0) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to reset read counter at 0x%08x", GEMINI_READ_COUNTER);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int gemini_stream_data_blocks(struct target *target, uint8_t *data, uint64_t data_size, struct gemini_stats *stats)
{
	int retval = ERROR_OK;
	struct duration block_duration, bop_duration;
	uint32_t read_counter;
	uint32_t write_counter = 0;
	uint32_t timeout_counter = 0;
	uint32_t block_counter = data_size / GEMINI_BLOCK_SIZE;

	// 1. halt target
	// 2. retrieve read counter
	// 3. validate read and write counters
	// 4. transfer blocks to the available cbuffer
	// 5. increment write pointer
	// 6. wait 20ms

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

		if (target_read_u32(target, GEMINI_READ_COUNTER, &read_counter) != ERROR_OK)
		{
			LOG_ERROR("[RS] Failed to retrieve read counter at 0x%08x", GEMINI_READ_COUNTER);
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
			LOG_ERROR("[RS] No. of pending buffers %d cannot be greater than %d", write_counter - read_counter, GEMINI_NUM_OF_BLOCKS);
			retval = ERROR_FAIL;
			break;
		}

		if ((write_counter - read_counter) < GEMINI_NUM_OF_BLOCKS)
		{
			uint32_t num_blocks = GEMINI_NUM_OF_BLOCKS - (write_counter - read_counter);

			for (uint32_t i = 0, num = num_blocks < block_counter ? num_blocks : block_counter; i < num; ++i)
			{
				if (target_write_memory(target, GEMINI_BUFFER_ADDR(write_counter), sizeof(uint32_t), GEMINI_BLOCK_SIZE / sizeof(uint32_t), data) != ERROR_OK)
				{
					LOG_ERROR("[RS] Failed to write a block to %x on the device", GEMINI_BUFFER_ADDR(write_counter));
					retval = ERROR_FAIL;
					break;
				}
				write_counter += 1;
				block_counter -= 1;
				stats->data_sent += GEMINI_BLOCK_SIZE;
			}

			if (retval != ERROR_OK)
				break;

			if (target_write_u32(target, GEMINI_WRITE_COUNTER, write_counter) != ERROR_OK)
			{
				LOG_ERROR("[RS] Failed to increment write counter at 0x%08x", GEMINI_WRITE_COUNTER);
				retval = ERROR_FAIL;
				break;
			}

			if (stats->log & 1)
				LOG_INFO("[RS] Progress %.2f%% (%"PRIu64"/%"PRIu64" bytes)", ((float)stats->data_sent / (float)stats->total_packages_size) * 100.0, stats->data_sent, stats->total_packages_size);

			timeout_counter = 0;
		}
		else
		{
			// circular buffer is full
			stats->cicular_buffer_full_count += 1;
			++timeout_counter;
			if (timeout_counter >= GEMINI_TIMEOUT_COUNTER)
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

static int gemini_program_bitstream(struct target *target, gemini_bit_file_t *bit_file, unsigned int progress_log)
{
	int retval = ERROR_OK;
	uint32_t cfg_done;
	uint32_t cfg_error;
	uint32_t status;
	uint8_t *bop;
	struct gemini_stats stats = { 0 };

	LOG_INFO("[RS] Configuring FPGA fabric...");

	stats.total_packages_size = gemini_get_total_packages_size(bit_file);
	stats.data_sent = 0;
	stats.package_count = 0;
	stats.cicular_buffer_full_count = 0;
	stats.wait_time_us = GEMINI_WAIT_TIME_USEC;
	stats.log = progress_log;

	bop = gemini_get_first_bop(bit_file);
	while (bop != NULL)
	{
		if (gemini_is_xcb_bop(gemini_get_bop_id(bop)) == true)
		{
			// reset read/write counters
			if (gemini_reset_read_write_counters(target) != ERROR_OK)
			{
				retval = ERROR_FAIL;
				break;
			}

			// write task cmd
			if (gemini_write_reg32(target, GEMINI_SPARE_REG, 16, 0, GEMINI_PRG_TSK_CMD_CFG_BITSTREAM_FPGA) != ERROR_OK)
			{
				LOG_ERROR("[RS] Failed to write command %d to spare_reg at 0x%08x", GEMINI_PRG_TSK_CMD_CFG_BITSTREAM_FPGA, GEMINI_SPARE_REG);
				retval = ERROR_FAIL;
				break;
			}

			// stream 2k data blocks onto device
			++stats.package_count;
			if (gemini_stream_data_blocks(target, bop, gemini_get_bop_size(bop), &stats) != ERROR_OK)
			{
				retval = ERROR_FAIL;
				break;
			}

			// check final cmd status
			if ((retval = gemini_poll_command_complete_and_status(target, &status)) != ERROR_OK)
			{
				break;
			}
		}

		bop = gemini_get_next_bop(bit_file);
	}

	if (stats.log & 2)
		gemini_print_stats(&stats);

	if (retval != ERROR_OK)
	{
		if (retval == ERROR_TIMEOUT_REACHED)
			gemini_reset_bcpu(target); // reset target to known state when command timeout
		LOG_ERROR("[RS] Failed to program bitstream to the device");
	}
	else
	{
		// check configuration done and error status
		retval = gemini_get_config_status(target, &cfg_done, &cfg_error);
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

static int gemini_program_flash(struct target *target, gemini_bit_file_t *bit_file, unsigned int progress_log)
{
	int retval = ERROR_OK;
	uint32_t status;
	struct gemini_stats stats = { 0 };
	uint64_t filesize = (uint64_t)bit_file->filesize;

	LOG_INFO("[RS] Programming SPI Flash...");

	if ((filesize % GEMINI_BLOCK_SIZE) != 0)
	{
		LOG_WARNING("[RS] Bitstream file size %" PRIu64 " is not multiple of 2k blocks", filesize);
		filesize += (GEMINI_BLOCK_SIZE - (filesize % GEMINI_BLOCK_SIZE));
	}

	stats.total_packages_size = filesize;
	stats.data_sent = 0;
	stats.package_count = 1;
	stats.cicular_buffer_full_count = 0;
	stats.wait_time_us = GEMINI_WAIT_TIME_USEC;
	stats.log = progress_log;

	// reset read/write counters
	if (gemini_reset_read_write_counters(target) != ERROR_OK)
		return ERROR_FAIL;

	// write task cmd
	if (gemini_write_reg32(target, GEMINI_SPARE_REG, 16, 0, GEMINI_PRG_TSK_CMD_CFG_BITSTREAM_FLASH) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to write command %d to spare_reg at 0x%08x", GEMINI_PRG_TSK_CMD_CFG_BITSTREAM_FLASH, GEMINI_SPARE_REG);
		return ERROR_FAIL;
	}

	// stream 2k data block to device
	if (gemini_stream_data_blocks(target, (uint8_t *)bit_file->ubi_header, filesize, &stats) != ERROR_OK)
		return ERROR_FAIL;

	if (stats.log & 2)
		gemini_print_stats(&stats);

	// check cmd status
	retval = gemini_poll_command_complete_and_status(target, &status);
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

static int gemini_program_device(struct pld_device *pld_device, const char *filename, enum gemini_prg_mode mode, unsigned int progress_log)
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

	if (mode == GEMINI_PRG_MODE_FPGA)
		if (gemini_program_bitstream(gemini_info->target, &bit_file, progress_log) != ERROR_OK)
			goto err;

	if (mode == GEMINI_PRG_MODE_SPI_FLASH)
		if (gemini_program_flash(gemini_info->target, &bit_file, progress_log) != ERROR_OK)
			goto err;

	// everything is completed successfully
	retval = ERROR_OK;
err:
	gemini_free_bit_file(&bit_file);
	return retval;
}

static int gemini_load(struct pld_device *pld_device, const char *filename)
{
	return gemini_program_device(pld_device, filename, GEMINI_PRG_MODE_FPGA, 0);
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
	struct timeval start, end, duration;
	struct pld_device *device;
	int retval;
	unsigned int progress_log = 0;
	unsigned int dev_id;
	enum gemini_prg_mode mode;

	gettimeofday(&start, NULL);

	if (CMD_ARGC < 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], dev_id);
	device = get_pld_device_by_num(dev_id);
	if (!device) {
		command_print(CMD, "pld device '#%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	if (!strcmp(CMD_ARGV[1], "fpga"))
		mode = GEMINI_PRG_MODE_FPGA;
	else if (!strcmp(CMD_ARGV[1], "flash"))
		mode = GEMINI_PRG_MODE_SPI_FLASH;
	else {
		command_print(CMD, "invalid mode '#%s'. supported modes are 'flash' and 'fpga' only", CMD_ARGV[1]);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (CMD_ARGC > 3)
	{
		if (!strcmp(CMD_ARGV[3], "-p"))
		{
			if (CMD_ARGC > 4)
				COMMAND_PARSE_NUMBER(uint, CMD_ARGV[4], progress_log);
			else
				progress_log = 1;
		}
		else
		{
			command_print(CMD, "invalid option %s", CMD_ARGV[3]);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	retval = gemini_program_device(device, CMD_ARGV[2], mode, progress_log);
	if (retval != ERROR_OK)
		command_print(CMD, "failed loading file %s to pld device %u", CMD_ARGV[2], dev_id);
	else {
		gettimeofday(&end, NULL);
		timeval_subtract(&duration, &end, &start);
		command_print(CMD, "loaded file %s to pld device %u in %jis %jius", CMD_ARGV[2], dev_id,
			(intmax_t)duration.tv_sec, (intmax_t)duration.tv_usec);
	}

	return retval;
}

static char *get_product_name_by_idcode(uint32_t idcode)
{
	switch (idcode)
	{
		case GEMINI_IDCODE:
			return "Gemini";
		default:
			break;
	}
	return "Unknown";
}

COMMAND_HANDLER(gemini_handle_get_cfg_status_command)
{
	struct pld_device *device = NULL;
	struct gemini_pld_device *gemini_device = NULL;
	unsigned int dev_id;
	uint32_t cfg_done;
	uint32_t cfg_error;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], dev_id);
	device = get_pld_device_by_num(dev_id);
	if (!device) {
		command_print(CMD, "pld device '#%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	gemini_device = (struct gemini_pld_device *)(device->driver_priv);

	if (gemini_get_config_status(gemini_device->target, &cfg_done, &cfg_error) != ERROR_OK)
		return ERROR_FAIL;

	// print header
	command_print(CMD, "            Device               cfg_done   cfg_error ");
	command_print(CMD, "----------- -------------------- ---------- ----------");

	// print cfg done and error status
	command_print(CMD, "Found %5d %-20s %-10d %-10d", dev_id, get_product_name_by_idcode(gemini_device->tap->idcode), cfg_done, cfg_error);

	return ERROR_OK;
}

COMMAND_HANDLER(gemini_handle_list_device_command)
{
	struct pld_device *device = NULL;
	struct gemini_pld_device *gemini_device = NULL;
	int i = 0;

	// print header
	command_print(CMD, "         Device               ID           IRLen      Flash Size      ");
	command_print(CMD, "-------- -------------------- ------------ ---------- ----------------");

	while (1)
	{
		device = get_pld_device_by_num(i);
		if (!device)
			break;

		if (strcmp(device->driver->name, "gemini") != 0)
			continue;

		gemini_device = (struct gemini_pld_device *)(device->driver_priv);
		if (gemini_device->tap)
		{
			// print device details
			command_print(CMD, "Found %2d %-20s 0x%-10x %-10d %d", i, get_product_name_by_idcode(gemini_device->tap->idcode),
							gemini_device->tap->idcode,
							gemini_device->tap->ir_length,
							1024*16 /*place holder*/);
		}
		++i;
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
		.usage = "index",
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
