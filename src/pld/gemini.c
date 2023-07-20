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
//#define PROTOTYPE_BUILD

#ifdef LOCAL_BUILD
#	define GEMINI_IDCODE			0x20000913
#else
#	define GEMINI_IDCODE			0x1000563d
#endif

#define GEMINI_PRODUCT_ID			0x31303050  // <-- need a map for bitstream product to device type
#define GEMINI_BLOCK_SIZE			2048
#define GEMINI_NUM_OF_BLOCKS		4
#define GEMINI_TIMEOUT_COUNTER		10
#define GEMINI_BUFFER_ADDR(base, x)	(base + ((x % GEMINI_NUM_OF_BLOCKS) * GEMINI_BLOCK_SIZE))
#define GEMINI_ACPU					1
#define GEMINI_BCPU					0
#define DM_SBCS						0x38
#define DM_SBADDRESS0				0x39
#define DM_SBDATA0					0x3c
#define DDR_INIT					1
#define DDR_NOT_INIT				0
#define GEMINI_SRAM_SIZE			261120 // 255kb
#define VIRGO_ILM_SIZE				65536  // 64kb
#define MSEC(miliseconds)			(miliseconds * 1000)
#define STATUS(x)					((x >> 10) & 0x3f)

enum gemini_prg_mode {
	GEMINI_PRG_MODE_FPGA,
	GEMINI_PRG_MODE_SPI_FLASH
};

struct device_t device_table[] =
{
#if defined(LOCAL_BUILD) || defined(PROTOTYPE_BUILD)
    { "local" , 0x80003ff0, 0x80003028, 0x800030f0, 0x80003ff4, 0x80000000, GEMINI_SRAM_SIZE, 0x80000000, 0x80003ffc, 0x80003ff8, { 0x12345678, 0 } },
    { "gemini", 0xf1000000, 0xf1000028, 0x8003DDF4, 0xf10a0000, 0x80000000, 131072          , 0x8003DDF8, 0x8003FDFC, 0x8003FDF8, { 0x10475253, 0 } },
#else
    { "Gemini", 0xf1000000, 0xf1000028, 0xf10000f0, 0xf10a0000, 0x80000000, GEMINI_SRAM_SIZE, 0x8003DDF8, 0x8003FDFC, 0x8003FDF8, { 0x10475253, 0 } },
    { "Virgo" , 0xa0110000, 0xa0110028, 0xa01100f0, 0xa0710000, 0xA0200000, VIRGO_ILM_SIZE  , 0xA040DFF8, 0xA040FFFC, 0xA040FFF8, { 0x10565253, 0 } },
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

static struct device_t *gemini_detect_device_type(struct target * target)
{
	struct device_t *dev = NULL;
	uint32_t chip_id;

	for (uint32_t i = 0; i < (sizeof(device_table) / sizeof(struct device_t)) && dev == NULL; ++i)
	{
		// try to read the chip id from the target
		if (gemini_read_reg32(target, device_table[i].probe_addr, &chip_id) != ERROR_OK)
		{
			continue;
		}

		// check to see it matches any chip id in the list of current device type
		for (int n = 0; n < MAX_NUM_OF_CHIP_ID; ++n)
		{
			if (device_table[i].chip_id[n] != 0 && device_table[i].chip_id[n] == chip_id)
			{
				// found a match!!
				dev = &device_table[i];
				break;
			}
		}
	}

	return dev;
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

static int gemini_get_cpu_type(struct target_info_t * target_info, uint32_t *cpu_type)
{
	uint32_t debug_control;

	if (gemini_read_reg32(target_info->target, target_info->device->debug_control, &debug_control) == ERROR_OK)
	{
		*cpu_type = debug_control & 0x1 ? GEMINI_ACPU : GEMINI_BCPU;
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

static int gemini_get_firmware_type(struct target_info_t * target_info, uint32_t *fw_type)
{
	uint32_t spare_reg;

	if (gemini_read_reg32(target_info->target, target_info->device->spare_reg, &spare_reg) == ERROR_OK)
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

static int gemini_get_command_status(struct target_info_t * target_info, uint32_t *status)
{
	uint32_t spare_reg;

	if (gemini_read_reg32(target_info->target, target_info->device->spare_reg, &spare_reg) == ERROR_OK)
	{
		*status = STATUS(spare_reg);
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

static int gemini_get_ddr_status(struct target_info_t * target_info, uint32_t *status)
{
	uint32_t spare_reg;

	if (gemini_read_reg32(target_info->target, target_info->device->spare_reg, &spare_reg) == ERROR_OK)
	{
		*status = (spare_reg & (1u << 16)) ? DDR_INIT : DDR_NOT_INIT;
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

static int gemini_check_target_device(struct target_info_t *target_info)
{
	if (target_info->tap->idcode != GEMINI_IDCODE)
	{
		LOG_ERROR("[RS] Unknown JTAG ID 0x%08x", target_info->tap->idcode);
		return ERROR_FAIL;
	}

	target_info->device = gemini_detect_device_type(target_info->target);
	if (!target_info->device)
	{
		LOG_ERROR("[RS] Failed to detect device type");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int gemini_get_config_status(struct target_info_t * target_info, uint32_t *cfg_done, uint32_t *cfg_error)
{
	uint32_t cfg_status = 0;

	if (gemini_read_reg32(target_info->target, target_info->device->cfg_status, &cfg_status) != ERROR_OK)
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

static int gemini_switch_to_bcpu(struct target_info_t * target_info)
{
	uint32_t cpu_type;

	if (gemini_get_cpu_type(target_info, &cpu_type) != ERROR_OK)
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

	if (gemini_sysbus_write_reg32(target_info->target, target_info->device->debug_control, 0) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to write debug_control register");
		return ERROR_FAIL;
	}

	jtag_add_tlr();
	jtag_execute_queue();
	target_info->target->examined = false;

	if (target_examine_one(target_info->target) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to re-initialize the target");
		return ERROR_FAIL;
	}

	if (gemini_get_cpu_type(target_info, &cpu_type) != ERROR_OK || cpu_type == GEMINI_ACPU)
	{
		LOG_INFO("[RS] Failed to switch to BCPU");
		return ERROR_FAIL;
	}

	LOG_INFO("[RS] Switched to BCPU");

	return ERROR_OK;
}

static int gemini_poll_command_complete_and_status(struct target_info_t * target_info, uint32_t *status, uint32_t wait_time_us, uint32_t num_polls)
{
	int retval = ERROR_OK;

	while (1)
	{
		retval = gemini_get_command_status(target_info, status);
		if (retval != ERROR_OK)
			break;
		if (*status != GEMINI_PRG_ST_PENDING)
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
		if (*status != GEMINI_PRG_ST_TASK_COMPLETE) {
			LOG_ERROR("[RS] Command error %d", *status);
			retval = ERROR_FAIL;
		}
	}
	// clear command and status field
	gemini_write_reg32(target_info->target, target_info->device->spare_reg, 16, 0, GEMINI_PRG_TSK_CMD_IDLE);

	return retval;
}

static int gemini_load_fsbl(struct target_info_t *target_info, gemini_bit_file_t *bit_file)
{
	int retval = ERROR_OK;
	uint32_t fsbl_size = bit_file->ubi_header->fsbl_size;
	uint32_t size = 1;
	uint32_t status;
	uint32_t fw_type;

	if (gemini_get_firmware_type(target_info, &fw_type) != ERROR_OK)
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

	if (fsbl_size > target_info->device->ram_size)
	{
		LOG_ERROR("[RS] FSBL package (%d bytes) is larger than the available SRAM size (%d bytes)", fsbl_size, target_info->device->ram_size);
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

	retval = gemini_write_memory(target_info->target, target_info->device->fsbl_ubi_addr, size, fsbl_size / size, (uint8_t *)bit_file->ubi_header);
	if (retval != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to write bitstream of %d bytes to SRAM at 0x%08" PRIxPTR, fsbl_size, target_info->device->fsbl_ubi_addr);
		return ERROR_FAIL;
	}

	LOG_DEBUG("[RS] Wrote %d bytes to SRAM at 0x%08" PRIxPTR, fsbl_size, target_info->device->fsbl_ubi_addr);

	if (gemini_write_reg32(target_info->target, target_info->device->spare_reg, 16, 0, GEMINI_PRG_TSK_CMD_BBF_FDI) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to write command 0x%x to spare_reg at 0x%08" PRIxPTR, GEMINI_PRG_TSK_CMD_BBF_FDI, target_info->device->spare_reg);
		return ERROR_FAIL;
	}

	LOG_DEBUG("[RS] Wrote command 0x%x to spare_reg at 0x%08" PRIxPTR, GEMINI_PRG_TSK_CMD_BBF_FDI, target_info->device->spare_reg);

	retval = gemini_poll_command_complete_and_status(target_info, &status, MSEC(1000), 5);
	if (retval == ERROR_OK)
	{
		// double check if the firmware type is FSBL
		if (gemini_get_firmware_type(target_info, &fw_type) != ERROR_OK || fw_type != GEMINI_PRG_FW_TYPE_CFG_FSBL)
			retval = ERROR_FAIL;
	}

	if (retval != ERROR_OK)
	{
		if (retval == ERROR_TIMEOUT_REACHED)
			gemini_reset_bcpu(target_info->target); // reset target to known state when command timeout
		LOG_ERROR("[RS] Failed to load FSBL firmware");
	}
	else
		LOG_INFO("[RS] Loaded FSBL firmware of size %d bytes successfully.", fsbl_size);

	return retval;
}

static int gemini_init_ddr(struct target_info_t *target_info)
{
	int retval = ERROR_OK;
	uint32_t status;

	if (gemini_get_ddr_status(target_info, &status) != ERROR_OK)
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

	if (gemini_write_reg32(target_info->target, target_info->device->spare_reg, 16, 0, GEMINI_PRG_TSK_CMD_BBF_FDI) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to write command 0x%x to spare_reg at 0x%08" PRIxPTR, GEMINI_PRG_TSK_CMD_BBF_FDI, target_info->device->spare_reg);
		return ERROR_FAIL;
	}

	retval = gemini_poll_command_complete_and_status(target_info, &status, MSEC(1000), 5);
	if (retval != ERROR_OK)
	{
		if (retval == ERROR_TIMEOUT_REACHED)
			gemini_reset_bcpu(target_info->target); // reset target to known state when command timeout
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

static int gemini_reset_read_write_counters(struct target_info_t *target_info)
{
	if (gemini_write_reg32(target_info->target, target_info->device->writer_counter, 32, 0, 0) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to reset write counter at 0x%08" PRIxPTR, target_info->device->writer_counter);
		return ERROR_FAIL;
	}

	if (gemini_write_reg32(target_info->target, target_info->device->read_counter, 32, 0, 0) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to reset read counter at 0x%08" PRIxPTR, target_info->device->read_counter);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int gemini_stream_data_blocks(struct target_info_t *target_info, uint8_t *data, uint64_t data_size, struct gemini_stats *stats)
{
	int retval = ERROR_OK;
	struct duration block_duration, bop_duration;
	uint32_t read_counter;
	uint32_t write_counter = 0;
	uint32_t timeout_counter = 0;
	uint32_t block_counter = data_size / GEMINI_BLOCK_SIZE;
	uint32_t spare_reg = 0;

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

		if (target_halt(target_info->target) != ERROR_OK)
		{
			LOG_ERROR("[RS] Failed to halt the BPU core");
			retval = ERROR_FAIL;
			break;
		}

		if (target_read_u32(target_info->target, target_info->device->read_counter, &read_counter) != ERROR_OK)
		{
			LOG_ERROR("[RS] Failed to retrieve read counter at 0x%08" PRIxPTR, target_info->device->read_counter);
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
				if (target_write_memory(target_info->target, GEMINI_BUFFER_ADDR(target_info->device->cbuffer, write_counter), sizeof(uint32_t), GEMINI_BLOCK_SIZE / sizeof(uint32_t), data) != ERROR_OK)
				{
					LOG_ERROR("[RS] Failed to write a block to 0x%08" PRIxPTR " on the device", GEMINI_BUFFER_ADDR(target_info->device->cbuffer, write_counter));
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

			if (target_write_u32(target_info->target, target_info->device->writer_counter, write_counter) != ERROR_OK)
			{
				LOG_ERROR("[RS] Failed to increment write counter at 0x%08" PRIxPTR, target_info->device->writer_counter);
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

			if (timeout_counter >= GEMINI_TIMEOUT_COUNTER)
			{
				if (target_read_u32(target_info->target, target_info->device->spare_reg, &spare_reg) == ERROR_OK && STATUS(spare_reg) != 0)
				{
					// read counter not advancing due to block processing error at fw
					LOG_ERROR("[RS] Command error %d.", STATUS(spare_reg));
					retval = ERROR_FAIL;
				}
				else
				{
					// read counter not advancing for unknown reason
					LOG_ERROR("[RS] Circular buffer timed out.");
					retval = ERROR_TIMEOUT_REACHED;
				}
				break;
			}
		}

		if (target_resume(target_info->target, true, 0, true, false) != ERROR_OK)
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

	if (target_info->target->halt_issued == true)
		target_resume(target_info->target, true, 0, true, false);

	return retval;
}

static int gemini_program_bitstream(struct target_info_t *target_info, gemini_bit_file_t *bit_file, unsigned int progress_log)
{
	int retval = ERROR_OK;
	uint32_t cfg_done;
	uint32_t cfg_error;
	uint32_t status;
	uint8_t *bop;
	struct gemini_stats stats = { 0 };

	LOG_INFO("[RS] Configuring FPGA fabric...");

	if (gemini_get_total_packages_size(bit_file, GEMINI_BLOCK_SIZE, &stats.total_packages_size) != 0)
	{
		LOG_ERROR("[RS] Bitstream BOP size is not multiple of %d bytes", GEMINI_BLOCK_SIZE);
		return ERROR_FAIL;
	}

	stats.data_sent = 0;
	stats.package_count = 0;
	stats.cicular_buffer_full_count = 0;
	stats.wait_time_us = MSEC(15);
	stats.log = progress_log;

	bop = gemini_get_first_bop(bit_file);
	while (bop != NULL)
	{
		if (gemini_is_xcb_bop(gemini_get_bop_id(bop)) == true)
		{
			// reset read/write counters
			if (gemini_reset_read_write_counters(target_info) != ERROR_OK)
			{
				retval = ERROR_FAIL;
				break;
			}

			// write task cmd
			if (gemini_write_reg32(target_info->target, target_info->device->spare_reg, 16, 0, GEMINI_PRG_TSK_CMD_CFG_BITSTREAM_FPGA) != ERROR_OK)
			{
				LOG_ERROR("[RS] Failed to write command %d to spare_reg at 0x%08" PRIxPTR, GEMINI_PRG_TSK_CMD_CFG_BITSTREAM_FPGA, target_info->device->spare_reg);
				retval = ERROR_FAIL;
				break;
			}

			// stream 2k data blocks onto device
			++stats.package_count;
			if ((retval = gemini_stream_data_blocks(target_info, bop, gemini_get_bop_size(bop), &stats)) != ERROR_OK)
			{
				break;
			}

			// check final cmd status
			if ((retval = gemini_poll_command_complete_and_status(target_info, &status, stats.wait_time_us, 20)) != ERROR_OK)
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
			gemini_reset_bcpu(target_info->target); // reset target to known state when command timeout
		LOG_ERROR("[RS] Failed to program bitstream to the device");
	}
	else
	{
		// check configuration done and error status
		retval = gemini_get_config_status(target_info, &cfg_done, &cfg_error);
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

static int gemini_program_flash(struct target_info_t *target_info, gemini_bit_file_t *bit_file, unsigned int progress_log)
{
	int retval = ERROR_OK;
	uint32_t status;
	struct gemini_stats stats = { 0 };
	uint64_t filesize = (uint64_t)bit_file->filesize;

	LOG_INFO("[RS] Programming SPI Flash...");

	if ((filesize % GEMINI_BLOCK_SIZE) != 0)
	{
		LOG_ERROR("[RS] Bitstream file size %" PRIu64 " is not multiple of %d bytes", filesize, GEMINI_BLOCK_SIZE);
		return ERROR_FAIL;
	}

	stats.total_packages_size = filesize;
	stats.data_sent = 0;
	stats.package_count = 1;
	stats.cicular_buffer_full_count = 0;
	stats.wait_time_us = MSEC(15);
	stats.log = progress_log;

	// reset read/write counters
	if (gemini_reset_read_write_counters(target_info) != ERROR_OK)
		return ERROR_FAIL;

	// write task cmd
	if (gemini_write_reg32(target_info->target, target_info->device->spare_reg, 16, 0, GEMINI_PRG_TSK_CMD_CFG_BITSTREAM_FLASH) != ERROR_OK)
	{
		LOG_ERROR("[RS] Failed to write command %d to spare_reg at 0x%08" PRIxPTR, GEMINI_PRG_TSK_CMD_CFG_BITSTREAM_FLASH, target_info->device->spare_reg);
		return ERROR_FAIL;
	}

	// stream 2k data block to device
	if (gemini_stream_data_blocks(target_info, (uint8_t *)bit_file->ubi_header, filesize, &stats) != ERROR_OK)
		return ERROR_FAIL;

	if (stats.log & 2)
		gemini_print_stats(&stats);

	// check cmd status
	retval = gemini_poll_command_complete_and_status(target_info, &status, stats.wait_time_us, 20);
	if (retval != ERROR_OK)
	{
		if (retval == ERROR_TIMEOUT_REACHED)
			gemini_reset_bcpu(target_info->target); // reset target to known state when command timeout
		LOG_ERROR("[RS] Failed to program SPI flash");
	}
	else
	{
		LOG_INFO("[RS] Programmed SPI Flash successfully");
	}

	return retval;
}

static int gemini_program_device(struct target_info_t *target_info, const char *filename, enum gemini_prg_mode mode, unsigned int progress_log)
{
	gemini_bit_file_t bit_file;
	int retval = ERROR_PLD_FILE_LOAD_FAILED;

	if (gemini_read_bit_file(&bit_file, filename) != ERROR_OK)
		return ERROR_PLD_FILE_LOAD_FAILED;

	if (gemini_check_target_device(target_info) != ERROR_OK)
		goto err;

	if (gemini_switch_to_bcpu(target_info) != ERROR_OK)
		goto err;

	if (gemini_load_fsbl(target_info, &bit_file) != ERROR_OK)
		goto err;

	if (gemini_init_ddr(target_info) != ERROR_OK)
		goto err;

	if (mode == GEMINI_PRG_MODE_FPGA)
		if (gemini_program_bitstream(target_info, &bit_file, progress_log) != ERROR_OK)
			goto err;

	if (mode == GEMINI_PRG_MODE_SPI_FLASH)
		if (gemini_program_flash(target_info, &bit_file, progress_log) != ERROR_OK)
			goto err;

	// everything is completed successfully
	retval = ERROR_OK;
err:
	gemini_free_bit_file(&bit_file);
	return retval;
}

static int gemini_load(struct pld_device *pld_device, const char *filename)
{
	return gemini_program_device(&(((struct gemini_pld_device *)pld_device->driver_priv)->target_info[0]), filename, GEMINI_PRG_MODE_FPGA, 0);
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
		if (!target && !target->tap)
		{
			command_print(CMD, "Target: %s is invalid", CMD_ARGV[i+1]);
			ret = ERROR_FAIL;
			break;
		}

		gemini_info->target_info[i].target = target;
		gemini_info->target_info[i].tap = target->tap;
		gemini_info->target_info[i].device = NULL;
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

COMMAND_HANDLER(gemini_handle_load_command)
{
	struct timeval start, end, duration;
	struct pld_device *device;
	struct gemini_pld_device *gemini_device = NULL;
	int retval;
	unsigned int progress_log = 0;
	unsigned int index;
	enum gemini_prg_mode mode;

	gettimeofday(&start, NULL);

	if (CMD_ARGC < 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	device = gemini_get_pld_device_driver();
	if (!device) {
		command_print(CMD, "pld device driver 'gemini' not found");
		return ERROR_FAIL;
	}

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], index);
	gemini_device = (struct gemini_pld_device *)(device->driver_priv);
	if (index >= gemini_device->count) {
		command_print(CMD, "device index '#%s' is out of bounds", CMD_ARGV[0]);
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

	retval = gemini_program_device(&gemini_device->target_info[index], CMD_ARGV[2], mode, progress_log);
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
	struct pld_device *device = NULL;
	struct gemini_pld_device *gemini_device = NULL;
	uint32_t cfg_done;
	uint32_t cfg_error;
	unsigned int index = 0;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	device = gemini_get_pld_device_driver();
	if (!device) {
		command_print(CMD, "pld device driver 'gemini' not found");
		return ERROR_FAIL;
	}

	// check if index is not out of bound
	gemini_device = (struct gemini_pld_device *)(device->driver_priv);
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], index);
	if (index >= gemini_device->count)
	{
		command_print(CMD, "device index '#%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	struct target_info_t *target_info = &gemini_device->target_info[index];
	if (gemini_check_target_device(target_info) != ERROR_OK)
		return ERROR_FAIL;

	if (gemini_get_config_status(target_info, &cfg_done, &cfg_error) != ERROR_OK)
		return ERROR_FAIL;

	// print header
	command_print(CMD, "            Device               cfg_done   cfg_error ");
	command_print(CMD, "----------- -------------------- ---------- ----------");

	// print cfg done and error status
	struct device_t *dev = gemini_device->target_info[index].device;
	command_print(CMD, "Found %5d %-20s %-10d %-10d", index, dev != NULL ? dev->name : "Unknown", cfg_done, cfg_error);

	return ERROR_OK;
}

COMMAND_HANDLER(gemini_handle_list_device_command)
{
	struct pld_device *device = NULL;
	struct gemini_pld_device *gemini_device = NULL;

	device = gemini_get_pld_device_driver();
	if (!device) {
		command_print(CMD, "pld device driver 'gemini' not found");
		return ERROR_FAIL;
	}

	// print header
	command_print(CMD, "         Device               ID           IRLen      Flash Size      ");
	command_print(CMD, "-------- -------------------- ------------ ---------- ----------------");

	gemini_device = (struct gemini_pld_device *)(device->driver_priv);
	for (uint32_t i = 0; i < gemini_device->count; ++i)
	{
		// detect device type
		struct device_t *dev = gemini_detect_device_type(gemini_device->target_info[i].target);

		// print device details
		command_print(CMD, "Found %2d %-20s 0x%-10x %-10d %d", i, dev != NULL ? dev->name : "Unknown",
						gemini_device->target_info[i].tap->idcode,
						gemini_device->target_info[i].tap->ir_length,
						1024*16 /*place holder*/);
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
