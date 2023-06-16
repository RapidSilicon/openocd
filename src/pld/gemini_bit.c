/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2023 by Rapid Silicon                                   *
 *   chernyee.kok@rapidsilicon.com                                         *
 ***************************************************************************/

#include "gemini_bit.h"
#include "pld.h"
#include <helper/log.h>
#include <sys/stat.h>
#include <helper/system.h>

#define OFFSET_OLD		28
#define OFFSET_NEW		8

int gemini_read_bit_file(gemini_bit_file_t *bit_file, const char *filename)
{
	FILE *input_file;
	struct stat input_stat;
	int read_count;

	if (!filename || !bit_file)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (stat(filename, &input_stat) == -1) {
		LOG_ERROR("[RS] couldn't stat() %s: %s", filename, strerror(errno));
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	if (S_ISDIR(input_stat.st_mode)) {
		LOG_ERROR("[RS] %s is a directory", filename);
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	if (input_stat.st_size == 0) {
		LOG_ERROR("[RS] Empty file %s", filename);
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	input_file = fopen(filename, "rb");
	if (!input_file) {
		LOG_ERROR("[RS] couldn't open %s: %s", filename, strerror(errno));
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	// get file size
	bit_file->filesize = input_stat.st_size;

	// allocate memory for file content
	bit_file->ubi_header = (ubi_header_t *)malloc(bit_file->filesize);

	// read entire file content
	read_count = fread(bit_file->ubi_header, sizeof(uint8_t), bit_file->filesize, input_file);
	if (read_count != bit_file->filesize) {
		LOG_ERROR("[RS] couldn't read the entire content from file '%s'", filename);
		fclose(input_file);
		free(bit_file->ubi_header);
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	// close the file
	fclose(input_file);

	// sanity check to ensure ubi header version is supported
	if (bit_file->ubi_header->ubi_hdr_version != UBI_VERSION)
	{
		LOG_ERROR("[RS] Unsupported UBI header version %x", bit_file->ubi_header->ubi_hdr_version);
		free(bit_file->ubi_header);
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	// sanity check to see if fsbl_size (if it exists) is within the file content boundary
	if (bit_file->ubi_header->fsbl_size != 0 && (long)bit_file->ubi_header->fsbl_size > bit_file->filesize)
	{
		LOG_ERROR("[RS] Corrupted bitstream content");
		free(bit_file->ubi_header);
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	bit_file->current_bop = NULL;
	bit_file->current_bop_index = 0;

	return ERROR_OK;
}

void gemini_free_bit_file(gemini_bit_file_t *bit_file)
{
	free(bit_file->ubi_header);
}

uint32_t gemini_get_bop_id(uint8_t *bop)
{
	return *(uint32_t *)bop;
}

int gemini_is_xcb_bop(uint32_t id)
{
	return (id == BOP_FCB || id == BOP_ICB || id == BOP_PCB) ? true : false;
}

uint64_t gemini_get_bop_size(uint8_t *bop)
{
	// old format -> offset_to_next_header @ byte offset 28, uint32_t
	// new format -> bop_size @ byte offset 8, uint64_t
	switch (*(uint32_t *)bop)
	{
		case BOP_FCB:
		case BOP_ICB:
		case BOP_PCB:
			return *(uint64_t *)(bop + OFFSET_NEW);
		case BOP_MANF:
		case BOP_FSBL:
		case BOP_ACPU:
		case BOP_UBOOT:
		case BOP_LINUX:
		case BOP_ZEPHYR:
			return *(uint32_t *)(bop + OFFSET_OLD);
		default:
			break;
	}
	return 0;
}

uint8_t *gemini_get_first_bop(gemini_bit_file_t *bit_file)
{
	if (!bit_file || bit_file->ubi_header->package_count == 0)
		return NULL;

	bit_file->current_bop = ((uint8_t *)bit_file->ubi_header) + sizeof(ubi_header_t);
	bit_file->current_bop_index = 0;

	return bit_file->current_bop;
}

uint8_t *gemini_get_next_bop(gemini_bit_file_t *bit_file)
{
	if (!bit_file || !bit_file->current_bop)
		return NULL;

	if ((bit_file->current_bop_index + 1) >= bit_file->ubi_header->package_count)
		return NULL;

	bit_file->current_bop_index += 1;
	bit_file->current_bop += gemini_get_bop_size(bit_file->current_bop);

	return bit_file->current_bop;
}

uint64_t gemini_get_total_packages_size(gemini_bit_file_t *bit_file)
{
	uint64_t total_size = 0;
	uint8_t *bop = NULL;

	if (!bit_file)
		return 0;

	bop = gemini_get_first_bop(bit_file);
	while (bop != NULL)
	{
		if (gemini_is_xcb_bop(gemini_get_bop_id(bop)) == true)
			total_size += gemini_get_bop_size(bop);
		bop = gemini_get_next_bop(bit_file);
	}

	return total_size;
}
