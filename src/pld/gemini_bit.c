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
	if (bit_file->ubi_header->fsbl_size != 0 && bit_file->ubi_header->fsbl_size > bit_file->filesize)
	{
		LOG_ERROR("[RS] Corrupted bitstream content");
		free(bit_file->ubi_header);
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	return ERROR_OK;
}

void gemini_free_bit_file(gemini_bit_file_t *bit_file)
{
	free(bit_file->ubi_header);
}
