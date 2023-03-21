
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

#define RS_CRC16_START 0x0000

static uint16_t crc16_table[256] = {
	0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241, 0xc601,
	0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440, 0xcc01, 0x0cc0,
	0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40, 0x0a00, 0xcac1, 0xcb81,
	0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841, 0xd801, 0x18c0, 0x1980, 0xd941,
	0x1b00, 0xdbc1, 0xda81, 0x1a40, 0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01,
	0x1dc0, 0x1c80, 0xdc41, 0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0,
	0x1680, 0xd641, 0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081,
	0x1040, 0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
	0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441, 0x3c00,
	0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41, 0xfa01, 0x3ac0,
	0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840, 0x2800, 0xe8c1, 0xe981,
	0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41, 0xee01, 0x2ec0, 0x2f80, 0xef41,
	0x2d00, 0xedc1, 0xec81, 0x2c40, 0xe401, 0x24c0, 0x2580, 0xe541, 0x2700,
	0xe7c1, 0xe681, 0x2640, 0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0,
	0x2080, 0xe041, 0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281,
	0x6240, 0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
	0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41, 0xaa01,
	0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840, 0x7800, 0xb8c1,
	0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41, 0xbe01, 0x7ec0, 0x7f80,
	0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40, 0xb401, 0x74c0, 0x7580, 0xb541,
	0x7700, 0xb7c1, 0xb681, 0x7640, 0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101,
	0x71c0, 0x7080, 0xb041, 0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0,
	0x5280, 0x9241, 0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481,
	0x5440, 0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
	0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841, 0x8801,
	0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40, 0x4e00, 0x8ec1,
	0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41, 0x4400, 0x84c1, 0x8581,
	0x4540, 0x8701, 0x47c0, 0x4680, 0x8641, 0x8201, 0x42c0, 0x4380, 0x8341,
	0x4100, 0x81c1, 0x8081, 0x4040};

uint16_t rs_crypto_crc16(const unsigned char *input_str, size_t num_bytes)
{
	uint16_t crc = RS_CRC16_START;

	if (input_str != NULL) {
		for (size_t i = 0; i < num_bytes; i++) {
			crc = (crc >> 8) ^ crc16_table[(crc ^ (uint16_t)*input_str++) & 0x00FF];
		}
	}

	return crc;
}

int gemini_read_bit_file(gemini_bit_file_t *bit_file, const char *filename)
{
	FILE *input_file;
	struct stat input_stat;
	int read_count;
	uint32_t header_offset;

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
	fseek(input_file, 0L, SEEK_END);
	bit_file->filesize = ftell(input_file);
	fseek(input_file, 0L, SEEK_SET);

	// allocate memory for file content
	bit_file->rawdata = (uint8_t *)malloc(bit_file->filesize);

	// read entire file content
	read_count = fread(bit_file->rawdata, sizeof(uint8_t), bit_file->filesize, input_file);
	if (read_count != bit_file->filesize) {
		LOG_ERROR("[RS] couldn't read the entire content from file '%s'", filename);
		fclose(input_file);
		free(bit_file->rawdata);
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	// close the file
	fclose(input_file);

	// parse bit file
	bit_file->ubi_header = (ubi_header_t *)bit_file->rawdata;
	bit_file->bop_header_list = (bop_header_t **)malloc(sizeof(bop_header_t *) * bit_file->ubi_header->packageCount);
	header_offset = bit_file->ubi_header->headerSize;

	// verify ubi header crc
	uint16_t ubi_header_crc16 = rs_crypto_crc16((const unsigned char *)bit_file->ubi_header, sizeof(ubi_header_t) - 2);
	if (ubi_header_crc16 != bit_file->ubi_header->crc16)
	{
		LOG_ERROR("[RS] Invalid UBI Header CRC %x", ubi_header_crc16);
		free(bit_file->rawdata);
		free(bit_file->bop_header_list);
		return ERROR_FAIL;
	}

	LOG_INFO("[RS] Input file name '%s'", filename);
	LOG_INFO("[RS] Input file size %ld byte(s)", bit_file->filesize);
	LOG_INFO("[RS] UBI Header");
	LOG_INFO("[RS]  Version %d", bit_file->ubi_header->ubiHeadrVersion);
	LOG_INFO("[RS]  Size %d", bit_file->ubi_header->headerSize);
	LOG_INFO("[RS]  Image Type 0x%x", bit_file->ubi_header->imageType);
	LOG_INFO("[RS]  Package Count %d", bit_file->ubi_header->packageCount);

	for (uint8_t i = 0; i < bit_file->ubi_header->packageCount; i++)
	{
		if (header_offset >= bit_file->filesize)
		{
			free(bit_file->bop_header_list);
			free(bit_file->rawdata);
			return ERROR_PLD_FILE_LOAD_FAILED;
		}

		bit_file->bop_header_list[i] = (bop_header_t *)(bit_file->rawdata + header_offset);
		header_offset += bit_file->bop_header_list[i]->offsetToNextHeader;

		// verify bop crc
		uint16_t bop_header_crc16 = rs_crypto_crc16((const unsigned char *)bit_file->bop_header_list[i], sizeof(bop_header_t) - 2);
		if (bop_header_crc16 != bit_file->bop_header_list[i]->crc16)
		{
			free(bit_file->bop_header_list);
			free(bit_file->rawdata);
			LOG_ERROR("[RS] Invalid BOP Header CRC %x", bop_header_crc16);
			return ERROR_PLD_FILE_LOAD_FAILED;
		}
		
		LOG_INFO("[RS]  BOP Header %d", i);
		LOG_INFO("[RS]   Id '%.*s'", 4, (char *)&bit_file->bop_header_list[i]->bopId);
		LOG_INFO("[RS]   Version %d", bit_file->bop_header_list[i]->bopHeaderVersion);
		LOG_INFO("[RS]   Binary Length %d", bit_file->bop_header_list[i]->binaryLen);
		LOG_INFO("[RS]   Signing Algorithm 0x%x", bit_file->bop_header_list[i]->sigAlgo);
		LOG_INFO("[RS]   Encryption 0x%x", bit_file->bop_header_list[i]->encAlgo);
	}

	return ERROR_OK;
}

int gemini_create_helper_bitstream(gemini_bit_file_t *bit_file, uint8_t **bitstream, uint32_t *filesize)
{
	bop_header_t *bop_fsbl = NULL;
	uint32_t padding = 0;

	if (!filesize || !bit_file)
		return ERROR_COMMAND_SYNTAX_ERROR;

	for (uint8_t i = 0; i < bit_file->ubi_header->packageCount; i++)
	{
		if (memcmp(&bit_file->bop_header_list[i]->bopId, "1POB", 4) == 0)
		{
			bop_fsbl = bit_file->bop_header_list[i];
			break;
		}
	}

	if (!bop_fsbl)
	{
		LOG_ERROR("[RS] FSBL BOP not found");
		return ERROR_FAIL;
	}

	// calculate fsbl bop size
	if (bop_fsbl->offsetToNextHeader == 0)
		*filesize = (bit_file->rawdata + bit_file->filesize) - ((uint8_t *)bop_fsbl) + 16;
	else
		*filesize = bop_fsbl->offsetToNextHeader + 16;

	// add padding if filesize not 32-bit word aligned
	if ((*filesize & 0x3) != 0) {
		LOG_WARNING("[RS] FSBL BOP size is not 32-bit word aligned");
		padding = 4 - (*filesize & 0x3);
		*filesize += padding;
	}

	if (bitstream)
	{
		*bitstream = (uint8_t *)malloc(*filesize);
		ubi_header_t *ubi_header = (ubi_header_t *)(*bitstream);
		bop_header_t *bop_header = (bop_header_t *)(*bitstream + 16);
		memcpy(ubi_header, bit_file->ubi_header, 16);
		memcpy(bop_header, bop_fsbl, *filesize - 16 - padding);
		ubi_header->packageCount = 1;
		ubi_header->crc16 = rs_crypto_crc16((const unsigned char *)ubi_header, sizeof(ubi_header_t) - 2);
		bop_header->offsetToNextHeader = 0;
		bop_header->crc16 = rs_crypto_crc16((const unsigned char *)bop_header, sizeof(bop_header_t) - 2);
	}

	return ERROR_OK;
}

void gemini_free_bit_file(gemini_bit_file_t *bit_file)
{
	free(bit_file->bop_header_list);
	free(bit_file->rawdata);
}