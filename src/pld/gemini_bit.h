/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2023 by Rapid Silicon                                   *
 *   chernyee.kok@rapidsilicon.com                                         *
 ***************************************************************************/

#ifndef OPENOCD_PLD_GEMINI_BIT_H
#define OPENOCD_PLD_GEMINI_BIT_H

#include "helper/types.h"

#define UBI_VERSION 0x1

/**
 * @struct   rs_ubi_header
 * @brief    Data structure for Universal Binary Image (UBI) header
 *
 */
typedef struct rs_ubi_header {
	uint16_t ubi_hdr_version;
	uint16_t hdr_size;
	uint32_t product_id;
	uint32_t customer_id;

	// image_version [31:0] :
	// b[31:16] - Image Major Version
	// b[15: 0] - Image Minor Version
	uint32_t image_version;

	uint32_t ubi_size;
	uint32_t fsbl_size; // Size of UBI header + FSBL BOP if it exists
	uint8_t image_type;
	uint8_t package_count;
	uint16_t crc_16;
} __attribute__((packed)) ubi_header_t;

typedef struct gemini_bit_file
{
	ubi_header_t *ubi_header;
	long filesize;
} gemini_bit_file_t;

int gemini_read_bit_file(gemini_bit_file_t *bit_file, const char *filename);
void gemini_free_bit_file(gemini_bit_file_t *bit_file);

#endif /* OPENOCD_PLD_GEMINI_BIT_H */
