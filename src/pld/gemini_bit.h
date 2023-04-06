/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2023 by Rapid Silicon                                   *
 *   chernyee.kok@rapidsilicon.com                                         *
 ***************************************************************************/

#ifndef OPENOCD_PLD_GEMINI_BIT_H
#define OPENOCD_PLD_GEMINI_BIT_H

#include "helper/types.h"

/**
 * @struct   rs_ubi_header
 * @brief    Data structure for Universal Binary Image (UBI) header
 *
 */
typedef struct rs_ubi_header {
	uint16_t ubi_hdr_version;
	uint16_t hdr_size;
	/*                     External Image number (64bits)
	*                   einMsw                         | einLsw
	* Product ID b[63:48] | Customer Build ID b[47:32] | Image Version b[31:0] */
	uint32_t ein_msw;  // Product ID b[63:48] | Customer Build ID b[47:32]
	uint32_t ein_lsw;  // Image Major Ver b[31:0] | Image Minor Ver b[15:0]
	uint32_t ubi_size;
	uint8_t image_type;
	uint8_t package_count;
	uint16_t crc_16;
} __attribute__((packed)) ubi_header_t;

/**
 * @struct   rs_bop_header
 * @brief    Data structure for Binary Object Protection (BOP) header.
 *
 * BOP IDs List
 * ============
 * BOPK : Key
 * BOP1 : FSBL
 * BOPF : FCB
 * BOPI : ICB
 * BOPP : PCB
 * BOPA : ACPU
 * BOPU : uboot
 * BOPL : Linux
 * BOPZ : Zephyr
 */
typedef struct rs_bop_header {
	uint32_t bop_id;  // byte [3:1] 'B','O','P', byte[0] : binary type
	uint16_t bop_hdr_version;
	uint16_t sign_tool_version;
	uint32_t binary_version;
	uint32_t binary_len;
	uint32_t load_addr;
	uint32_t entry_addr;
	uint32_t offset_to_binary;
	uint32_t offset_to_next_header;
	uint8_t sig_algo;
	uint8_t option;
	uint8_t enc_algo;
	uint8_t iv_len;
	uint16_t pub_key_len;
	uint16_t enc_key_len;
	uint16_t sig_len;
	uint8_t compression_algo;
	uint8_t bin_pad_bytes;  // Number of padding bytes in the payload binary or
							// bitstream
	uint16_t xcb_header_size;
	uint8_t padding[16];  // To make BOP header 64 bytes for scatter gather hash
						// calculation
	uint16_t crc_16;
} __attribute__((packed)) bop_header_t;

typedef struct gemini_bit_file
{
	ubi_header_t *ubi_header;
	bop_header_t **bop_header_list;
	uint8_t *rawdata;
	long filesize;
} gemini_bit_file_t;

int gemini_create_helper_bitstream(gemini_bit_file_t *bit_file, uint8_t **bitstream, uint32_t *filesize);
int gemini_read_bit_file(gemini_bit_file_t *bit_file, const char *filename);
void gemini_free_bit_file(gemini_bit_file_t *bit_file);

#endif /* OPENOCD_PLD_GEMINI_BIT_H */
