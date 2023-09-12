/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2023 by Rapid Silicon                                   *
 *   chernyee.kok@rapidsilicon.com                                         *
 ***************************************************************************/

#ifndef OPENOCD_PLD_GEMINI_BIT_H
#define OPENOCD_PLD_GEMINI_BIT_H

#include "helper/types.h"

#define UBI_VERSION 0x1

enum rs_bop_type {
	BOP_MANF = 0x464e414d,      // "MANF"  - Manufacturer's FSBL
	BOP_FSBL = 0x4c425346,      // "FSBL"  - Production FSBL
	BOP_FCB = 0x00424346,       // "FCB\0" - FPGA Config Block
	BOP_ICB = 0x00424349,       // "ICB\0" - IO Config Block
	BOP_PCB = 0x00424350,       // "PCB\0" - Preload Control Block
	BOP_FPGA = 0x41475046,      // "FPGA"  - Any of FCB, ICB, PCB or all
	BOP_ACPU = 0x55504341,      // "ACPU"  - ACPU baremetal image
	BOP_UBOOT = 0x00544255,     // "UBT\0" - U-boot image
	BOP_LINUX = 0x00584e4c,     // "LNX\0" - Linux image
	BOP_ZEPHYR = 0x5248505a,    // "ZPHR"  - Zephyr image
	BOP_KEY_CERT = 0x4359454b,  // "KEYC"  - Key certificate
};

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
	uint8_t *current_bop;
	uint32_t current_bop_index;
	long filesize;
} gemini_bit_file_t;

int gemini_read_bit_file(gemini_bit_file_t *bit_file, const char *filename);
void gemini_free_bit_file(gemini_bit_file_t *bit_file);
uint8_t *gemini_get_first_bop(gemini_bit_file_t *bit_file);
uint8_t *gemini_get_next_bop(gemini_bit_file_t *bit_file);
int gemini_get_total_packages_size(gemini_bit_file_t *bit_file, uint32_t block_size, enum rs_bop_type type, uint64_t *total_size);
uint64_t gemini_get_bop_size(uint8_t *bop);
uint32_t gemini_get_bop_id(uint8_t *bop);
int gemini_is_bop_type(uint8_t *bop, enum rs_bop_type type);

#endif /* OPENOCD_PLD_GEMINI_BIT_H */
