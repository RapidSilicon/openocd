/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2023 by Rapid Silicon                                   *
 *   chernyee.kok@rapidsilicon.com                                         *
 ***************************************************************************/

#ifndef OPENOCD_PLD_GEMINI_H
#define OPENOCD_PLD_GEMINI_H

#include <jtag/jtag.h>
#include <target/target.h>
#include <target/riscv/riscv.h>

struct gemini_pld_device {
	struct target *target;
	struct jtag_tap *tap;
};

/* fsbl firmware command and status defines */
#define DDR_INIT				1
#define DDR_NOT_INIT			0
#define FW_BOOTROM				1
#define FW_FSBL					2
#define COMMAND_IDLE			0
#define COMMAND_LOAD_FSBL		1
#define COMMAND_INIT_DDR		2
#define COMMAND_LOAD_BITSTREAM	3
#define STATUS_IDLE				0
#define STATUS_IN_PROGRESS		1
#define STATUS_SUCCESS			2
#define STATUS_FAIL				3

#endif /* OPENOCD_PLD_GEMINI_H */
