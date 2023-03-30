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
#define TASK_COMMAND_IDLE		0
#define TASK_COMMAND_A			1	/* BootROM to load and jump to FSBL, FSBL to initalize DDR */
#define TASK_COMMAND_B			2	/* FSBL to program bitstream to QSPI Flash */
#define TASK_COMMAND_C			3	/* FSBL to program bitstream to FPGA Fabric */
#define TASK_STATUS_IDLE		0	/* This is mean the command is in progress as well */
#define TASK_STATUS_SUCCESS		1
#define TASK_STATUS_FAIL		2

#endif /* OPENOCD_PLD_GEMINI_H */
