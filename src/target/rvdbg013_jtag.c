/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2019  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/** 
 + This file implements the JTAG-DP specific functions of the
 * RISC-V External Debug Support Version 0.13
 */

#include <limits.h>
#include "general.h"
#include "exception.h"
#include "jtag_scan.h"
#include "jtagtap.h"
#include "morse.h"
#include "rvdbg.h"

#define BIT_SIZEOF(x) (sizeof(x) * CHAR_BIT)

enum DTM_REGISTERS {
	// 0x00 is recommended to be IR_BYPASS
	IR_IDCODE = 0x01,
	IR_DTMCS  = 0x10, /* DTM control and status              */
	IR_DMI    = 0x11, /* Debug module interface access       */
	// 0x12 to 0x17 reserved
	IR_BYPASS = 0x1f,
};

enum DMISTAT {
	DMISTAT_NO_ERROR       = 0,
	DMISTAT_RESERVED       = 1,
	DMISTAT_OP_FAILED      = 2,
	DMISTAT_OP_INTERRUPTED = 3,
};

#define DTMCS_DMIRESET       0x10000
#define DTMCS_DMIHARDRESET   0x20000
#define DTMCS_GET_VERSION(x) (x & 0xf)
#define DTMCS_GET_ABITS(x)   ((x>>4) & 0x3f)
#define DTMCS_GET_DMISTAT(x) ((x>>10) & 0x3)
#define DTMCS_GET_IDLE(x)    ((x>>12) & 0x7)

static int rvdbg_jtag_init(RVDBGv013_DP_t *dp);

void rvdbg013_jtag_dp_handler(jtag_dev_t *dev)
{
	RVDBGv013_DP_t *dp = calloc(1, sizeof(*dp));
	if (!dp) {
		DEBUG("calloc: failed in %s\n", __func__);
		return;
	}

	dp->dev = dev;
	dp->idcode = dev->idcode;

	if (rvdbg_jtag_init(dp) < 0) {
		free(dp);
	}
	// TODO: Replace later
	else free(dp);

}

static void rvdbg_dmi_reset(RVDBGv013_DP_t *dp, bool hard_reset)
{
	jtag_dev_write_ir(dp->dev, IR_DTMCS);
	
	uint64_t dtmcontrol =  /* uint64_t due to https://github.com/blacksphere/blackmagic/issues/542 */
		hard_reset ? DTMCS_DMIHARDRESET : DTMCS_DMIRESET;

	jtag_dev_shift_dr(dp->dev, (void*)&dtmcontrol, (void*)&dtmcontrol, 32);

	DEBUG("after dmireset: dtmcs = 0x%08x\n", (uint32_t)dtmcontrol);
}

static int rvdbg_jtag_init(RVDBGv013_DP_t *dp)
{
	uint64_t dtmcontrol; /* uint64_t due to https://github.com/blacksphere/blackmagic/issues/542 */
	uint8_t version;

	DEBUG("RISC-V DTM id 0x%x detected: `%s`\n"
		"Scanning RISC-V target ...\n", dp->idcode, dp->dev->descr);

	// Read from the DTM control and status register
	jtag_dev_write_ir(dp->dev, IR_DTMCS);
	jtag_dev_shift_dr(dp->dev, (void*)&dtmcontrol,
		(void*)&dtmcontrol, 32);
		
	DEBUG("  dtmcs: 0x%08x\n", (uint32_t)dtmcontrol);

	version = DTMCS_GET_VERSION((uint32_t)dtmcontrol);
	switch (version) {
		case RISCV_DEBUG_VERSION_011:
			DEBUG("Warning: RISC-V target might not be fully supported\n");
			/* FALLTHROUGH */
		case RISCV_DEBUG_VERSION_013:
			dp->debug_version = version;
			break;
		case RISCV_DEBUG_VERSION_UNKNOWN:
		default:
			DEBUG("RISC-V target unknown debug spec verson: %d\n", version);
			return -1;
	}
	
	dp->idle = DTMCS_GET_IDLE(dtmcontrol);
	dp->abits = DTMCS_GET_ABITS(dtmcontrol);

	DEBUG("  version: %d\n  abits: %d\n  dmistat: %d\n  idle: ",
		dp->debug_version, dp->abits, DTMCS_GET_DMISTAT((uint32_t)dtmcontrol));
	switch (dp->idle) {
		case 0:
			DEBUG("no run/test state\n");
			break;
		case 1:
			DEBUG("leave run/test immediately\n");
			break;
		default:
			DEBUG("stay %d cycles in run/test\n", dp->idle - 1);
			break;
	}

	rvdbg_dmi_reset(dp, false);

	// Switch to DMI register
	jtag_dev_write_ir(dp->dev, IR_DMI);

	return 0;
}
