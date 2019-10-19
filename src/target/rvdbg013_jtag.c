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

enum DMI_OP {
	DMI_OP_NOP   = 0,
	DMI_OP_READ  = 1,
	DMI_OP_WRITE = 2,
};

enum DMI_REG {
	DMI_REG_ABSTRACT_DATA0    = 0x04,
	DMI_REG_ABSTRACT_DATA11   = 0x0f,
	DMI_REG_DMCONTROL         = 0x10,
	DMI_REG_DMSTATUS          = 0x11,
	DMI_REG_HARTINFO          = 0x12,
	DMI_REG_HALTSUM1          = 0x13,
	DMI_REG_HAWINDOWSEL       = 0x14,
	DMI_REG_HAWINDOW          = 0x15,
	DMI_REG_ABSTRACT_CS       = 0x16,
	DMI_REG_ABSTRACT_CMD      = 0x17,
	DMI_REG_ABSTRACT_AUTOEXEC = 0x18,
	DMI_REG_CONFSTR_PTR0      = 0x19,
	DMI_REG_CONFSTR_PTR1      = 0x1a,
	DMI_REG_CONFSTR_PTR2      = 0x1b,
	DMI_REG_CONFSTR_PTR3      = 0x1c,
	DMI_REG_NEXTDM_ADDR       = 0x1d, 
};

#define DMI_BASE_BIT_COUNT   34

#define DTMCS_DMIRESET       0x10000
#define DTMCS_DMIHARDRESET   0x20000
#define DTMCS_GET_VERSION(x) (x & 0xf)
#define DTMCS_GET_ABITS(x)   ((x>>4) & 0x3f)
#define DTMCS_GET_DMISTAT(x) ((x>>10) & 0x3)
#define DTMCS_GET_IDLE(x)    ((x>>12) & 0x7)

#define DMI_GET_OP(x)        (x & 0x3)

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

static int rvdbg_dmi_low_access(RVDBGv013_DP_t *dp, uint32_t *dmi_data_out, uint64_t dmi_cmd)
{
	uint64_t dmi_ret;

retry:
	jtag_dev_shift_dr(dp->dev, (void*)&dmi_ret, (const void*)&dmi_cmd, DMI_BASE_BIT_COUNT + dp->abits);

	switch (DMI_GET_OP(dmi_ret)) {
		case DMISTAT_OP_INTERRUPTED:
			// Retry after idling, restore last dmi
			rvdbg_dmi_reset(dp, false);
			jtag_dev_write_ir(dp->dev, IR_DMI);
			jtag_dev_shift_dr(dp->dev, (void*)&dmi_ret, (const void*)&dp->last_dmi, DMI_BASE_BIT_COUNT + dp->abits);

			DEBUG("in %"PRIx64"\n", dmi_ret);

			if (dp->idle >= 2)
				jtagtap_tms_seq(0, dp->idle - 1);
			goto retry;

		case DMISTAT_NO_ERROR:
			dp->last_dmi = dmi_cmd;
			break;

		case DMISTAT_RESERVED:
		case DMISTAT_OP_FAILED:
		default:
			DEBUG("DMI returned error: %"PRIx64"\n", dmi_ret);
			jtag_dev_write_ir(dp->dev, IR_DMI);
			rvdbg_dmi_reset(dp, false);
			// TODO: Support recovering?
			return -1;
	}

	if (dmi_data_out != NULL)
		*dmi_data_out = (dmi_ret >> 2);

	return 0;
}

// static int rvdbg_dmi_write(RVDBGv013_DP_t *dp, uint32_t addr, uint32_t data)
// {
// 	return rvdbg_dmi_low_access(dp, NULL, ((uint64_t)addr << DMI_BASE_BIT_COUNT) | (data << 2) | DMI_OP_WRITE);
// }

static int rvdbg_dmi_read(RVDBGv013_DP_t *dp, uint32_t addr, uint32_t *data)
{
	if (rvdbg_dmi_low_access(dp, NULL, ((uint64_t)addr << DMI_BASE_BIT_COUNT) | DMI_OP_READ) < 0)
		return -1;

	return rvdbg_dmi_low_access(dp, data, DMI_OP_NOP);
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

	uint32_t dmstatus;
	if (rvdbg_dmi_read(dp, 0x11, &dmstatus) < 0)
		return -1;

	DEBUG("dmstatus = 0x%08x\n", dmstatus);

	return 0;
}
