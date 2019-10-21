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

typedef struct RVDBGv013_JTAG_s {
	RVDBGv013_DMI_t dmi;
	jtag_dev_t *dev;
	uint64_t last_dmi;
} RVDBGv013_JTAG_t;

static void rvdbg_dmi_reset_jtag(RVDBGv013_DMI_t *dmi, bool hard_reset)
{
	RVDBGv013_JTAG_t *rvdbg_jtag = container_of(dmi, RVDBGv013_JTAG_t, dmi);
	
	jtag_dev_write_ir(rvdbg_jtag->dev, IR_DTMCS);
	
	uint64_t dtmcontrol =  /* uint64_t due to https://github.com/blacksphere/blackmagic/issues/542 */
		hard_reset ? DTMCS_DMIHARDRESET : DTMCS_DMIRESET;

	jtag_dev_shift_dr(rvdbg_jtag->dev, (void*)&dtmcontrol, (void*)&dtmcontrol, 32);

	// DEBUG("after dmireset: dtmcs = 0x%08x\n", (uint32_t)dtmcontrol);
	
	// Switch to DMI register
	jtag_dev_write_ir(rvdbg_jtag->dev, IR_DMI);
}

// TODO: Rewrite to always use proper run/test/idle timing
static int rvdbg_dmi_low_access_jtag(RVDBGv013_DMI_t *dmi, uint32_t *dmi_data_out, uint64_t dmi_cmd)
{
	RVDBGv013_JTAG_t *rvdbg_jtag = container_of(dmi, RVDBGv013_JTAG_t, dmi);
	
	uint64_t dmi_ret;

retry:
	jtag_dev_shift_dr(rvdbg_jtag->dev, (void*)&dmi_ret, (const void*)&dmi_cmd,
		DMI_BASE_BIT_COUNT + dmi->abits);

	switch (DMI_GET_OP(dmi_ret)) {
		case DMISTAT_OP_INTERRUPTED:
			// Retry after idling, restore last dmi
			rvdbg_dmi_reset_jtag(dmi, false);
			jtag_dev_shift_dr(rvdbg_jtag->dev, (void*)&dmi_ret, (const void*)&rvdbg_jtag->last_dmi,
				DMI_BASE_BIT_COUNT + dmi->abits);

			// DEBUG("in 0x%"PRIx64"\n", dmi_ret);

			if (dmi->idle >= 2)
				jtagtap_tms_seq(0, dmi->idle - 1);
			goto retry;

		case DMISTAT_NO_ERROR:
			rvdbg_jtag->last_dmi = dmi_cmd;
			break;

		case DMISTAT_RESERVED:
		case DMISTAT_OP_FAILED:
		default:
			DEBUG("DMI returned error: %"PRIx64"\n", dmi_ret);
			// jtag_dev_write_ir(rvdbg_jtag->dev, IR_DMI);
			rvdbg_dmi_reset_jtag(dmi, false);
			// TODO: Support recovering?
			return -1;
	}

	if (dmi_data_out != NULL)
		*dmi_data_out = (dmi_ret >> 2);

	return 0;
}

void rvdbg013_jtag_dp_handler(jtag_dev_t *dev)
{
	uint64_t dtmcontrol;
	uint8_t version;

	RVDBGv013_JTAG_t *rvdbg_jtag = calloc(1, sizeof(*rvdbg_jtag));
	if (!rvdbg_jtag) {
		DEBUG("calloc: failed in %s\n", __func__);
		return;
	}

	rvdbg_jtag->dev = dev;
	rvdbg_jtag->dmi.idcode = dev->idcode;
	rvdbg_jtag->dmi.rvdbg_dmi_low_access = rvdbg_dmi_low_access_jtag;
	rvdbg_jtag->dmi.rvdbg_dmi_reset = rvdbg_dmi_reset_jtag;

	DEBUG("RISC-V DTM id 0x%x detected: `%s`\n"
		"Scanning RISC-V target ...\n", rvdbg_jtag->dmi.idcode, rvdbg_jtag->dev->descr);

	// Read from the DTM control and status register
	jtag_dev_write_ir(rvdbg_jtag->dev, IR_DTMCS);
	jtag_dev_shift_dr(rvdbg_jtag->dev, (void*)&dtmcontrol,
		(void*)&dtmcontrol, 32);
		
	DEBUG("  dtmcs = 0x%08x\n", (uint32_t)dtmcontrol);

	version = DTMCS_GET_VERSION((uint32_t)dtmcontrol);
	if (rvdbg_set_debug_version(&rvdbg_jtag->dmi, version) < 0) {
		free(rvdbg_jtag);
		return;
	}
	
	rvdbg_jtag->dmi.idle = DTMCS_GET_IDLE(dtmcontrol);
	rvdbg_jtag->dmi.abits = DTMCS_GET_ABITS(dtmcontrol);

	if (rvdbg_dtm_init(&rvdbg_jtag->dmi) < 0) {
		free(rvdbg_jtag);
	}
	// TODO: Replace later
	else free(rvdbg_jtag);
}
