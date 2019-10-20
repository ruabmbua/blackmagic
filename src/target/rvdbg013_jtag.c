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
#define ARRAY_NUMELEM(x) (sizeof(x) / sizeof(x[0]))

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
	DMI_REG_ABSTRACTDATA_BEGIN = 0x04,
	DMI_REG_ABSTRACTDATA_END   = 0x0f,
	DMI_REG_DMCONTROL          = 0x10,
	DMI_REG_DMSTATUS           = 0x11,
	DMI_REG_HARTINFO           = 0x12,
	DMI_REG_HALTSUM1           = 0x13,
	DMI_REG_HAWINDOWSEL        = 0x14,
	DMI_REG_HAWINDOW           = 0x15,
	DMI_REG_ABSTRACT_CS        = 0x16,
	DMI_REG_ABSTRACT_CMD       = 0x17,
	DMI_REG_ABSTRACT_AUTOEXEC  = 0x18,
	DMI_REG_CONFSTR_PTR0       = 0x19,
	DMI_REG_CONFSTR_PTR1       = 0x1a,
	DMI_REG_CONFSTR_PTR2       = 0x1b,
	DMI_REG_CONFSTR_PTR3       = 0x1c,
	DMI_REG_NEXTDM_ADDR        = 0x1d,
	DMI_REG_PROGRAMBUF_BEGIN   = 0x20,
	DMI_REG_PROGRAMBUF_END     = 0x2f,
	DMI_REG_AUTHDATA		   = 0x30,
	DMI_REG_HALTSUM2           = 0x34,
	DMI_REG_HALTSUM3 		   = 0x35,
	DMI_REG_SBADDRESS3		   = 0x37,
	DMI_REG_SYSBUSCS           = 0x38,
	DMI_REG_SBADDRESS0		   = 0x39,
	DMI_REG_SBADDRESS1		   = 0x3a,
	DMI_REG_SBADDRESS2		   = 0x3b,
	DMI_REG_SBDATA0			   = 0x3c,
	DMI_REG_SBDATA1			   = 0x3d,
	DMI_REG_SBDATA2			   = 0x3e,
	DMI_REG_SBDATA3			   = 0x3f,
	DMI_REG_HALTSUM0	 	   = 0x40,
};

enum ABSTRACTCMD_TYPE {
	ABSTRACTCMD_TYPE_ACCESS_REGISTER = 0x0,
	ABSTRACTCMD_TYPE_QUICK_ACCESS    = 0x1,
	ABSTRACTCMD_TYPE_ACCESS_MEMORY   = 0x2,
};

enum BUS_ACCESS {
	BUS_ACCESS_8   = 0x0,
	BUS_ACCESS_16  = 0x1,
	BUS_ACCESS_32  = 0x2,
	BUS_ACCESS_64  = 0x3,
	BUS_ACCESS_128 = 0x4,
};

enum ABSTRACTCMD_ERR {
	ABSTRACTCMD_ERR_NONE = 0x0,
	ABSTRACTCMD_ERR_BUSY = 0x1,
	ABSTRACTCMD_ERR_NOT_SUPPORTED = 0x2,
	ABSTRACTCMD_ERR_EXCEPTION = 0x3,
	ABSTRACTCMD_ERR_HALT_RESUME = 0x4,
	ABSTRACTCMD_ERR_BUS = 0x5,
	ABSTRACTCMD_ERR_OTHER = 0x7,
};

enum AUTOEXEC_STATE {
	AUTOEXEC_STATE_NONE, /* Ingnore autoexec */
	AUTOEXEC_STATE_INIT, /* Setup everything + AARAUTOINC */
	AUTOEXEC_STATE_CONT, /* Only access data0 register */
};

#define DMI_BASE_BIT_COUNT   		 34

#define DTMCS_DMIRESET               0x10000
#define DTMCS_DMIHARDRESET           0x20000
#define DTMCS_GET_VERSION(x)         (x & 0xf)
#define DTMCS_GET_ABITS(x)           ((x>>4) & 0x3f)
#define DTMCS_GET_DMISTAT(x)         ((x>>10) & 0x3)
#define DTMCS_GET_IDLE(x)            ((x>>12) & 0x7)

#define DMI_GET_OP(x)                (x & 0x3)

#define DMSTATUS_GET_VERSION(x)         DTMCS_GET_VERSION(x)
#define DMSTATUS_GET_CONFSTRPTRVALID(x) ((x >> 4) & 0x1)
#define DMSTATUS_GET_HASRESETHALTREQ(x) ((x >> 5) & 0x1)
#define DMSTATUS_GET_AUTHBUSY(x)		((x >> 6) & 0x1)
#define DMSTATUS_GET_AUTHENTICATED(x)   ((x >> 7) & 0x1)
#define DMSTATUS_GET_ANYNONEXISTENT(x)  ((x >> 14) & 0x1)
#define DMSTATUS_GET_IMPEBREAK(x)	    ((x >> 22) & 0x1)

#define DMCONTROL_GET_HARTSEL(x)     (((x >> 16) & 0x3ff) | (((x >> 6) & 0x3ff) << 10))
#define DMCONTROL_SET_HARTSEL(t, s)  do { \
	t &= ~(0xfffff << 6); \
	t |= (s & 0x3ff) << 16; \
	t |= (s & (0x3ff << 10) >> 4); } while(0)

#define ABSTRACTCS_GET_DATACOUNT(x)   (x & 0xf)
#define ABSTRACTCS_GET_CMDERR(x)      ((x >> 8) & 0x7)
#define ABSTRACTCS_CLEAR_CMDERR(t) do { t |= (0x7 << 8);} while (0)
#define ABSTRACTCS_GET_BUSY(x)		  ((x >> 12) & 0x1)
#define ABSTRACTCS_GET_PROGBUFSIZE(x) ((x >> 24) & 0x1f)

#define ABSTRACTCMD_SET_TYPE(t, s) do { \
	t &= ~(0xff << 24); \
	t |= (s & 0xff) << 24; } while (0)
#define ABSTRACTCMD_ACCESS_REGISTER_SET_AARSIZE(t, s) do { \
	t &= ~(0x7 << 20); \
	t |= (s & 0x7) << 20; } while (0)
#define ABSTRACTCMD_ACCESS_REGISTER_SET_AARPOSTINCREMENT(t, s) do { \
	t &= ~(0x1 << 19); \
	t |= (s & 0x1) << 19; } while (0)
#define ABSTRACTCMD_ACCESS_REGISTER_SET_POSTEXEC(t, s) do { \
	t &= ~(0x1 << 18); \
	t |= (s & 0x1) << 18; } while (0)
#define ABSTRACTCMD_ACCESS_REGISTER_SET_TRANSFER(t, s) do { \
	t &= ~(0x1 << 17); \
	t |= (s & 0x1) << 17; } while (0)
#define ABSTRACTCMD_ACCESS_REGISTER_SET_WRITE(t, s)    do { \
	t &= ~(0x1 << 16); \
	t |= (s & 0x1) << 16; } while (0)
#define ABSTRACTCMD_ACCESS_REGISTER_SET_REGNO(t, s)    do { \
	t &= ~(0xffff); \
	t |= s & 0xffff; } while (0)

#define ABSTRACTAUTO_SOME_PATTEN		(0b101010101010)
#define ABSTRACTAUTO_GET_DATA(x)        (x & 0xfff)
#define ABSTRACTAUTO_SET_DATA(t, s)     do { \
	t &= ~(0xfff); \
	t |= s & 0xfff; } while (0)

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

	// DEBUG("after dmireset: dtmcs = 0x%08x\n", (uint32_t)dtmcontrol);
}

// TODO: Rewrite to always use proper run/test/idle timing
static int rvdbg_dmi_low_access(RVDBGv013_DP_t *dp, uint32_t *dmi_data_out, uint64_t dmi_cmd)
{
	uint64_t dmi_ret;

retry:
	jtag_dev_shift_dr(dp->dev, (void*)&dmi_ret, (const void*)&dmi_cmd,
		DMI_BASE_BIT_COUNT + dp->abits);

	switch (DMI_GET_OP(dmi_ret)) {
		case DMISTAT_OP_INTERRUPTED:
			// Retry after idling, restore last dmi
			rvdbg_dmi_reset(dp, false);
			jtag_dev_write_ir(dp->dev, IR_DMI);
			jtag_dev_shift_dr(dp->dev, (void*)&dmi_ret, (const void*)&dp->last_dmi,
				DMI_BASE_BIT_COUNT + dp->abits);

			// DEBUG("in 0x%"PRIx64"\n", dmi_ret);

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

static int rvdbg_dmi_write(RVDBGv013_DP_t *dp, uint32_t addr, uint32_t data)
{
	return rvdbg_dmi_low_access(dp, NULL, 
		((uint64_t)addr << DMI_BASE_BIT_COUNT) | (data << 2) | DMI_OP_WRITE);
}

static int rvdbg_dmi_read(RVDBGv013_DP_t *dp, uint32_t addr, uint32_t *data)
{
	if (rvdbg_dmi_low_access(dp, NULL, ((uint64_t)addr << DMI_BASE_BIT_COUNT) | DMI_OP_READ) < 0)
		return -1;

	return rvdbg_dmi_low_access(dp, data, DMI_OP_NOP);
}

static int rvdbg_set_debug_version(RVDBGv013_DP_t *dp, uint8_t version)
{
	switch (version) {
		case RISCV_DEBUG_VERSION_013:
			dp->debug_version = version;
			break;
		case RISCV_DEBUG_VERSION_011:
			DEBUG("Error: RISC-V debug 0.11 not supported\n");
			return -1;
		case RISCV_DEBUG_VERSION_UNKNOWN:
		default:
			DEBUG("RISC-V target unknown debug spec verson: %d\n", version);
			return -1;
	}

	return 0;
}

#ifdef ENABLE_DEBUG
static const char* rvdbg_version_tostr(enum RISCV_DEBUG_VERSION version)
{
	switch (version) {
		case RISCV_DEBUG_VERSION_011:
			return "0.11";
		case RISCV_DEBUG_VERSION_013:
			return "0.13";
		case RISCV_DEBUG_VERSION_UNKNOWN:
		default:
			return "UNKNOWN";
	}
}
#endif /* ENABLE_DEBUG */

static int rvdbg_discover_harts(RVDBGv013_DP_t *dp)
{
	uint32_t dmcontrol = 0;
	uint32_t hart_idx, hartsellen, dmstatus;

	// Set all 20 bits of hartsel
	DMCONTROL_SET_HARTSEL(dmcontrol, 0xfffff);
	if (rvdbg_dmi_write(dp, DMI_REG_DMCONTROL, dmcontrol) < 0)
		return -1;

	if (rvdbg_dmi_read(dp, DMI_REG_DMCONTROL, &dmcontrol) < 0)
		return -1;

	hartsellen = DMCONTROL_GET_HARTSEL(dmcontrol);

	DEBUG("hartsellen = 0x%05x\n", hartsellen);
	
	// Iterate over all possible harts
	for (hart_idx = 0; hart_idx <= hartsellen 
			&& dp->num_harts < ARRAY_NUMELEM(dp->harts); hart_idx++) {
		dmcontrol = 0;
		DMCONTROL_SET_HARTSEL(dmcontrol, hart_idx);
		
		if (rvdbg_dmi_write(dp, DMI_REG_DMCONTROL, dmcontrol) < 0)
			return -1;
		
		// Check if anynonexist is true -> abort
		if (rvdbg_dmi_read(dp, DMI_REG_DMSTATUS, &dmstatus) < 0)
			return -1;

		if (DMSTATUS_GET_ANYNONEXISTENT(dmstatus)) {
			DEBUG("Hart idx 0x%05x does not exist\n", hart_idx);
			break;
		}

		// TODO: Add the hart
		dp->num_harts++;
	}

	DEBUG("num_harts = %d\n", dp->num_harts);

	return 0;
}

/**
 * Returns negative error, or positive cmderror
 */
static int rvdbg_abstract_command_run(RVDBGv013_DP_t *dp, uint32_t command)
{
	uint32_t abstractcs;
	uint8_t cmderror;

retry:
	if (rvdbg_dmi_write(dp, DMI_REG_ABSTRACT_CMD, command) < 0)
		return -1;

	// Wait until the abstract command finished
	do {
		if (rvdbg_dmi_read(dp, DMI_REG_ABSTRACT_CS, &abstractcs) < 0)
			return -1;
	} while (ABSTRACTCS_GET_BUSY(abstractcs));

	cmderror = ABSTRACTCS_GET_CMDERR(abstractcs);

	if (cmderror != ABSTRACTCMD_ERR_NONE) {
		// Clear the error
		abstractcs = 0;
		ABSTRACTCS_CLEAR_CMDERR(abstractcs);
		if (rvdbg_dmi_write(dp, DMI_REG_ABSTRACT_CS, abstractcs) < 0)
			return -1;

		// Handle ERR_BUSY retries automatically
		if (cmderror == ABSTRACTCMD_ERR_BUSY) {
			DEBUG("RISC-V abstract command busy, retry...\n");
			goto retry;
		}
	}

	return cmderror;
}

static int rvdbg_read_single_reg(RVDBGv013_DP_t *dp, uint16_t reg_idx, uint32_t *out,
	enum AUTOEXEC_STATE astate)
{
	uint32_t command = 0;
	uint32_t abstractcs;
	int ret;

	// Construct abstract command
	// TODO: Do not expect XLEN of 32 by default
	ABSTRACTCMD_SET_TYPE(command, ABSTRACTCMD_TYPE_ACCESS_REGISTER);
	ABSTRACTCMD_ACCESS_REGISTER_SET_AARSIZE(command, BUS_ACCESS_32);
	ABSTRACTCMD_ACCESS_REGISTER_SET_TRANSFER(command, 1);
	ABSTRACTCMD_ACCESS_REGISTER_SET_REGNO(command, reg_idx);
	ABSTRACTCMD_ACCESS_REGISTER_SET_AARPOSTINCREMENT(command,
		astate == AUTOEXEC_STATE_INIT ? 1 : 0);

	// Avoid wrinting command, when in autoexec cont mode
	if (astate != AUTOEXEC_STATE_CONT) {
		// Initiate register read command
		if ((ret = rvdbg_abstract_command_run(dp, command)) < 0)
			return -1;
	
		// Handle error
		switch (ret) {
			case ABSTRACTCMD_ERR_NONE:
				break;
			case ABSTRACTCMD_ERR_EXCEPTION:
				// TODO: This check becomes invalid as soon as postexec is set.
				DEBUG("RISC-V register 0x%"PRIx16"\n does not exist", reg_idx);
				return -1;
			default:
				DEBUG("RISC-V abstract command error: %d\n", ret);
				return -1;
		}
	}

	if (rvdbg_dmi_read(dp, DMI_REG_ABSTRACTDATA_BEGIN, out) < 0)
		return -1;

	if (astate == AUTOEXEC_STATE_CONT) {
		// In cont mode, only read when not busy (not guarded by rvdbg_abstract_command_run)
		do {
			if (rvdbg_dmi_read(dp, DMI_REG_ABSTRACT_CS, &abstractcs) < 0)
				return -1;
		} while (ABSTRACTCS_GET_BUSY(abstractcs));
	}

	return 0;
}

static int rvdbg_write_single_reg(RVDBGv013_DP_t *dp, uint16_t reg_id, uint32_t value,
	enum AUTOEXEC_STATE astate)
{
	uint32_t command = 0;
	uint32_t abstractcs;
	int ret;

	// Write value to data0
	if (rvdbg_dmi_write(dp, DMI_REG_ABSTRACTDATA_BEGIN, value) < 0)
		return -1;

	// Construct abstract command
	// TODO: Do not expect XLEN of 32 by default
	ABSTRACTCMD_SET_TYPE(command, ABSTRACTCMD_TYPE_ACCESS_REGISTER);
	ABSTRACTCMD_ACCESS_REGISTER_SET_AARSIZE(command, BUS_ACCESS_32);
	ABSTRACTCMD_ACCESS_REGISTER_SET_TRANSFER(command, 1);
	ABSTRACTCMD_ACCESS_REGISTER_SET_WRITE(command, 1);
	ABSTRACTCMD_ACCESS_REGISTER_SET_REGNO(command, reg_id);
	ABSTRACTCMD_ACCESS_REGISTER_SET_AARPOSTINCREMENT(command, 
		astate == AUTOEXEC_STATE_INIT ? 1 : 0);

	// Only initiate the write, if not in autoexec cont state
	if (astate != AUTOEXEC_STATE_CONT) {
		// Initiate register write command
		if ((ret = rvdbg_abstract_command_run(dp, command)) < 0)
			return -1;
		
		// Handle error
		switch (ret) {
			case ABSTRACTCMD_ERR_NONE:
				break;
			case ABSTRACTCMD_ERR_EXCEPTION:
				// TODO: This check becomes invalid as soon as postexec is set.
				DEBUG("RISC-V register 0x%"PRIx16"\n does not exist", reg_id);
				return -1;
			default:
				DEBUG("RISC-V abstract command error: %d\n", ret);
				return -1;
		}
	} else {
		// When in cont state, make sure to wait until write is done
		do {
			if (rvdbg_dmi_read(dp, DMI_REG_ABSTRACT_CS, &abstractcs) < 0)
				return -1;
		} while (ABSTRACTCS_GET_BUSY(abstractcs));
	}


	return 0;
}

static int rvdbg_write_regs(RVDBGv013_DP_t *dp, uint16_t reg_id, const uint32_t *values,
	uint16_t len)
{
	enum AUTOEXEC_STATE astate = AUTOEXEC_STATE_NONE;
	uint32_t abstractauto;
	uint16_t i;
	int err = 0;

	// When more than one reg written and autoexec support
	if (len > 1  && dp->support_autoexecdata) {
		astate = AUTOEXEC_STATE_INIT;
		abstractauto = 0;
		ABSTRACTAUTO_SET_DATA(abstractauto, ABSTRACTAUTO_SOME_PATTEN);
		if (rvdbg_dmi_write(dp, DMI_REG_ABSTRACT_AUTOEXEC, abstractauto) < 0)
			return -1;
	}

	for (i = 0; i < len; i++) {
		if (rvdbg_write_single_reg(dp, reg_id + i, values[i], astate) < 0) {
			err = -1;
			break;
		}
		if (astate == AUTOEXEC_STATE_INIT)
			astate = AUTOEXEC_STATE_CONT;
	}

	// Reset auto exec state
	if (astate != AUTOEXEC_STATE_NONE) {
		abstractauto = 0;
		ABSTRACTAUTO_SET_DATA(abstractauto, 0);
		if (rvdbg_dmi_write(dp, DMI_REG_ABSTRACT_AUTOEXEC, abstractauto) < 0)
			return -1;
	}

	return err;
}

static int rvdbg_read_regs(RVDBGv013_DP_t *dp, uint16_t reg_id, uint32_t *values,
	uint16_t len)
{
	enum AUTOEXEC_STATE astate = AUTOEXEC_STATE_NONE;
	uint32_t abstractauto;
	uint16_t i;
	int err = 0;

	// When more than one reg read and autoexec support
	if (len > 1  && dp->support_autoexecdata) {
		astate = AUTOEXEC_STATE_INIT;
		abstractauto = 0;
		ABSTRACTAUTO_SET_DATA(abstractauto, ABSTRACTAUTO_SOME_PATTEN);
		if (rvdbg_dmi_write(dp, DMI_REG_ABSTRACT_AUTOEXEC, abstractauto) < 0)
			return -1;
	}

	for (i = 0; i < len; i++) {
		if (rvdbg_read_single_reg(dp, reg_id + i, &values[i], astate) < 0) {
			err = -1;
			break;
		}
		if (astate == AUTOEXEC_STATE_INIT)
			astate = AUTOEXEC_STATE_CONT;
	}

	// Reset auto exec state
	if (astate != AUTOEXEC_STATE_NONE) {
		abstractauto = 0;
		ABSTRACTAUTO_SET_DATA(abstractauto, 0);
		if (rvdbg_dmi_write(dp, DMI_REG_ABSTRACT_AUTOEXEC, abstractauto) < 0)
			return -1;
	}

	return err;
}

static int rvdbg_progbuf_upload(RVDBGv013_DP_t *dp, const uint32_t* buffer, uint8_t buffer_len)
{
	uint8_t i;

	for (i = DMI_REG_PROGRAMBUF_BEGIN; i < buffer_len; i++) {
		if (rvdbg_dmi_write(dp, DMI_REG_PROGRAMBUF_BEGIN + i, buffer[i]) < 0)
			return -1;
	}
}

// static void rvdbg_progbuf_exec(RVDBGv013_DP_t *dp, uint32_t *args, uint8_t arglen)
// {
// 	// Backup argument registers

// }

// static void rvdbg_read_csr_progbuf(RVDBGv013_DP_t *dp, uint16_t reg_id, uint32_t* value) { }

// static void rvdbg_write_csr_progbuf(RVDBGv013_DP_t *dp, uint16_t reg_id, uint32_t value) { }

// static void rvdbg_read_mem_progbuf(RVDBGv013_DP_t *dp, uint32_t address, uint32_t* value) { }

// static void rvdbg_write_mem_progbuf(RVDBGv013_DP_t *dp, uint32_t address, uint32_t value) { }

static int rvdbg_select_mem_and_csr_access_impl(RVDBGv013_DP_t *dp)
{
	uint32_t abstractcs, abstractauto;
	
	if (rvdbg_dmi_read(dp, DMI_REG_ABSTRACT_CS, &abstractcs) < 0)
		return -1;

	dp->progbuf_size = ABSTRACTCS_GET_PROGBUFSIZE(abstractcs);
	dp->abstract_data_count = ABSTRACTCS_GET_DATACOUNT(abstractcs);

	if (dp->abstract_data_count < 1 || dp->abstract_data_count > 12) {
		// Invalid count of abstract data
		DEBUG("RISC-V: Invalid count of abstract data: %d\n", dp->abstract_data_count);
		return -1;
	}

	if (dp->progbuf_size > 16) {
		// Invalid progbuf size
		DEBUG("RISC-V: progbufsize is too large: %d\n", dp->progbuf_size);
		return -1;
	} else if (dp->progbuf_size == 1 && !dp->impebreak) {
		// When progbufsize is 1, impebreak is required.
		DEBUG("RISC-V: progbufsize 1 requires impebreak feature\n");
		return -1;
	}

	// DEBUG("datacount = %d\n", dp->abstract_data_count);

	// Check if a program buffer is supported, and it is sufficient for accessing
	// CSR and / or MEMORY.
	// --------------------------------------------------------------------------
	if (dp->progbuf_size > 0) {
		// PROGBUF supported
		DEBUG("RISC-V: Program buffer with size %d supported.\n", dp->progbuf_size);

		// dp->read_csr = rvdbg_read_csr_progbuf;
		// dp->write_csr = rvdbg_write_csr_progbuf;
		// dp->read_mem = rvdbg_read_mem_progbuf;
		// dp->write_mem = rvdbg_write_mem_progbuf;
	}

	// Check if autoexecdata feature can be used
	// -----------------------------------------
	abstractauto = 0;
	ABSTRACTAUTO_SET_DATA(abstractauto, ABSTRACTAUTO_SOME_PATTEN);
	if (rvdbg_dmi_write(dp, DMI_REG_ABSTRACT_AUTOEXEC, abstractauto) < 0)
		return -1;
	if (rvdbg_dmi_read(dp, DMI_REG_ABSTRACT_AUTOEXEC, &abstractauto) < 0)
		return -1;

	if (ABSTRACTAUTO_GET_DATA(abstractauto) == ABSTRACTAUTO_SOME_PATTEN) {
		DEBUG("RISC-V: autoexecdata feature supported\n");
		dp->support_autoexecdata = true;
	}

	ABSTRACTAUTO_SET_DATA(abstractauto, 0);
	if (rvdbg_dmi_write(dp, DMI_REG_ABSTRACT_AUTOEXEC, abstractauto) < 0)
		return -1;

	return 0;
}

static int rvdbg_jtag_init(RVDBGv013_DP_t *dp)
{
	uint64_t dtmcontrol; /* uint64_t due to https://github.com/blacksphere/blackmagic/issues/542 */
	uint8_t version;
	uint32_t dmstatus, nextdmaddr;

	DEBUG("RISC-V DTM id 0x%x detected: `%s`\n"
		"Scanning RISC-V target ...\n", dp->idcode, dp->dev->descr);

	// Read from the DTM control and status register
	jtag_dev_write_ir(dp->dev, IR_DTMCS);
	jtag_dev_shift_dr(dp->dev, (void*)&dtmcontrol,
		(void*)&dtmcontrol, 32);
		
	DEBUG("  dtmcs = 0x%08x\n", (uint32_t)dtmcontrol);

	version = DTMCS_GET_VERSION((uint32_t)dtmcontrol);
	if (rvdbg_set_debug_version(dp, version) < 0)
		return -1;
	
	dp->idle = DTMCS_GET_IDLE(dtmcontrol);
	dp->abits = DTMCS_GET_ABITS(dtmcontrol);

	DEBUG("  debug version = %s\n  abits = %d\n  dmistat = %d\n  idle = ",
		rvdbg_version_tostr(dp->debug_version), dp->abits, DTMCS_GET_DMISTAT((uint32_t)dtmcontrol));

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

	rvdbg_dmi_reset(dp, true);

	// Switch to DMI register
	jtag_dev_write_ir(dp->dev, IR_DMI);

	if (rvdbg_dmi_read(dp, DMI_REG_DMSTATUS, &dmstatus) < 0)
		return -1;

	DEBUG("dmstatus = 0x%08x\n", dmstatus);

	version = DMSTATUS_GET_VERSION(dmstatus);
	if (version == 0) {
		DEBUG("No debug module present\n");
	} else if ((uint8_t)(version - 1) != dp->debug_version) {
		DEBUG("dtmcs and dmstatus debug version mismatch\n");
		// Trust the dmstatus register. Ignore error, and leave
		// previous version active
		// ----------------------------------------------------
		if (version != (uint8_t)RISCV_DEBUG_VERSION_UNKNOWN)
			rvdbg_set_debug_version(dp, version);
	}

	// TODO: Implement authentification plugins
	if (!DMSTATUS_GET_AUTHENTICATED(dmstatus)) {
		// Not authentificated -> not supported
		DEBUG("RISC-V DM requires authentification!\n");
		return -1;
	}

	if (DMSTATUS_GET_CONFSTRPTRVALID(dmstatus)) {
		DEBUG("RISC-V configuration string available\n");
	}

	if (rvdbg_dmi_read(dp, DMI_REG_NEXTDM_ADDR, &nextdmaddr) < 0)
		return -1;
	if (nextdmaddr) {
		// Multiple DM per DMI not yet supported
		DEBUG("Warning: Detected multiple RISC-V debug modules, only one supported!\n");
	}

	// Get impebreak before selecting mem and csr access impl
	dp->impebreak = DMSTATUS_GET_IMPEBREAK(dmstatus);

	if (rvdbg_select_mem_and_csr_access_impl(dp) < 0) {
		DEBUG("RISC-V: no compatible MEM / CSR access implementation detected.\n");
		return -1;
	}

	if (rvdbg_discover_harts(dp) < 0) 
		return -1;

	return 0;
}
