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

#ifndef __RVDBG_H
#define __RVDBG_H

#include "jtag_scan.h"

#define RVDBG_MAX_HARTS 8

#define BIT_SIZEOF(x) (sizeof(x) * CHAR_BIT)
#define ARRAY_NUMELEM(x) (sizeof(x) / sizeof(x[0]))
#define container_of(ptr, type, member) ({                      \
        const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
        (type *)( (char *)__mptr - offsetof(type,member) );})

#define DMI_BASE_BIT_COUNT   		 34

#define DMI_GET_OP(x)                (x & 0x3)


#define DTMCS_DMIRESET               0x10000
#define DTMCS_DMIHARDRESET           0x20000
#define DTMCS_GET_VERSION(x)         (x & 0xf)
#define DTMCS_GET_ABITS(x)           ((x>>4) & 0x3f)
#define DTMCS_GET_DMISTAT(x)         ((x>>10) & 0x3)
#define DTMCS_GET_IDLE(x)            ((x>>12) & 0x7)

enum RISCV_DEBUG_VERSION {
	RISCV_DEBUG_VERSION_011 	= 0,
	RISCV_DEBUG_VERSION_013 	= 1,
	RISCV_DEBUG_VERSION_UNKNOWN = 15,
};

typedef struct HART_s {
    uint8_t idx;
    uint8_t mhartid;

    // Back up registers for progbuf communication (excludes x0)
    // TODO: Do not assume XLEN 32
    uint32_t gp_register_backup[31];
} HART_t;

typedef struct RVDBGv013_DTM_s {
    int refcnt;

    uint32_t idcode;
    enum RISCV_DEBUG_VERSION debug_version;
    uint8_t idle;
    uint8_t abits;

    uint8_t progbuf_size;
    bool impebreak;
    uint8_t abstract_data_count;
    bool support_autoexecdata;

    HART_t harts[RVDBG_MAX_HARTS];
    uint8_t num_harts;
    HART_t *current_hart;

    int (*rvdbg_dmi_low_access)(struct RVDBGv013_DTM_s *dtm, uint32_t *dmi_data_out, uint64_t dmi_cmd);
    void (*rvdbg_dmi_reset)(struct RVDBGv013_DTM_s *dtm, bool hard_reset);

    int (*read_csr)(struct RVDBGv013_DTM_s *dp, uint16_t reg_id, uint32_t *value);
    int (*write_csr)(struct RVDBGv013_DTM_s *dp, uint16_t reg_id, uint32_t value);
    int (*read_mem)(struct RVDBGv013_DTM_s *dp, uint32_t address, uint32_t *value);
    int (*write_mem)(struct RVDBGv013_DTM_s *dp, uint32_t address, uint32_t value);
} RVDBGv013_DMI_t;

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

int rvdbg_dtm_init(RVDBGv013_DMI_t *dtm);
void rvdbg013_jtag_dp_handler(jtag_dev_t *dev);
void rvdbg_dtm_ref(RVDBGv013_DMI_t *dtm);
void rvdbg_dtm_unref(RVDBGv013_DMI_t *dtm);
int rvdbg_set_debug_version(RVDBGv013_DMI_t *dp, uint8_t version);

#endif /* __RVDBG_H */