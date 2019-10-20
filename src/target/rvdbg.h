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

typedef struct RVDBGv013_DP_s {
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

    uint64_t last_dmi;

    int (*read_csr)(struct RVDBGv013_DP_s *dp, uint16_t reg_id, uint32_t *value);
    int (*write_csr)(struct RVDBGv013_DP_s *dp, uint16_t reg_id, uint32_t value);
    int (*read_mem)(struct RVDBGv013_DP_s *dp, uint32_t address, uint32_t *value);
    int (*write_mem)(struct RVDBGv013_DP_s *dp, uint32_t address, uint32_t value);

    jtag_dev_t *dev;
} RVDBGv013_DP_t;

void rvdbg013_jtag_dp_handler(jtag_dev_t *dev);

#endif /* __RVDBG_H */