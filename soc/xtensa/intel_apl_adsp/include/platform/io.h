/*
 * Copyright(c) 2016 Intel Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Author: Liam Girdwood <liam.r.girdwood@linux.intel.com>
 */

#ifndef __INCLUDE_IO__
#define __INCLUDE_IO__

#include <stdint.h>
#include <config.h>
#include <sys/sys_io.h>
#include <arch/common/sys_io.h>

#if CONFIG_LIBRARY

static inline uint32_t io_reg_read(uint32_t reg) { return 0; }
static inline void io_reg_write(uint32_t reg, uint32_t val) {}
static inline void io_reg_update_bits(uint32_t reg, uint32_t mask,
				      uint32_t value) {}
static inline uint16_t io_reg_read16(uint32_t reg) { return 0; }
static inline void io_reg_write16(uint32_t reg, uint16_t val) {}

#else

static inline uint32_t io_reg_read(uint32_t reg)
{
	return sys_read32(reg);
}

static inline void io_reg_write(uint32_t reg, uint32_t val)
{
	/* Note: Parameters in different order */
	sys_write32(val, reg);
}

static inline void io_reg_update_bits(uint32_t reg, uint32_t mask,
				      uint32_t value)
{
	io_reg_write(reg, (io_reg_read(reg) & (~mask)) | (value & mask));
}

static inline uint16_t io_reg_read16(uint32_t reg)
{
	return sys_read16(reg);
}

static inline void io_reg_write16(uint32_t reg, uint16_t val)
{
	/* Note: Parameters in different order */
	sys_write16(val, reg);
}

#endif

#endif
