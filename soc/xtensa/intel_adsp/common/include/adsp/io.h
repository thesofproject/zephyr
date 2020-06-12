/*
 * Copyright(c) 2016 Intel Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Author: Liam Girdwood <liam.r.girdwood@linux.intel.com>
 */

#ifndef __INCLUDE_IO__
#define __INCLUDE_IO__

#include <stdint.h>
#include <soc/memory.h>
#include <sys/sys_io.h>
#include <arch/common/sys_io.h>

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

static inline uint32_t shim_read(uint32_t reg)
{
	return sys_read32(SHIM_BASE + reg);
}

static inline void shim_write(uint32_t reg, uint32_t val)
{
	sys_write32(val, (SHIM_BASE + reg));
}

static inline uint64_t shim_read64(uint32_t reg)
{
	return *((volatile uint64_t*)(SHIM_BASE + reg));
}

static inline void shim_write64(uint32_t reg, uint64_t val)
{
	*((volatile uint64_t*)(SHIM_BASE + reg)) = val;
}

static inline uint32_t irq_read(uint32_t reg)
{
	return *((volatile uint32_t*)(IRQ_BASE + reg));
}

static inline void irq_write(uint32_t reg, uint32_t val)
{
	*((volatile uint32_t*)(IRQ_BASE + reg)) = val;
}

static inline uint32_t ipc_read(uint32_t reg)
{
	return sys_read32(IPC_HOST_BASE + reg);
}

static inline void ipc_write(uint32_t reg, uint32_t val)
{
	sys_write32(val, (IPC_HOST_BASE + reg));
}

static inline uint32_t idc_read(uint32_t reg, uint32_t core_id)
{
	return *((volatile uint32_t*)(IPC_DSP_BASE(core_id) + reg));
}

static inline void idc_write(uint32_t reg, uint32_t core_id, uint32_t val)
{
	*((volatile uint32_t*)(IPC_DSP_BASE(core_id) + reg)) = val;
}

#endif
