/* SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright(c) 2019 Intel Corporation. All rights reserved.
 *
 * Author: Tomasz Lauda <tomasz.lauda@linux.intel.com>
 */


#ifndef __PLATFORM_TRACE_TRACE_H__
#define __PLATFORM_TRACE_TRACE_H__

#include <sof/lib/cpu.h>
#include <sof/lib/mailbox.h>
#include <sof/lib/memory.h>
#include <stdint.h>

#define PLATFORM_TRACEP_SLAVE_CORE(x) \
	(SRAM_REG_FW_TRACEP_SLAVE_CORE_BASE + ((x) - 1) * 0x4)

/* Platform defined trace code */
static inline void platform_trace_point(uint32_t x)
{
	int cpu = cpu_get_id();
	uint32_t offset;

	if (cpu == PLATFORM_MASTER_CORE_ID)
		offset = SRAM_REG_FW_TRACEP;
	else
		offset = PLATFORM_TRACEP_SLAVE_CORE(cpu);

	mailbox_sw_reg_write(offset, x);
}

#endif /* __PLATFORM_TRACE_TRACE_H__ */

