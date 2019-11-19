/*
 * Copyright(c) 2018 Intel Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Author: Rander Wang <rander.wang@linux.intel.com>
 */

#ifndef __INCLUDE_ARCH_CPU__
#define __INCLUDE_ARCH_CPU__

/* FIXME workaround for includes */
#ifndef __SOF_CPU_H__
#define __SOF_CPU_H__

#include <xtensa/config/core.h>

#define PLATFORM_CORE_COUNT 1

void arch_cpu_enable_core(int id);

void arch_cpu_disable_core(int id);

int arch_cpu_is_core_enabled(int id);

static inline int arch_cpu_get_id(void)
{
	int prid;
#if XCHAL_HAVE_PRID
	__asm__("rsr.prid %0" : "=a"(prid));
#else
	prid = PLATFORM_MASTER_CORE_ID;
#endif
	return prid;
}

static inline void cpu_write_threadptr(int threadptr)
{
#if XCHAL_HAVE_THREADPTR
	__asm__ __volatile__(
		"wur.threadptr %0" : : "a" (threadptr) : "memory");
#else
#error "Core support for XCHAL_HAVE_THREADPTR is required"
#endif
}

static inline int cpu_read_threadptr(void)
{
	int threadptr;
#if XCHAL_HAVE_THREADPTR
	__asm__ __volatile__(
		"rur.threadptr %0" : "=a"(threadptr));
#else
#error "Core support for XCHAL_HAVE_THREADPTR is required"
#endif
	return threadptr;
}


static inline int cpu_get_id(void)
{
	return arch_cpu_get_id();
}

static inline int cpu_is_core_enabled(int id)
{
        return 0;
}

static inline void cpu_enable_core(int id)
{
}

static inline void cpu_disable_core(int id)
{
}

#endif

#endif
