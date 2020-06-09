/*
 * Copyright (c) 2016 Cadence Design Systems, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <tracing/tracing.h>

#if (CONFIG_WAITI_DELAY)
static inline void arch_cpu_idle_delay(void)
{
	int i;
	uint32_t ps;

	/* this sequence must be atomic on LX6 */
	__asm__ __volatile__(	"rsil	%0, 5" : "=r"(ps));

	/* LX6 needs a delay */
	for (i = 0; i < 128; i++)
		__asm__ volatile("nop");

	/* and to flush all loads/stores prior to wait */
	__asm__ volatile("isync");
	__asm__ volatile("extw");
}

#endif

void arch_cpu_idle(void)
{
	sys_trace_idle();
#if(CONFIG_WAITI_DELAY)
	arch_cpu_idle_delay();
#endif
	__asm__ volatile ("waiti 0");
}
void arch_cpu_atomic_idle(unsigned int key)
{
	sys_trace_idle();
#if(CONFIG_WAITI_DELAY)
	arch_cpu_idle_delay();
#endif
	__asm__ volatile ("waiti 0\n\t"
			  "wsr.ps %0\n\t"
			  "rsync" :: "a"(key));
}

/* TODO: add logic for CAVS low power sequencer. */
#if 0
void platform_wait_for_interrupt(int level)
{
#if CONFIG_CAVS_USE_LPRO_IN_WAITI
	platform_clock_on_waiti();
#endif
#if (CONFIG_CAVS_LPS)
	if (pm_runtime_is_active(PM_RUNTIME_DSP, PLATFORM_MASTER_CORE_ID))
		arch_wait_for_interrupt(level);
	else
		lps_wait_for_interrupt(level);
#else
	arch_wait_for_interrupt(level);
#endif
}
#endif
