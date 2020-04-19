/* SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright(c) 2019 Intel Corporation. All rights reserved.
 *
 * Author: Tomasz Lauda <tomasz.lauda@linux.intel.com>
 */

#ifndef __CAVS_DRIVERS_INTERRUPT_H__
#define __CAVS_DRIVERS_INTERRUPT_H__

#include <platform/lib/clk.h>

extern const char irq_name_level2[];
extern const char irq_name_level3[];
extern const char irq_name_level4[];
extern const char irq_name_level5[];

#if CONFIG_CAVS_USE_LPRO_IN_WAITI
static inline void platform_interrupt_on_wakeup(void)
{
	platform_clock_on_wakeup();
}
#endif

#endif /* __CAVS_DRIVERS_INTERRUPT_H__ */

