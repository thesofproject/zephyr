/* SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright(c) 2018 Intel Corporation. All rights reserved.
 *
 * Author: Tomasz Lauda <tomasz.lauda@linux.intel.com>
 */

/**
 * \file platform/icelake/include/platform/lib/pm_runtime.h
 * \brief Runtime power management header file for Icelake
 * \author Tomasz Lauda <tomasz.lauda@linux.intel.com>
 */

#ifndef __PLATFORM_LIB_PM_RUNTIME_H__
#define __PLATFORM_LIB_PM_RUNTIME_H__

#include <sof-config.h>
#include <cavs/lib/pm_runtime.h>
#include <stdint.h>

/**
 * \brief Initializes platform specific runtime power management.
 * \param[in,out] prd Runtime power management data.
 */
void platform_pm_runtime_init(struct pm_runtime_data *prd);

/**
 * \brief Retrieves platform specific power management resource.
 *
 * \param[in] context Type of power management context.
 * \param[in] index Index of the device.
 * \param[in] flags Flags, set of RPM_...
 */
void platform_pm_runtime_get(uint32_t context, uint32_t index, uint32_t flags);

/**
 * \brief Releases platform specific power management resource.
 *
 * \param[in] context Type of power management context.
 * \param[in] index Index of the device.
 * \param[in] flags Flags, set of RPM_...
 */
void platform_pm_runtime_put(uint32_t context, uint32_t index, uint32_t flags);

void platform_pm_runtime_enable(uint32_t context, uint32_t index);

void platform_pm_runtime_disable(uint32_t context, uint32_t index);

bool platform_pm_runtime_is_active(uint32_t context, uint32_t index);

/**
 * \brief Power gates platform specific hardware resources.
 */
void platform_pm_runtime_power_off(void);

#endif /* __PLATFORM_LIB_PM_RUNTIME_H__ */

