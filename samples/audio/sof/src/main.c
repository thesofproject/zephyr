/*
 * Copyright (c) 2020 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>

/**
 * TODO: Here comes SOF initialization
 */

void main(void)
{
	printk("SOF on %s\n", CONFIG_BOARD);
}
