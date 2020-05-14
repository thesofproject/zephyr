/*
 * Copyright (c) 2020 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/**
 * Should be included from sof/schedule/task.h
 * but triggers include chain issue
 * FIXME
 */
int task_main_start(void);

/**
 * TODO: Here comes SOF initialization
 */

void main(void)
{
	int ret;

	LOG_INF("SOF on %s", CONFIG_BOARD);

	/* task_main_start is actually SOF initialization */
	ret = task_main_start();
	if (ret) {
		LOG_ERR("SOF initialization failed");
	}

	LOG_INF("SOF initialized");
}
