/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>

/*
 * Including the header below trigger include chain issue
 */
enum mem_zone {
        SOF_MEM_ZONE_SYS = 0,           /**< System zone */
        SOF_MEM_ZONE_SYS_RUNTIME,       /**< System-runtime zone */
        SOF_MEM_ZONE_RUNTIME,           /**< Runtime zone */
        SOF_MEM_ZONE_BUFFER,            /**< Buffer zone */
};

void *rzalloc(enum mem_zone zone, uint32_t flags, uint32_t caps, size_t bytes);

/**
 * Safe to include headers
 */
#include <sof/list.h>
#include <sof/schedule/task.h>
#include <sof/schedule/schedule.h>

int schedule_task_init_edf(struct task *task, uint32_t uid,
			   const struct task_ops *ops,
			   void *data, uint16_t core, uint32_t flags);

#define SOF_UUID(uuid_name) 0

static enum task_state test_task(void *data)
{
	struct k_sem *sem = data;

	TC_PRINT("Task %s running\n", __func__);

	k_sem_give(sem);

	return SOF_TASK_STATE_COMPLETED;
}

static uint64_t task_main_deadline(void *data)
{
	return SOF_TASK_DEADLINE_IDLE;
}

struct task_ops ops = {
	.run = test_task,
	.get_deadline = task_main_deadline,
};

static bool test_task_init(struct task **task, void *data)
{
	int ret;

	TC_PRINT("Initializing task\n");

	*task = rzalloc(SOF_MEM_ZONE_SYS, 0, 0, sizeof(**task));
	zassert_not_null(*task, "Memory allocation failed");

	ret = schedule_task_init_edf(*task, SOF_UUID(main_task_uuid),
				     &ops, data, 0, 0);
	zassert_equal(ret, 0, "Task init failed");

	return ret;
}

static struct k_sem task_sync_sem;

void test_audio_sof_sched_edf(void)
{
	struct task *task;
	int ret;

	ret = k_sem_init(&task_sync_sem, 0, 1);
	zassert_equal(ret, 0, "Semaphore initialization failed");

	ret = test_task_init(&task, &task_sync_sem);

	ret = schedule_task(task, 0, 0);
	zassert_equal(ret, 0, "Scheduling task failed");

	/* Wait for task */
	k_sem_take(&task_sync_sem, K_FOREVER);
}
