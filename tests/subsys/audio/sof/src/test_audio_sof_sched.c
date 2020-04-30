/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>

/**
 * Safe to include headers
 */
#include <sof/list.h>
#include <sof/schedule/task.h>
#include <sof/schedule/schedule.h>

/**
 * TODO: Include from headers
 */
int schedule_task_init_edf(struct task *task, uint32_t uid,
			   const struct task_ops *ops,
			   void *data, uint16_t core, uint32_t flags);

int schedule_task_init_ll(struct task *task,
			  uint32_t uid, uint16_t type, uint16_t priority,
			  enum task_state (*run)(void *data), void *data,
			  uint16_t core, uint32_t flags);

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

static void test_task_init_edf(struct task *task, void *data)
{
	int ret;

	ret = schedule_task_init_edf(task, 0, &ops, data, 0, 0);
	zassert_equal(ret, 0, "Task init failed");
}

static enum task_state test_task_ll(void *data)
{
	struct k_sem *sem = data;

	TC_PRINT("Task %s running\n", __func__);

	k_sem_give(sem);

	return SOF_TASK_STATE_COMPLETED;
}

static void test_task_init_ll(struct task *task, void *data)
{
	int ret;

	ret = schedule_task_init_ll(task, 0, 0, 0, test_task_ll, data, 0, 0);
	zassert_equal(ret, 0, "Task init failed");
}

#define NUM_TASKS	5

void test_audio_sof_sched_edf(void)
{
	struct task tasks_edf[NUM_TASKS], tasks_ll[NUM_TASKS];
	struct k_sem task_sync_sem;
	int ret;
	int i;

	ret = k_sem_init(&task_sync_sem, 0, ARRAY_SIZE(tasks_edf));
	zassert_equal(ret, 0, "Semaphore initialization failed");

	for (i = 0; i < ARRAY_SIZE(tasks_edf); i++) {
		TC_PRINT("Initializing edf task %d\n", i);
		test_task_init_edf(&tasks_edf[i], &task_sync_sem);

		ret = schedule_task(&tasks_edf[i], 0, 0);
		zassert_equal(ret, 0, "Scheduling task failed");
	}

	/* Wait for all edf tasks */
	for (i = 0; i < ARRAY_SIZE(tasks_edf); i++) {
		k_sem_take(&task_sync_sem, K_FOREVER);
	}

	TC_PRINT("All %d edf tasks are finished\n", i);

	k_sem_reset(&task_sync_sem);

	for (i = 0; i < ARRAY_SIZE(tasks_ll); i++) {
		TC_PRINT("Initializing ll task %d\n", i);
		test_task_init_ll(&tasks_ll[i], &task_sync_sem);

		ret = schedule_task(&tasks_ll[i], 0, 0);
		zassert_equal(ret, 0, "Scheduling task failed");
	}

	/* Wait for all ll tasks */
	for (i = 0; i < ARRAY_SIZE(tasks_edf); i++) {
		k_sem_take(&task_sync_sem, K_FOREVER);
	}

	TC_PRINT("All %d ll tasks are finished\n", i);
}
