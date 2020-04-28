/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>

extern void test_audio_sof_alloc(void);
extern void test_audio_sof_rzalloc(void);
extern void test_audio_sof_irq(void);
void test_audio_sof_sched_edf(void);

/*test case main entry*/
void test_main(void)
{
	ztest_test_suite(audio_sof,
			 ztest_unit_test(test_audio_sof_alloc),
			 ztest_unit_test(test_audio_sof_rzalloc),
			 ztest_unit_test(test_audio_sof_irq),
			 ztest_unit_test(test_audio_sof_sched_edf));
	ztest_run_test_suite(audio_sof);
}
