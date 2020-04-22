/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>

/**
 * This should be included from
 *
 * #include <sof/lib/alloc.h>
 *
 * but it triggers include chain issue.
*/

#define SOF_MEM_CAPS_RAM                        (1 << 0)

enum mem_zone {
        SOF_MEM_ZONE_SYS = 0,           /**< System zone */
        SOF_MEM_ZONE_SYS_RUNTIME,       /**< System-runtime zone */
        SOF_MEM_ZONE_RUNTIME,           /**< Runtime zone */
        SOF_MEM_ZONE_BUFFER,            /**< Buffer zone */
};

void *rmalloc(enum mem_zone zone, uint32_t flags, uint32_t caps, size_t bytes);
void *rzalloc(enum mem_zone zone, uint32_t flags, uint32_t caps, size_t bytes);
void rfree(void *ptr);

#define BLK_SIZE_MIN 64
#define BLK_NUM_MAX (CONFIG_HEAP_MEM_POOL_SIZE / BLK_SIZE_MIN)
#define BLK_NUM_MIN (CONFIG_HEAP_MEM_POOL_SIZE / (BLK_SIZE_MIN << 2))

/* Use SOF_MEM_ZONE_BUFFER */

/**
 * Test rmalloc() and rfree() API functionality
 */
void test_audio_sof_alloc(void)
{
	void *block[2 * BLK_NUM_MAX], *block_fail;
	int nb;

	for (nb = 0; nb < ARRAY_SIZE(block); nb++) {
		block[nb] = rmalloc(SOF_MEM_ZONE_BUFFER, 0, SOF_MEM_CAPS_RAM,
				    BLK_SIZE_MIN);
		if (block[nb] == NULL) {
			break;
		}
	}

	block_fail = rmalloc(SOF_MEM_ZONE_BUFFER, 0, SOF_MEM_CAPS_RAM,
			     BLK_SIZE_MIN);
	/* Return NULL if fail.*/
	zassert_is_null(block_fail, NULL);

	for (int i = 0; i < nb; i++) {
		rfree(block[i]);
	}

	/* If ptr is NULL, no operation is performed.*/
	rfree(NULL);
}

#define SIZE    256

/**
 * Test to demonstrate rzalloc() API functionality.
 */
void test_audio_sof_rzalloc(void)
{
	char *mem;

	mem = rzalloc(SOF_MEM_ZONE_BUFFER, 0, SOF_MEM_CAPS_RAM, SIZE);

	zassert_not_null(mem, "calloc operation failed");

	/* Memory should be zeroed and not crash us if we read/write to it */
	for (int i = 0; i < SIZE; i++) {
		zassert_equal(mem[i], 0, NULL);
		mem[i] = 1;
	}

	rfree(mem);
}
