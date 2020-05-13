/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>
#include <adsp/cache.h>

/* Simple char-at-a-time output rig to the host kernel from a ADSP
 * device.  The protocol uses an array of "slots" in shared memory,
 * each of which has a 16 bit magic number to validate and a
 * sequential ID number.  The remaining bytes are a (potentially
 * nul-terminated) string containing output data.
 *
 * IMPORTANT NOTE on cache coherence: the shared memory window is in
 * HP-SRAM.  Each DSP core has an L1 cache that is incoherent (!) from
 * the perspective of the other cores.  To handle this, we take care
 * to access all memory through the uncached window into HP-SRAM at
 * 0x9xxxxxxx and not the L1-cached mapping of the same memory at
 * 0xBxxxxxxx.
 */

#define SLOT_SIZE 64
#define SLOT_MAGIC 0x55aa

#define NSLOTS (CONFIG_ADSP_LOG_WIN_SIZE / SLOT_SIZE)
#define MSGSZ (SLOT_SIZE - sizeof(struct slot_hdr))

struct slot_hdr {
	u16_t magic;
	u16_t id;
};

struct slot {
	struct slot_hdr hdr;
	char msg[MSGSZ];
};

struct metadata {
	struct k_spinlock lock;
	int initialized;
	int curr_slot;   /* To which slot are we writing? */
	int n_bytes;     /* How many bytes buffered in curr_slot */
};

/* Give it a cache line all its own! */
static __aligned(64) union {
	struct metadata meta;
	u32_t cache_pad[16];
} data_rec;

/* Force to the low/uncached mapping, regardless of how it was linked */
#define data ((struct metadata *)(((long) &data_rec.meta) & ~0x20000000))

static inline struct slot *slot(int i)
{
	struct slot *slots = (struct slot *)(long) CONFIG_ADSP_LOG_WIN_BASE;

	return &slots[i];
}

int arch_printk_char_out(int c)
{
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	if (!data->initialized) {
		slot(0)->hdr.magic = 0;
		slot(0)->hdr.id = 0;
		data->initialized = 1;
	}

	struct slot *s = slot(data->curr_slot);

	s->msg[data->n_bytes++] = c;

	if (data->n_bytes < MSGSZ) {
		s->msg[data->n_bytes] = 0;
	}

	if (c == '\n' || data->n_bytes >= MSGSZ) {
		data->curr_slot = (data->curr_slot + 1) % NSLOTS;
		data->n_bytes = 0;
		slot(data->curr_slot)->hdr.magic = 0;
		slot(data->curr_slot)->hdr.id = s->hdr.id + 1;
		s->hdr.magic = SLOT_MAGIC;
	}

	k_spin_unlock(&data->lock, key);
	return 0;
}
