/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __INCLUDE_PLATFORM_STRING_H__
#define __INCLUDE_PLATFORM_STRING_H__

static inline int memcpy_s(void *dest, size_t dest_size,
			   const void *src, size_t src_size)
{
	if (!dest || !src) {
		return -EINVAL;
	}

	if (((char *)dest + dest_size >= (char *)src &&
	     (char *)dest + dest_size <= (char *)src + src_size) ||
		((char *)src + src_size >= (char *)dest &&
		 (char *)src + src_size <= (char *)dest + dest_size)) {
		return -EINVAL;
	}

	if (src_size > dest_size) {
		return -EINVAL;
	}

	memcpy(dest, src, src_size);

	return 0;
}

static inline int memset_s(void *dest, size_t dest_size,
			   int data, size_t count)
{
	if (!dest) {
		return -EINVAL;
	}

	if (count > dest_size) {
		return -EINVAL;
	}

	memset(dest, data, count);

	return 0;
}

#endif /* __INCLUDE_PLATFORM_STRING_H__ */
