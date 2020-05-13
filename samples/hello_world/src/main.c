/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>

#define STACK_SIZE 2048
K_THREAD_STACK_DEFINE(stack, STACK_SIZE);

/* Weirdness with a non-array "first_dword" because -Warray-bounds is
 * too smart and flips out when we calculate the uncached mapping, no
 * matter how hard I try to hide the array.
 */
static __aligned(64) struct {
	u32_t first_dword;
	u32_t rest[15];
} cacheline;

/* Test rig to do an operation through cached/uncached pointers.  Logs
 * what is going to happen, executes it, makes sure the compiler
 * doens't muck with the results, and prints the old and new values
 */
#define CACHEOP(expr) do {				\
	printk("%s\n", #expr);			\
	u32_t u0 = *ucp, c0 = *cp;			\
	__asm__ volatile("" : : : "memory");		\
	expr;						\
	__asm__ volatile("" : : : "memory");		\
	printk(" :: *cp = %d -> %d, *ucp = %d -> %d\n", c0, *cp, u0, *ucp); \
} while(0)

static volatile void *uncache_ptr(volatile void *p)
{
	return (void*)(((long)p) & ~0x20000000);
}

void check_cache(void)
{
	for (int region = 0; region < 8; region++) {
		u32_t addr = 0x20000000 * region;
		u32_t attrib;

		__asm__ volatile("rdtlb1 %0, %1" : "=r"(attrib) : "r"(addr));
		printk("region %d @0x%x cachattr 0x%x\n", region, addr, attrib & 0xf);
	}

	/* Cached and uncached pointers to the same value. */
        long addr = (long)&cacheline.first_dword;
	volatile u32_t *cp = (void *)(addr | 0x20000000);
	volatile u32_t *ucp = (void *)(addr & ~0x20000000);

	printk("cp %p ucp %p\n", cp, ucp);

	*cp = *ucp = 0;

	/* Set the cached pointer to 1, only it should show the results */
	CACHEOP(*cp = 1);

	/* Set the uncached pointer, the cached one shouldn't change */
	CACHEOP(*ucp = 2);

	/* Flush the cache from before, BOTH should become 1 */
	CACHEOP(SOC_DCACHE_FLUSH((void*)cp, sizeof(*cp)));

	/* Now prime the cache with a new value, and invalidate it */
	CACHEOP(*cp = 3);
	CACHEOP(SOC_DCACHE_INVALIDATE((void*)cp, sizeof(*cp)));
}

FUNC_NORETURN void cputop(void *data)
{
	printk("Second CPU!\n");
	check_cache();

	while(1) {
		for(volatile int i=0; i<10000000; i++);
		printk(".\n");
	}
}

int __incoherent cached_int;
int __attribute__((__section__(".cached"))) cached2_int;

static int bss_int;

void main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD);

        int stack_int;
	printk("&cached_int = %p\n", &cached_int);
	printk("&cached2_int = %p\n", &cached2_int);
        printk("&bss_int = %p\n", &bss_int);
        printk("&stack_int = %p\n", &stack_int);

	check_cache();

	arch_start_cpu(1, stack, STACK_SIZE, cputop, NULL);

	while(1);
}
