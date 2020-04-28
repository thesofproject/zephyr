/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* The test origin is dynamic_isr.c adapted for SOF API testing */

#include <ztest.h>
extern struct _isr_table_entry __sw_isr_table _sw_isr_table[];
extern void z_irq_spurious(void *unused);

static void dyn_isr(void *arg)
{
	ARG_UNUSED(arg);
}

/* TODO: Include tight header */
int interrupt_register(uint32_t irq, void(*handler)(void *arg), void *arg);

/**
 * Test dynamic ISR installation
 *
 * This routine locates an unused entry in the software ISR table, installs a
 * dynamic ISR to the unused entry by calling the `arch_irq_connect_dynamic`
 * function, and verifies that the ISR is successfully installed by checking
 * the software ISR table entry.
 */
void test_audio_sof_irq(void)
{
	int i;
	void *argval;

	for (i = 0; i < (CONFIG_NUM_IRQS - CONFIG_GEN_IRQ_START_VECTOR); i++) {
		if (_sw_isr_table[i].isr == z_irq_spurious) {
			break;
		}
	}

	zassert_true(_sw_isr_table[i].isr == z_irq_spurious,
		     "could not find slot for dynamic isr");

	TC_PRINT("Installing dynamic ISR for IRQ %d\n",
		 CONFIG_GEN_IRQ_START_VECTOR + i);

	argval = &i;
	interrupt_register(i + CONFIG_GEN_IRQ_START_VECTOR, dyn_isr, argval);

	zassert_true(_sw_isr_table[i].isr == dyn_isr &&
		     _sw_isr_table[i].arg == argval,
		     "dynamic isr did not install successfully");
}
