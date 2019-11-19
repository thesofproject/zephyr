/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <init.h>

#include <platform/ipc.h>
#include <platform/mailbox.h>
#include <platform/shim.h>
#include <platform/string.h>

#include "soc.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(soc_adsp, CONFIG_SOC_LOG_LEVEL);

static const struct adsp_ipc_fw_ready fw_ready_apl
	__attribute__((section(".fw_ready"))) __attribute__((used)) = {
	.hdr = {
		.cmd = ADSP_IPC_FW_READY,
		.size = sizeof(struct adsp_ipc_fw_ready),
	},
	.version = {
		.hdr.size = sizeof(struct adsp_ipc_fw_version),
		.micro = 0,
		.minor = 1,
		.major = 0,

		.build = 0,
		.date = __DATE__,
		.time = __TIME__,

		.tag = "zephyr",
		.abi_version = 0,
	},
	.flags = 0,
};

#define NUM_WINDOWS			7

static const struct adsp_ipc_window sram_window = {
	.ext_hdr = {
		.hdr.cmd = ADSP_IPC_FW_READY,
		.hdr.size = sizeof(struct adsp_ipc_window) +
			    sizeof(struct adsp_ipc_window_elem) * NUM_WINDOWS,
		.type = ADSP_IPC_EXT_WINDOW,
	},
	.num_windows = NUM_WINDOWS,
	.window = {
		{
			.type   = ADSP_IPC_REGION_REGS,
			.id     = 0,	/* map to host window 0 */
			.flags  = 0,
			.size   = MAILBOX_SW_REG_SIZE,
			.offset = 0,
		},
		{
			.type   = ADSP_IPC_REGION_UPBOX,
			.id     = 0,	/* map to host window 0 */
			.flags  = 0,
			.size   = MAILBOX_DSPBOX_SIZE,
			.offset = MAILBOX_SW_REG_SIZE,
		},
		{
			.type   = ADSP_IPC_REGION_DOWNBOX,
			.id     = 1,	/* map to host window 1 */
			.flags  = 0,
			.size   = MAILBOX_HOSTBOX_SIZE,
			.offset = 0,
		},
		{
			.type   = ADSP_IPC_REGION_DEBUG,
			.id     = 2,	/* map to host window 2 */
			.flags  = 0,
			.size   = MAILBOX_EXCEPTION_SIZE + MAILBOX_DEBUG_SIZE,
			.offset = 0,
		},
		{
			.type   = ADSP_IPC_REGION_EXCEPTION,
			.id     = 2,	/* map to host window 2 */
			.flags  = 0,
			.size   = MAILBOX_EXCEPTION_SIZE,
			.offset = MAILBOX_EXCEPTION_OFFSET,
		},
		{
			.type   = ADSP_IPC_REGION_STREAM,
			.id     = 2,	/* map to host window 2 */
			.flags  = 0,
			.size   = MAILBOX_STREAM_SIZE,
			.offset = MAILBOX_STREAM_OFFSET,
		},
		{
			.type   = ADSP_IPC_REGION_TRACE,
			.id     = 3,	/* map to host window 3 */
			.flags  = 0,
			.size   = MAILBOX_TRACE_SIZE,
			.offset = 0,
		},
	},
};

static void clk_and_pm_init()
{
	/* Setup clocks and power management */
	shim_write(SHIM_CLKCTL,
		   SHIM_CLKCTL_HDCS_PLL | /* HP domain clocked by PLL */
		   SHIM_CLKCTL_LDCS_PLL | /* LP domain clocked by PLL */
		   SHIM_CLKCTL_DPCS_DIV1(0) | /* Core 0 clk not divided */
		   SHIM_CLKCTL_DPCS_DIV1(1) | /* Core 1 clk not divided */
		   SHIM_CLKCTL_HPMPCS_DIV2 | /* HP mem clock div by 2 */
		   SHIM_CLKCTL_LPMPCS_DIV4 | /* LP mem clock div by 4 */
		   SHIM_CLKCTL_TCPAPLLS_DIS |
		   SHIM_CLKCTL_TCPLCG_DIS(0) | SHIM_CLKCTL_TCPLCG_DIS(1));

	shim_write(SHIM_LPSCTL, shim_read(SHIM_LPSCTL));
}

/*
 * Sets up the host windows so that the host can see the memory
 * content on the DSP SRAM.
 */
static void prepare_host_windows()
{
	/* window0, for fw status & outbox/uplink mbox */
	sys_write32((HP_SRAM_WIN0_SIZE | 0x7), DMWLO(0));
	sys_write32((HP_SRAM_WIN0_BASE | DMWBA_READONLY | DMWBA_ENABLE),
		    DMWBA(0));
	memset((void *)(HP_SRAM_WIN0_BASE + SRAM_REG_FW_END), 0,
	      HP_SRAM_WIN0_SIZE - SRAM_REG_FW_END);
	SOC_DCACHE_FLUSH((void *)(HP_SRAM_WIN0_BASE + SRAM_REG_FW_END),
			 HP_SRAM_WIN0_SIZE - SRAM_REG_FW_END);

	/* window1, for inbox/downlink mbox */
	sys_write32((HP_SRAM_WIN1_SIZE | 0x7), DMWLO(1));
	sys_write32((HP_SRAM_WIN1_BASE | DMWBA_ENABLE), DMWBA(1));
	memset((void *)HP_SRAM_WIN1_BASE, 0, HP_SRAM_WIN1_SIZE);
	SOC_DCACHE_FLUSH((void *)HP_SRAM_WIN1_BASE, HP_SRAM_WIN1_SIZE);

	/* window2, for debug */
	sys_write32((HP_SRAM_WIN2_SIZE | 0x7), DMWLO(2));
	sys_write32((HP_SRAM_WIN2_BASE | DMWBA_ENABLE), DMWBA(2));
	memset((void *)HP_SRAM_WIN2_BASE, 0, HP_SRAM_WIN2_SIZE);
	SOC_DCACHE_FLUSH((void *)HP_SRAM_WIN2_BASE, HP_SRAM_WIN2_SIZE);

	/* window3, for trace
	 * zeroed by trace initialization
	 */
	sys_write32((HP_SRAM_WIN3_SIZE | 0x7), DMWLO(3));
	sys_write32((HP_SRAM_WIN3_BASE | DMWBA_READONLY | DMWBA_ENABLE),
		    DMWBA(3));
	memset((void *)HP_SRAM_WIN3_BASE, 0, HP_SRAM_WIN3_SIZE);
	SOC_DCACHE_FLUSH((void *)HP_SRAM_WIN3_BASE, HP_SRAM_WIN3_SIZE);
}

static inline
void mailbox_dspbox_write(size_t offset, const void *src, size_t bytes)
{
	memcpy_s((void *)(MAILBOX_DSPBOX_BASE + offset),
			  MAILBOX_DSPBOX_SIZE - offset, src, bytes);
	SOC_DCACHE_FLUSH((void *)(MAILBOX_DSPBOX_BASE + offset), bytes);
}

/*
 * Sends the firmware ready message so the firmware loader can
 * map the host windows.
 */
static void send_fw_ready()
{
	mailbox_dspbox_write(0, &fw_ready_apl, sizeof(fw_ready_apl));
	mailbox_dspbox_write(sizeof(fw_ready_apl), &sram_window,
			     sram_window.ext_hdr.hdr.size);

	ipc_write(IPC_DIPCIE, (0x80000 >> 12));
	ipc_write(IPC_DIPCI, (0x80000000 | ADSP_IPC_FW_READY));
}

static int adsp_init(struct device *dev)
{
	clk_and_pm_init();

	prepare_host_windows();

	send_fw_ready();

	return 0;
}

SYS_INIT(adsp_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
