/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright(c) 2016 Intel Corporation. All rights reserved.
 *
 * Author: Liam Girdwood <liam.r.girdwood@linux.intel.com>
 *         Keyon Jie <yang.jie@linux.intel.com>
 */

#ifndef __PLATFORM_MEMORY_H__
#define __PLATFORM_MEMORY_H__

#include <cavs/lib/memory.h>

/* Memory banks */

#define NUM_LP_MEMORY_BANKS		2

#define NUM_HP_MEMORY_BANKS		8

#define SRAM_BANK_SIZE                  (64 * 1024)

#define EBB_BANKS_IN_SEGMENT            32

#define EBB_SEGMENT_SIZE                EBB_BANKS_IN_SEGMENT


/* physical DSP addresses */

/* shim */
#define SHIM_BASE		0x00001000
#define SHIM_SIZE		0x00000100

/* cmd IO to audio codecs */
#define CMD_BASE		0x00001100
#define CMD_SIZE		0x00000010

/* resource allocation */
#define RES_BASE		0x00001110
#define RES_SIZE		0x00000010

/* IPC to the host */
#define IPC_HOST_BASE		0x00001180
#define IPC_HOST_SIZE		0x00000020

/* intra DSP  IPC */
#define IPC_DSP_SIZE		0x00000080
#define IPC_DSP_BASE(x)		(0x00001200 + x * IPC_DSP_SIZE)

/* SRAM window for HOST */
#define HOST_WIN_SIZE		0x00000008
#define HOST_WIN_BASE(x)	(0x00001580 + x * HOST_WIN_SIZE)

/* IRQ controller */
#define IRQ_BASE		0x00001600
#define IRQ_SIZE		0x00000200

/* time stamping */
#define TIME_BASE		0x00001800
#define TIME_SIZE		0x00000200

/* M/N dividers */
#define MN_BASE			0x00008E00
#define MN_SIZE			0x00000200

/* low power DMA position */
#define LP_GP_DMA_LINK_SIZE	0x00000010
#define LP_GP_DMA_LINK_BASE(x) (0x00001C00 + x * LP_GP_DMA_LINK_SIZE)

/* high performance DMA position */
#define HP_GP_DMA_LINK_SIZE	0x00000010
#define HP_GP_DMA_LINK_BASE(x)	(0x00001D00 + x * HP_GP_DMA_LINK_SIZE)

/* link DMAC stream */
#define GTW_LINK_OUT_STREAM_SIZE	0x00000020
#define GTW_LINK_OUT_STREAM_BASE(x) \
	(0x00002400 + x * GTW_LINK_OUT_STREAM_SIZE)

#define GTW_LINK_IN_STREAM_SIZE	0x00000020
#define GTW_LINK_IN_STREAM_BASE(x) \
	(0x00002600 + x * GTW_LINK_IN_STREAM_SIZE)

/* host DMAC stream */
#define GTW_HOST_OUT_STREAM_SIZE	0x00000040
#define GTW_HOST_OUT_STREAM_BASE(x) \
	(0x00002800 + x * GTW_HOST_OUT_STREAM_SIZE)

#define GTW_HOST_IN_STREAM_SIZE		0x00000040
#define GTW_HOST_IN_STREAM_BASE(x) \
	(0x00002C00 + x * GTW_HOST_IN_STREAM_SIZE)

/* code loader */
#define GTW_CODE_LDR_SIZE	0x00000040
#define GTW_CODE_LDR_BASE	0x00002BC0

/* L2 TLBs */
#define L2_HP_SRAM_TLB_SIZE	0x00001000
#define L2_HP_SRAM_TLB_BASE	0x00003000

/* DMICs */
#define DMIC_BASE		0x00004000
#define DMIC_SIZE		0x00004000

/* SSP */
#define SSP_SIZE		0x0000200
#define SSP_BASE(x)		(0x00008000 + x * SSP_SIZE)

/* Timestamp */
#define TIMESTAMP_BASE		0x00001800

/* low power DMACs */
#define LP_GP_DMA_SIZE		0x00001000
#define LP_GP_DMA_BASE(x)	(0x0000C000 + x * LP_GP_DMA_SIZE)

/* high performance DMACs */
#define HP_GP_DMA_SIZE		0x00001000
#define HP_GP_DMA_BASE(x)	(0x0000E000 + x * HP_GP_DMA_SIZE)

/* ROM */
#define ROM_BASE		0xBEFE0000
#define ROM_SIZE		0x00002000

/* IMR accessible via L2$ */
#define L2_SRAM_BASE		0xA000A000
#define L2_SRAM_SIZE		0x00056000

/* SRAM window for HOST */
#define HOST_WIN_SIZE		0x00000008
#define HOST_WIN_BASE(x)	(0x00001580 + x * HOST_WIN_SIZE)

#define L2_VECTOR_SIZE			0x1000

#define UUID_ENTRY_ELF_BASE	0x1FFFA000
#define UUID_ENTRY_ELF_SIZE	0x6000

#define LOG_ENTRY_ELF_BASE	0x20000000
#define LOG_ENTRY_ELF_SIZE	0x2000000

/*
 * The HP SRAM Region Apollolake is organised like this :-
 * +--------------------------------------------------------------------------+
 * | Offset              | Region         |  Size                             |
 * +---------------------+----------------+-----------------------------------+
 * | HP_SRAM_BASE        | DMA            |  HEAP_HP_BUFFER_SIZE              |
 * +---------------------+----------------+-----------------------------------+
 * | SRAM_TRACE_BASE     | Trace Buffer W3|  SRAM_TRACE_SIZE                  |
 * +---------------------+----------------+-----------------------------------+
 * | SRAM_DEBUG_BASE     | Debug data  W2 |  SRAM_DEBUG_SIZE                  |
 * +---------------------+----------------+-----------------------------------+
 * | SRAM_EXCEPT_BASE    | Debug data  W2 |  SRAM_EXCEPT_SIZE                 |
 * +---------------------+----------------+-----------------------------------+
 * | SRAM_STREAM_BASE    | Stream data W2 |  SRAM_STREAM_SIZE                 |
 * +---------------------+----------------+-----------------------------------+
 * | SRAM_INBOX_BASE     | Inbox  W1      |  SRAM_INBOX_SIZE                  |
 * +---------------------+----------------+-----------------------------------+
 * | SRAM_SW_REG_BASE    | SW Registers W0|  SRAM_SW_REG_SIZE                 |
 * +---------------------+----------------+-----------------------------------+
 * | SRAM_OUTBOX_BASE    | Outbox W0      |  SRAM_MAILBOX_SIZE                |
 * +---------------------+----------------+-----------------------------------+
 */

/* HP SRAM */

#define HP_SRAM_BASE		0xBE000000
#define HP_SRAM_MASK		0xFF000000

/* HP SRAM Heap */
#define HEAP_HP_BUFFER_BASE	HP_SRAM_BASE
#define HEAP_HP_BUFFER_SIZE	0x8000

#define HEAP_HP_BUFFER_BLOCK_SIZE	0x180
#define HEAP_HP_BUFFER_COUNT \
	(HEAP_HP_BUFFER_SIZE / HEAP_HP_BUFFER_BLOCK_SIZE)

/* HP SRAM windows */

/* window 3 */
#define SRAM_TRACE_BASE		SRAM_WND_BASE
#define SRAM_TRACE_SIZE		0x2000

/* window 2 */
#define SRAM_DEBUG_BASE		(SRAM_TRACE_BASE + SRAM_TRACE_SIZE)
#define SRAM_DEBUG_SIZE		0x800

#define SRAM_EXCEPT_BASE	(SRAM_DEBUG_BASE + SRAM_DEBUG_SIZE)
#define SRAM_EXCEPT_SIZE	0x800

#define SRAM_STREAM_BASE	(SRAM_EXCEPT_BASE + SRAM_EXCEPT_SIZE)
#define SRAM_STREAM_SIZE	0x1000

/* window 1 */
#define SRAM_INBOX_BASE		(SRAM_STREAM_BASE + SRAM_STREAM_SIZE)
#define SRAM_INBOX_SIZE		0x2000

/* window 0 */
#define SRAM_SW_REG_BASE	(SRAM_INBOX_BASE + SRAM_INBOX_SIZE)
#define SRAM_SW_REG_SIZE	0x1000

/* SRAM window 0 FW "registers" */
#define SRAM_REG_ROM_STATUS			0x0
#define SRAM_REG_FW_STATUS			0x4
#define SRAM_REG_FW_TRACEP			0x8
#define SRAM_REG_FW_IPC_RECEIVED_COUNT		0xc
#define SRAM_REG_FW_IPC_PROCESSED_COUNT		0x10
#define SRAM_REG_FW_TRACEP_SLAVE_CORE_BASE	0x14
#define SRAM_REG_FW_END \
	(SRAM_REG_FW_TRACEP_SLAVE_CORE_BASE + (PLATFORM_CORE_COUNT - 1) * 0x4)

#define SRAM_OUTBOX_BASE	(SRAM_SW_REG_BASE + SRAM_SW_REG_SIZE)
#define SRAM_OUTBOX_SIZE	0x1000

#define HP_SRAM_WIN0_BASE	SRAM_SW_REG_BASE
#define HP_SRAM_WIN0_SIZE	(SRAM_SW_REG_SIZE + SRAM_OUTBOX_SIZE)
#define HP_SRAM_WIN1_BASE	SRAM_INBOX_BASE
#define HP_SRAM_WIN1_SIZE	SRAM_INBOX_SIZE
#define HP_SRAM_WIN2_BASE	SRAM_DEBUG_BASE
#define HP_SRAM_WIN2_SIZE	(SRAM_DEBUG_SIZE + SRAM_EXCEPT_SIZE + \
				SRAM_STREAM_SIZE)
#define HP_SRAM_WIN3_BASE	SRAM_TRACE_BASE
#define HP_SRAM_WIN3_SIZE	SRAM_TRACE_SIZE

/* Apollolake HP-SRAM config */
#define SRAM_ALIAS_OFFSET	0x20000000

#define SRAM_WND_BASE		(HEAP_HP_BUFFER_BASE + HEAP_HP_BUFFER_SIZE)

#define HP_SRAM_VECBASE_RESET	(HP_SRAM_WIN0_BASE + HP_SRAM_WIN0_SIZE)
#define HP_SRAM_VECBASE_OFFSET	0x0

#define SOF_FW_START		(HP_SRAM_VECBASE_RESET + 0x400)
#define SOF_FW_BASE		(SOF_FW_START)

#define PLATFORM_HEAP_SYSTEM		PLATFORM_CORE_COUNT /* one per core */
#define PLATFORM_HEAP_SYSTEM_RUNTIME	PLATFORM_CORE_COUNT /* one per core */
#define PLATFORM_HEAP_RUNTIME		1
#define PLATFORM_HEAP_BUFFER		2

#define HEAP_BUFFER_SIZE	0x10000

/* max size for all var-size sections (text/rodata/bss) */
#define SOF_FW_MAX_SIZE		(0x41000 - 0x400)

#define SOF_TEXT_START		(SOF_FW_START)
#define SOF_TEXT_BASE		(SOF_FW_START)

/* Stack configuration */
#define SOF_STACK_BASE		(HP_SRAM_BASE + HP_SRAM_SIZE)
#define SOF_STACK_END		(SOF_STACK_BASE - SOF_STACK_TOTAL_SIZE)

#define SOF_MEMORY_SIZE		(SOF_STACK_BASE - HP_SRAM_BASE)

/* LP SRAM */
#define LP_SRAM_BASE		0xBE800000

/* boot loader in IMR */
#define IMR_BOOT_LDR_TEXT_ENTRY_BASE	0xB000A000

#define IMR_BOOT_LDR_TEXT_ENTRY_SIZE	0x86

/* Manifest base address in IMR - used by boot loader copy procedure. */
#define IMR_BOOT_LDR_MANIFEST_BASE	0xB0004000

/* Manifest size (seems unused). */
#define IMR_BOOT_LDR_MANIFEST_SIZE	0x6000

#endif
