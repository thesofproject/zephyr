/*
 * Copyright (c) 2020 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT intel_cavs_idc

#include <stdint.h>
#include <device.h>
#include <init.h>
#include <drivers/ipm.h>
#include <arch/common/sys_io.h>

#include <soc.h>
#include <platform/shim.h>

#include "ipm_cavs_idc.h"

#ifdef CONFIG_SCHED_IPI_SUPPORTED
extern void z_sched_ipi(void);
#endif

struct cavs_idc_data {
	ipm_callback_t	cb;
	void		*ctx;
};

static struct device DEVICE_NAME_GET(cavs_idc);
static struct cavs_idc_data cavs_idc_device_data;

static void cavs_idc_isr(struct device *dev)
{
	struct cavs_idc_data *drv_data = dev->driver_data;

	u32_t i, id;
	void *ext;
	u32_t idctfc;
	u32_t curr_cpu_id = arch_curr_cpu()->id;
#ifdef CONFIG_SCHED_IPI_SUPPORTED
	bool do_sched_ipi = false;
#endif

	for (i = 0; i < CONFIG_MP_NUM_CPUS; i++) {
		if (i == curr_cpu_id) {
			/* skip current core */
			continue;
		}

		idctfc = idc_read(IPC_IDCTFC(i), curr_cpu_id);

		if ((idctfc & IPC_IDCTFC_BUSY) == 0) {
			/* No message from this core */
			continue;
		}

		/* Extract the message */
		id = idctfc & IPC_IDCTFC_MSG_MASK;

		switch (id) {
#ifdef CONFIG_SCHED_IPI_SUPPORTED
		case IPM_CAVS_IDC_MSG_SCHED_IPI_ID:
			do_sched_ipi = true;
			break;
#endif
		default:
			if (drv_data->cb != NULL) {
				ext = UINT_TO_POINTER(
					idc_read(IPC_IDCTEFC(i), curr_cpu_id) &
					IPC_IDCTEFC_MSG_MASK);
				drv_data->cb(drv_data->ctx, id, ext);
			}
			break;
		}

		/* Reset busy bit by writing to it */
		idctfc |= IPC_IDCTFC_BUSY;
		idc_write(IPC_IDCTFC(i), curr_cpu_id, idctfc);
	}
#ifdef CONFIG_SCHED_IPI_SUPPORTED
	if (do_sched_ipi) {
		z_sched_ipi();
	}
#endif
}

static int cavs_idc_send(struct device *dev, int wait, u32_t id,
			 const void *data, int size)
{
	u32_t curr_cpu_id = arch_curr_cpu()->id;
	u32_t ext = POINTER_TO_UINT(data);
	u32_t reg;
	bool busy;
	int i;

	if ((wait != 0) || (size != 0)) {
		return -ENOTSUP;
	}

	/* Check if any core is still busy */
	busy = false;
	for (i = 0; i < CONFIG_MP_NUM_CPUS; i++) {
		if (i == curr_cpu_id) {
			/* skip current core */
			continue;
		}

		reg = idc_read(IPC_IDCITC(i), curr_cpu_id);
		if ((reg & IPC_IDCITC_BUSY) != 0) {
			busy = true;
			break;
		}
	}

	/* Can't send if busy */
	if (busy) {
		return -EBUSY;
	}

	id &= IPC_IDCITC_MSG_MASK;
	ext &= IPC_IDCIETC_MSG_MASK;
	ext |= IPC_IDCIETC_DONE; /* always clear DONE bit */

	for (i = 0; i < CONFIG_MP_NUM_CPUS; i++) {
		if (i == curr_cpu_id) {
			/* skip current core */
			continue;
		}

		idc_write(IPC_IDCIETC(i), curr_cpu_id, ext);
		idc_write(IPC_IDCITC(i), curr_cpu_id, id | IPC_IDCITC_BUSY);
	}

	return 0;
}

static int cavs_idc_max_data_size_get(struct device *dev)
{
	ARG_UNUSED(dev);

	/* IDC can send an ID (of 31 bits, the header) and
	 * another data of 30 bits (the extension). It cannot
	 * send a whole message over. Best we can do is send
	 * a 4-byte aligned pointer.
	 *
	 * So return 0 here for max data size.
	 */

	return 0;
}

static u32_t cavs_idc_max_id_val_get(struct device *dev)
{
	ARG_UNUSED(dev);

	return IPM_CAVS_IDC_ID_MASK;
}

static void cavs_idc_register_callback(struct device *dev, ipm_callback_t cb,
				       void *context)
{
	struct cavs_idc_data *drv_data = dev->driver_data;

	drv_data->cb = cb;
	drv_data->ctx = context;
}

static int cavs_idc_set_enabled(struct device *dev, int enable)
{
	int i, j;
	u32_t mask;

#ifdef CONFIG_SCHED_IPI_SUPPORTED
	/* With scheduler IPI, IDC must always be enabled. */
	if (enable == 0) {
		return -ENOTSUP;
	}
#endif

	for (i = 0; i < CONFIG_MP_NUM_CPUS; i++) {
		mask = 0;

		if (enable) {
			for (j = 0; j < CONFIG_MP_NUM_CPUS; j++) {
				if (i == j) {
					continue;
				}

				mask |= IPC_IDCCTL_IDCTBIE(j);
			}
		}

		idc_write(IPC_IDCCTL, i, mask);

		/* FIXME: when we have API to enable IRQ on specific core. */
		sys_set_bit(DT_CAVS_ICTL_BASE_ADDR + 0x04 +
			    CAVS_ICTL_INT_CPU_OFFSET(i),
			    CAVS_IRQ_NUMBER(DT_INST_IRQN(0)));
	}

	return 0;
}

static int cavs_idc_init(struct device *dev)
{
	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    cavs_idc_isr, DEVICE_GET(cavs_idc), 0);

	irq_enable(DT_INST_IRQN(0));

	return 0;
}

static const struct ipm_driver_api cavs_idc_driver_api = {
	.send = cavs_idc_send,
	.register_callback = cavs_idc_register_callback,
	.max_data_size_get = cavs_idc_max_data_size_get,
	.max_id_val_get = cavs_idc_max_id_val_get,
	.set_enabled = cavs_idc_set_enabled,
};

DEVICE_AND_API_INIT(IPM_CAVS_IDC_DEV_NAME,
		    DT_INST_LABEL(0),
		    &cavs_idc_init, &cavs_idc_device_data, NULL,
		    PRE_KERNEL_2, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &cavs_idc_driver_api);

#ifdef CONFIG_SCHED_IPI_SUPPORTED
static int cavs_idc_smp_init(struct device *dev)
{
	/* Enable IDC for scheduler IPI */
	cavs_idc_set_enabled(dev, 1);

	return 0;
}

SYS_INIT(cavs_idc_smp_init, SMP, 0);
#endif
