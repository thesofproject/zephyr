/*
 * Copyright (c) 2019, Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <device.h>
#include <soc.h>
#include <drivers/ipm.h>

#include <platform/mailbox.h>
#include <platform/shim.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(ipm_adsp, CONFIG_IPM_LOG_LEVEL);

/* FIXME define in DTS */
#define DT_IPM_ADSP_0_BASE_ADDRESS	0x10
#define DT_IPM_ADSP_0_NAME		"IPM_0"
#define DT_IPM_ADSP_0_IRQ		SOC_AGGREGATE_IRQ(7, 6)
#define DT_IPM_ADSP_0_IRQ_PRI		3

/* FIXME: set to 64 for now */
#define IPM_ADSP_MAX_DATA_SIZE		256
#define IPM_ADSP_MAX_ID_VAL		IPC_DIPCI_MSG_MASK

/* Mailbox ADSP -> Host */
#define IPM_ADSP_MAILBOX_OUT		MAILBOX_DSPBOX_BASE
#define IPM_ADSP_MAILBOX_OUT_SIZE	MAILBOX_DSPBOX_SIZE

/* Mailbox Host -> ADSP */
#define IPM_ADSP_MAILBOX_IN		MAILBOX_HOSTBOX_BASE
#define IPM_ADSP_MAILBOX_IN_SIZE	MAILBOX_HOSTBOX_SIZE

struct ipm_adsp_config {
	u8_t *base;
	void (*irq_config_func)(struct device *dev);
};

struct ipm_adsp_data {
	ipm_callback_t callback;
	void *callback_ctx;
};

static void ipm_adsp_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	const struct ipm_adsp_data *data = dev->driver_data;
	uint32_t dipcctl, dipcie, dipct;

	dipct = ipc_read(IPC_DIPCT);
	dipcie = ipc_read(IPC_DIPCIE);
	dipcctl = ipc_read(IPC_DIPCCTL);

	LOG_DBG("dipct 0x%x dipcie 0x%x dipcctl 0x%x", dipct, dipcie, dipcctl);

	/* IPC Target Busy Interrupt Enable (IPCTBIE) */
	if (dipct & IPC_DIPCT_BUSY && dipcctl & IPC_DIPCCTL_IPCTBIE) {
		/* Mask Busy interrupt */
		ipc_write(IPC_DIPCCTL, dipcctl & ~IPC_DIPCCTL_IPCTBIE);

		if (data->callback) {
			SOC_DCACHE_INVALIDATE((void *)IPM_ADSP_MAILBOX_IN,
					      IPM_ADSP_MAILBOX_IN_SIZE);
			/* Use zero copy */
			data->callback(data->callback_ctx,
				       dipct & IPC_DIPCI_MSG_MASK,
				       (void *)IPM_ADSP_MAILBOX_IN);
		}

		/* Clear BUSY enabling interrupts */
		ipc_write(IPC_DIPCT, ipc_read(IPC_DIPCT) | IPC_DIPCT_BUSY);

		/* Unmask interrupts */
		ipc_write(IPC_DIPCCTL,
			  ipc_read(IPC_DIPCCTL) | IPC_DIPCCTL_IPCTBIE);
	}

	if (dipcie & IPC_DIPCIE_DONE && dipcctl & IPC_DIPCCTL_IPCIDIE) {
		/* Mask Done interrupt */
		ipc_write(IPC_DIPCCTL,
			  ipc_read(IPC_DIPCCTL) & ~IPC_DIPCCTL_IPCIDIE);

		/* Clear DONE bit - tell host we have completed the operation */
		ipc_write(IPC_DIPCIE,
			  ipc_read(IPC_DIPCIE) | IPC_DIPCIE_DONE);

		/* Unmask Done interrupt */
		ipc_write(IPC_DIPCCTL,
			  ipc_read(IPC_DIPCCTL) | IPC_DIPCCTL_IPCIDIE);

		LOG_DBG("Not handled: IPC_DIPCCTL_IPCIDIE");

		/* TODO: implement queued message sending if needed */
	}
}

static int ipm_adsp_send(struct device *dev, int wait, u32_t id,
			 const void *data, int size)
{
	LOG_DBG("Send: id %d data %p size %d", id, data, size);
	LOG_HEXDUMP_DBG(data, size, "send");

	if (id > IPM_ADSP_MAX_ID_VAL) {
		return -EINVAL;
	}

	if (size > IPM_ADSP_MAX_DATA_SIZE) {
		return -EMSGSIZE;
	}

	if (wait) {
		while(ipc_read(IPC_DIPCI) & IPC_DIPCI_BUSY) {
		}
	} else {
		if (ipc_read(IPC_DIPCI) & IPC_DIPCI_BUSY) {
			LOG_DBG("Busy: previous message is not handled");
			return -EBUSY;
		}
	}

	memcpy((void *)IPM_ADSP_MAILBOX_OUT, data, size);
	SOC_DCACHE_FLUSH((void *)IPM_ADSP_MAILBOX_OUT, size);

	ipc_write(IPC_DIPCIE, 0);
	ipc_write(IPC_DIPCI, IPC_DIPCI_BUSY | id);

	return 0;
}

static void ipm_adsp_register_callback(struct device *dev,
				       ipm_callback_t cb,
				       void *context)
{
	struct ipm_adsp_data *data = dev->driver_data;

	data->callback = cb;
	data->callback_ctx = context;
}

static int ipm_adsp_max_data_size_get(struct device *dev)
{
	ARG_UNUSED(dev);

	LOG_DBG("dev %p", dev);

	return IPM_ADSP_MAX_DATA_SIZE;
}

static u32_t ipm_adsp_max_id_val_get(struct device *dev)
{
	ARG_UNUSED(dev);

	LOG_DBG("dev %p", dev);

	return IPM_ADSP_MAX_ID_VAL;
}

static int ipm_adsp_set_enabled(struct device *dev, int enable)
{
	LOG_DBG("dev %p", dev);

	return 0;
}

static int ipm_adsp_init(struct device *dev)
{
	const struct ipm_adsp_config *config = dev->config->config_info;

	LOG_DBG("dev %p", dev);

	config->irq_config_func(dev);

	return 0;
}

static const struct ipm_driver_api ipm_adsp_driver_api = {
	.send = ipm_adsp_send,
	.register_callback = ipm_adsp_register_callback,
	.max_data_size_get = ipm_adsp_max_data_size_get,
	.max_id_val_get = ipm_adsp_max_id_val_get,
	.set_enabled = ipm_adsp_set_enabled,
};

static void ipm_adsp_config_func_0(struct device *dev);

static const struct ipm_adsp_config ipm_adsp_0_config = {
	.base = (u8_t *)DT_IPM_ADSP_0_BASE_ADDRESS,
	.irq_config_func = ipm_adsp_config_func_0,
};

static struct ipm_adsp_data ipm_adsp_0_data;

DEVICE_AND_API_INIT(ipm_adsp_0, DT_IPM_ADSP_0_NAME,
		    &ipm_adsp_init,
		    &ipm_adsp_0_data, &ipm_adsp_0_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &ipm_adsp_driver_api);

static void ipm_adsp_config_func_0(struct device *dev)
{
	IRQ_CONNECT(DT_IPM_ADSP_0_IRQ,
		    DT_IPM_ADSP_0_IRQ_PRI,
		    ipm_adsp_isr, DEVICE_GET(ipm_adsp_0), 0);

	irq_enable(DT_IPM_ADSP_0_IRQ);

	/* enable IPC interrupts from host */
	ipc_write(IPC_DIPCCTL, IPC_DIPCCTL_IPCIDIE | IPC_DIPCCTL_IPCTBIE);
}
