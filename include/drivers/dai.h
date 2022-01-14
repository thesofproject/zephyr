/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * TODO:
 *
 * 1) Extend to support DMIC, HDA and ALH types from SOF (check the IPC headers
 *    from AMD, NXP, Mediatek too for any quirks/types that need to be added).
 * 2) Support early/late BCLKs for SSPs.
 * 3) Dont break backwards compat for older Zephyr code using i2s.h to transition
 *    to this DAI API.
 * 4) Fix comment to remove I2S refs.
 * 5) Add any aliases for I2S/SSP format modes to ensure legacy Ze[hyr and SOF
 *    compat.
 * 6) make the mem_slab mode use by i2s.h as optional ops.
 * 7) Add any ops/data to help with integration with DMA.
 */

/**
 * @file
 * @brief Public APIs for the DAI (Digital Audio Interface) bus drivers.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DAI_H_
#define ZEPHYR_INCLUDE_DRIVERS_DAI_H_

/**
 * @defgroup dai_interface DAI Interface
 * @ingroup io_interfaces
 * @brief DAI (Inter-IC Sound) Interface
 *
 * The DAI API provides support for the standard DAI interface standard as well
 * as common non-standard extensions such as PCM Short/Long Frame Sync,
 * Left/Right Justified Data Format.
 * @{
 */

#include <zephyr/types.h>
#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The following #defines are used to configure the DAI controller.
 *
 * TODO converge with IPC headers.
 */


typedef uint8_t dai_fmt_t;

/** Data Format bit field position. */
#define DAI_FMT_DATA_FORMAT_SHIFT           0
/** Data Format bit field mask. */
#define DAI_FMT_DATA_FORMAT_MASK            (0x7 << DAI_FMT_DATA_FORMAT_SHIFT)


/** @brief Standard DAI Data Format.
 *
 * Serial data is transmitted in two's complement with the MSB first. Both
 * Word Select (WS) and Serial Data (SD) signals are sampled on the rising edge
 * of the clock signal (SCK). The MSB is always sent one clock period after the
 * WS changes. Left channel data are sent first indicated by WS = 0, followed
 * by right channel data indicated by WS = 1.
 *
 *        -. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-.
 *     SCK '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '
 *        -.                               .-------------------------------.
 *     WS  '-------------------------------'                               '----
 *        -.---.---.---.---.---.---.---.---.---.---.---.---.---.---.---.---.---.
 *     SD  |   |MSB|   |...|   |LSB| x |...| x |MSB|   |...|   |LSB| x |...| x |
 *        -'---'---'---'---'---'---'---'---'---'---'---'---'---'---'---'---'---'
 *             | Left channel                  | Right channel                 |
 */
#define DAI_FMT_DATA_FORMAT_I2S             (0 << DAI_FMT_DATA_FORMAT_SHIFT)

/** @brief PCM Short Frame Sync Data Format.
 *
 * Serial data is transmitted in two's complement with the MSB first. Both
 * Word Select (WS) and Serial Data (SD) signals are sampled on the falling edge
 * of the clock signal (SCK). The falling edge of the frame sync signal (WS)
 * indicates the start of the PCM word. The frame sync is one clock cycle long.
 * An arbitrary number of data words can be sent in one frame.
 *
 *          .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-.
 *     SCK -' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-
 *          .---.                                                       .---.
 *     WS  -'   '-                                                     -'   '-
 *         -.---.---.---.---.---.---.---.---.---.---.---.---.---.---.---.---.---
 *     SD   |   |MSB|   |...|   |LSB|MSB|   |...|   |LSB|MSB|   |...|   |LSB|
 *         -'---'---'---'---'---'---'---'---'---'---'---'---'---'---'---'---'---
 *              | Word 1            | Word 2            | Word 3  |  Word n |
 */
#define DAI_FMT_DATA_FORMAT_PCM_SHORT       (1 << DAI_FMT_DATA_FORMAT_SHIFT)

/* Alias for interface compatibility with SOF and Linux */
#define DAI_FMT_DATA_FORMAT_PCM_A		DAI_FMT_DATA_FORMAT_PCM_SHORT

/** @brief PCM Long Frame Sync Data Format.
 *
 * Serial data is transmitted in two's complement with the MSB first. Both
 * Word Select (WS) and Serial Data (SD) signals are sampled on the falling edge
 * of the clock signal (SCK). The rising edge of the frame sync signal (WS)
 * indicates the start of the PCM word. The frame sync has an arbitrary length,
 * however it has to fall before the start of the next frame. An arbitrary
 * number of data words can be sent in one frame.
 *
 *          .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-.
 *     SCK -' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-
 *              .--- ---.    ---.        ---.                               .---
 *     WS      -'       '-      '-          '-                             -'
 *         -.---.---.---.---.---.---.---.---.---.---.---.---.---.---.---.---.---
 *     SD   |   |MSB|   |...|   |LSB|MSB|   |...|   |LSB|MSB|   |...|   |LSB|
 *         -'---'---'---'---'---'---'---'---'---'---'---'---'---'---'---'---'---
 *              | Word 1            | Word 2            | Word 3  |  Word n |
 */
#define DAI_FMT_DATA_FORMAT_PCM_LONG        (2 << DAI_FMT_DATA_FORMAT_SHIFT)

/* for interface compatibility with SOF and Linux */
#define DAI_FMT_DATA_FORMAT_PCM_B		DAI_FMT_DATA_FORMAT_PCM_LONG

/**
 * @brief Left Justified Data Format.
 *
 * Serial data is transmitted in two's complement with the MSB first. Both
 * Word Select (WS) and Serial Data (SD) signals are sampled on the rising edge
 * of the clock signal (SCK). The bits within the data word are left justified
 * such that the MSB is always sent in the clock period following the WS
 * transition. Left channel data are sent first indicated by WS = 1, followed
 * by right channel data indicated by WS = 0.
 *
 *          .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-.
 *     SCK -' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-
 *            .-------------------------------.                               .-
 *     WS  ---'                               '-------------------------------'
 *         ---.---.---.---.---.---.---.---.---.---.---.---.---.---.---.---.---.-
 *     SD     |MSB|   |...|   |LSB| x |...| x |MSB|   |...|   |LSB| x |...| x |
 *         ---'---'---'---'---'---'---'---'---'---'---'---'---'---'---'---'---'-
 *            | Left channel                  | Right channel                 |
 */
#define DAI_FMT_DATA_FORMAT_I2S_LEFT_JUSTIFIED  (3 << DAI_FMT_DATA_FORMAT_SHIFT)

/**
 * @brief Right Justified Data Format.
 *
 * Serial data is transmitted in two's complement with the MSB first. Both
 * Word Select (WS) and Serial Data (SD) signals are sampled on the rising edge
 * of the clock signal (SCK). The bits within the data word are right justified
 * such that the LSB is always sent in the clock period preceding the WS
 * transition. Left channel data are sent first indicated by WS = 1, followed
 * by right channel data indicated by WS = 0.
 *
 *          .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-. .-.
 *     SCK -' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-' '-
 *            .-------------------------------.                               .-
 *     WS  ---'                               '-------------------------------'
 *         ---.---.---.---.---.---.---.---.---.---.---.---.---.---.---.---.---.-
 *     SD     | x |...| x |MSB|   |...|   |LSB| x |...| x |MSB|   |...|   |LSB|
 *         ---'---'---'---'---'---'---'---'---'---'---'---'---'---'---'---'---'-
 *            | Left channel                  | Right channel                 |
 */
#define DAI_FMT_DATA_FORMAT_I2S_RIGHT_JUSTIFIED (4 << DAI_FMT_DATA_FORMAT_SHIFT)

/**
 * @brief Pulse density modulation.
 *
 * Serial interface standard used by DMICs and some codecs.
 */
#define DAI_FMT_DATA_FORMAT_PDM			(5 << DAI_FMT_DATA_FORMAT_SHIFT)

/** Send MSB first */
#define DAI_FMT_DATA_ORDER_MSB              (0 << 3)
/** Send LSB first */
#define DAI_FMT_DATA_ORDER_LSB              BIT(3)
/** Invert bit ordering, send LSB first */
#define DAI_FMT_DATA_ORDER_INV              DAI_FMT_DATA_ORDER_LSB

/** Data Format bit field position. */
#define DAI_FMT_CLK_FORMAT_SHIFT           4
/** Data Format bit field mask. */
#define DAI_FMT_CLK_FORMAT_MASK            (0x3 << DAI_FMT_CLK_FORMAT_SHIFT)

/** Invert bit clock */
#define DAI_FMT_BIT_CLK_INV                 BIT(4)
/** Invert frame clock */
#define DAI_FMT_FRAME_CLK_INV               BIT(5)

/** NF represents "Normal Frame" whereas IF represents "Inverted Frame"
 *  NB represents "Normal Bit Clk" whereas IB represents "Inverted Bit clk"
 */
#define DAI_FMT_CLK_NF_NB		(0 << DAI_FMT_CLK_FORMAT_SHIFT)
#define DAI_FMT_CLK_NF_IB		(1 << DAI_FMT_CLK_FORMAT_SHIFT)
#define DAI_FMT_CLK_IF_NB		(2 << DAI_FMT_CLK_FORMAT_SHIFT)
#define DAI_FMT_CLK_IF_IB		(3 << DAI_FMT_CLK_FORMAT_SHIFT)

typedef uint8_t dai_opt_t;

/** Run bit clock continuously */
#define DAI_OPT_BIT_CLK_CONT                (0 << 0)
/** Run bit clock when sending data only */
#define DAI_OPT_BIT_CLK_GATED               BIT(0)

/** DAI driver is bit clock provider (codec is consumer) */
#define DAI_OPT_BIT_CLK_PROVIDER              (0 << 1)
/** DAI driver is bit clock consumer (codec is provider) */
#define DAI_OPT_BIT_CLK_CONSUMER               BIT(1)
/** DAI driver is frame clock provider (codec is consumer) */
#define DAI_OPT_FRAME_CLK_PROVIDER            (0 << 2)
/** DAI driver is frame clock consumer (codec is provider) */
#define DAI_OPT_FRAME_CLK_CONSUMER             BIT(2)

/** @brief Loop back mode.
 *
 * In loop back mode RX input will be connected internally to TX output.
 * This is used primarily for testing.
 */
#define DAI_OPT_LOOPBACK                    BIT(7)

/** @brief Ping pong mode
 *
 * Inherited from legacy TBD to check.
 *
 * In ping pong mode TX output will keep alternating between a ping buffer
 * and a pong buffer. This is normally used in audio streams when one buffer
 * is being populated while the other is being played (DMAed) and vice versa.
 * So, in this mode, 2 sets of buffers fixed in size are used. Static Arrays
 * are used to achieve this and hence they are never freed.
 */
#define DAI_OPT_PINGPONG                    BIT(6)

/** \brief Types of DAI
 *
 * The type of the DAI. This ID type is used to configure bespoke DAI HW
 * settings.
 *
 * DAIs have a lot of physical link feature variability and therefore need
 * different configuration data to cater for different use cases. We
 * sometimes may need to pass extra configuration data prior to DAI start.
 */
enum dai_type {
	DAI_LEGACY_I2S = 0,	/**< Legacy I2S compatible with i2s.h */
	DAI_INTEL_SSP,		/**< Intel SSP */
	DAI_INTEL_DMIC,		/**< Intel DMIC */
	DAI_INTEL_HDA,		/**< Intel HD/A */
	DAI_INTEL_ALH,		/**< Intel ALH */
	DAI_IMX_SAI,                /**< i.MX SAI */
	DAI_IMX_ESAI,               /**< i.MX ESAI */
	DAI_AMD_BT,			/**< Amd BT */
	DAI_AMD_SP,			/**< Amd SP */
	DAI_AMD_DMIC,		/**< Amd DMIC */
	DAI_MEDIATEK_AFE,            /**< Mtk AFE */
	DAI_INTEL_SSP_NHLT,            /**< nhlt ssp */
	DAI_INTEL_DMIC_NHLT,            /**< nhlt ssp */
	DAI_INTEL_HDA_NHLT,		/**< nhlt Intel HD/A */
	DAI_INTEL_ALH_NHLT,		/**< nhlt Intel ALH */
};

/**
 * @brief I2C Direction
 */
enum dai_dir {
	/** Receive data */
	DAI_DIR_RX = 0,
	/** Transmit data */
	DAI_DIR_TX,
	/** Both receive and transmit data */
	DAI_DIR_BOTH,
};

/** Interface state */
enum dai_state {
	/** @brief The interface is not ready.
	 *
	 * The interface was initialized but is not yet ready to receive /
	 * transmit data. Call dai_configure() to configure interface and change
	 * its state to READY.
	 */
	DAI_STATE_NOT_READY = 0,
	/** The interface is ready to receive / transmit data. */
	DAI_STATE_READY,
	/** The interface is receiving / transmitting data. */
	DAI_STATE_RUNNING,
	/** The interface is clocking but not receiving / transmitting data. */
	DAI_STATE_PRE_RUNNING,
	/** The interface is draining its transmit queue. */
	DAI_STATE_STOPPING,
	/** TX buffer underrun or RX buffer overrun has occurred. */
	DAI_STATE_ERROR,
	/** TBD not used today, here for completeness */
	DAI_STATE_DRAINING,
};

/** Trigger command */
enum dai_trigger_cmd {
	/** @brief Start the transmission / reception of data.
	 *
	 * If DAI_DIR_TX is set some data has to be queued for transmission by
	 * the dai_write() function. This trigger can be used in READY state
	 * only and changes the interface state to RUNNING.
	 */
	DAI_TRIGGER_START = 0,
	/** @brief Optional - Pre Start the transmission / reception of data.
	 *
	 * Allows the DAI and downstream codecs to prepare for audio Tx/Rx by
	 * starting any required clocks for downstream PLL/FLL locking.
	 */
	DAI_TRIGGER_PRE_START,
	/** @brief Stop the transmission / reception of data.
	 *
	 * Stop the transmission / reception of data at the end of the current
	 * memory block. This trigger can be used in RUNNING state only and at
	 * first changes the interface state to STOPPING. When the current TX /
	 * RX block is transmitted / received the state is changed to READY.
	 * Subsequent START trigger will resume transmission / reception where
	 * it stopped.
	 */
	DAI_TRIGGER_STOP,
	/** @brief Optional - Post Stop the transmission / reception of data.
	 *
	 * Allows the DAI and downstream codecs to shutdown cleanly after audio
	 * Tx/Rx by stopping any required clocks for downstream audio completion.
	 */
	DAI_TRIGGER_POST_STOP,
	/** @brief Empty the transmit queue.
	 *
	 * Send all data in the transmit queue and stop the transmission.
	 * If the trigger is applied to the RX queue it has the same effect as
	 * DAI_TRIGGER_STOP. This trigger can be used in RUNNING state only and
	 * at first changes the interface state to STOPPING. When all TX blocks
	 * are transmitted the state is changed to READY.
	 */
	DAI_TRIGGER_DRAIN,
	/** @brief Discard the transmit / receive queue.
	 *
	 * Stop the transmission / reception immediately and discard the
	 * contents of the respective queue. This trigger can be used in any
	 * state other than NOT_READY and changes the interface state to READY.
	 */
	DAI_TRIGGER_DROP,
	/** @brief Prepare the queues after underrun/overrun error has occurred.
	 *
	 * This trigger can be used in ERROR state only and changes the
	 * interface state to READY.
	 */
	DAI_TRIGGER_PREPARE,
	/** @brief Reset
	 *
	 * This trigger frees resources.
	 */
	DAI_TRIGGER_RESET,
};

/** DAi properties used for binding with dma and other things */
enum dai_property {
	/* fifo id */
	DAI_PROPERTY_FIFO = 0,
	/* handshake id */
	DAI_PROPERTY_HANDSHAKE,
	DAI_PROPERTY_INIT_DELAY,
};

/** @struct dai_legacy_params
 * @brief Interface configuration options.
 *
 * Copied exactly from existing i2s interface to provide integration path for 
 * legacy zephyr drivers.
 *
 * Memory slab pointed to by the mem_slab field has to be defined and
 * initialized by the user. For DAI driver to function correctly number of
 * memory blocks in a slab has to be at least 2 per queue. Size of the memory
 * block should be multiple of frame_size where frame_size = (channels *
 * word_size_bytes). As an example 16 bit word will occupy 2 bytes, 24 or 32
 * bit word will occupy 4 bytes.
 *
 * Please check Zephyr Kernel Primer for more information on memory slabs.
 *
 * @remark When DAI data format is selected parameter channels is ignored,
 * number of words in a frame is always 2.
 *
 * @param word_size Number of bits representing one data word.
 * @param channels Number of words per frame.
 * @param format Data stream format as defined by DAI_FMT_* constants.
 * @param options Configuration options as defined by DAI_OPT_* constants.
 * @param frame_clk_freq Frame clock (WS) frequency, this is sampling rate.
 * @param mem_slab memory slab to store RX/TX data.
 * @param block_size Size of one RX/TX memory block (buffer) in bytes.
 * @param timeout Read/Write timeout. Number of milliseconds to wait in case TX
 *        queue is full or RX queue is empty, or 0, or SYS_FOREVER_MS.
 */
struct dai_legacy_params {
	uint8_t word_size;
	uint8_t channels;
	dai_fmt_t format;
	dai_opt_t options;
	uint32_t frame_clk_freq;
	struct k_mem_slab *mem_slab;
	size_t block_size;
	int32_t timeout;
};

/** Main dai config struct */
struct dai_config {
	enum dai_type type;
	uint32_t dai_index;

	/* generic dai config members */
	dai_fmt_t format;
	uint32_t channels;
	uint32_t frame_clk_freq;
	uint8_t group_id;
	uint8_t flags;

	/* optional dai specific params */
	const void *params;
};

/* TODO 1) array of clocks 2) wallclk naming to be clear */
struct dai_timestamp_cfg {
	uint32_t walclk_rate; /* Rate in Hz, e.g. 19200000 */
	int type; /* SSP, DMIC, HDA, etc. */
	int direction; /* Playback, capture */
	int index; /* For SSPx to select correct timestamp register */
	int dma_id; /* DMA instance id */
	int dma_chan_index; /* Used DMA channel */
	int dma_chan_count; /* Channels in single DMA */
};

struct dai_timestamp_data {
	uint64_t walclk; /* Wall clock */
	uint64_t sample; /* Sample count */
	uint32_t walclk_rate; /* Rate in Hz, e.g. 19200000 */
};

/**
 * @cond INTERNAL_HIDDEN
 *
 * For internal use only, skip these in public documentation.
 */
__subsystem struct dai_driver_api {
	int (*init)(const struct device *dev);
	int (*deinit)(const struct device *dev);

	int (*config_set)(const struct device *dev, const struct dai_config *cfg,
			  const void * bespoke_cfg);
	const struct dai_config *(*config_get)(const struct device *dev,
					       enum dai_dir dir);

	uint32_t (*get_property)(const struct device *dev, enum dai_dir dir, int stream_id,
				 enum dai_property prop);

	int (*trigger)(const struct device *dev, enum dai_dir dir, int stream_id,
		       enum dai_trigger_cmd cmd);

	/* simple mem_block interface */
	int (*read)(const struct device *dev, void **mem_block, size_t *size, int stream_id);
	int (*write)(const struct device *dev, void *mem_block, size_t size, int stream_id);

	/* timestamp ops */
	int (*timestamps_config)(const struct device *dev, struct dai_timestamp_cfg *cfg);
	int (*timestamp_start)(const struct device *dev, struct dai_timestamp_cfg *cfg);
	int (*timestamp_stop)(const struct device *dev, struct dai_timestamp_cfg *cfg);
	int (*timestamp_get)(const struct device *dev, struct dai_timestamp_cfg *cfg,
			     struct dai_timestamp_data *tsd);
};
/**
 * @endcond
 */

/**
 * @brief Configure operation of a host DAI controller.
 *
 * The dir parameter specifies if Transmit (TX) or Receive (RX) direction
 * will be configured by data provided via cfg parameter.
 *
 * The function can be called in NOT_READY or READY state only. If executed
 * successfully the function will change the interface state to READY.
 *
 * If the function is called with the parameter cfg->frame_clk_freq set to 0
 * the interface state will be changed to NOT_READY.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param dir Stream direction: RX, TX, or both, as defined by DAI_DIR_*.
 *            The DAI_DIR_BOTH value may not be supported by some drivers.
 *            For those, the RX and TX streams need to be configured separately.
 * @param cfg Pointer to the structure containing configuration parameters.
 *
 * @retval 0 If successful.
 * @retval -EINVAL Invalid argument.
 * @retval -ENOSYS DAI_DIR_BOTH value is not supported.
 */
__syscall int dai_z_config_set(const struct device *dev,
			      const struct dai_config *cfg,
			      const void *bespoke_cfg);

static inline int z_impl_dai_z_config_set(const struct device *dev,
					  const struct dai_config *cfg,
					  const void *bespoke_cfg)
{
	const struct dai_driver_api *api =
		(const struct dai_driver_api *)dev->api;

	return api->config_set(dev, cfg, bespoke_cfg);
}

/**
 * @brief Fetch configuration information of a host DAI controller
 *
 * @param dev Pointer to the device structure for the driver instance
 * @param dir Stream direction: RX or TX as defined by DAI_DIR_*
 * @retval Pointer to the structure containing configuration parameters,
 *         or NULL if un-configured
 */
static inline const struct dai_config * dai_z_config_get(const struct device *dev,
							 enum dai_dir dir)
{
	const struct dai_driver_api *api =
		(const struct dai_driver_api *)dev->api;

	return api->config_get(dev, dir);
}

/**
 * @brief Read data from the RX queue.
 *
 * Data received by the DAI interface is stored in the RX queue consisting of
 * memory blocks preallocated by this function from rx_mem_slab (as defined by
 * dai_configure). Ownership of the RX memory block is passed on to the user
 * application which has to release it.
 *
 * The data is read in chunks equal to the size of the memory block. If the
 * interface is in READY state the number of bytes read can be smaller.
 *
 * If there is no data in the RX queue the function will block waiting for
 * the next RX memory block to fill in. This operation can timeout as defined
 * by dai_configure. If the timeout value is set to K_NO_WAIT the function
 * is non-blocking.
 *
 * Reading from the RX queue is possible in any state other than NOT_READY.
 * If the interface is in the ERROR state it is still possible to read all the
 * valid data stored in RX queue. Afterwards the function will return -EIO
 * error.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param mem_block Pointer to the RX memory block containing received data.
 * @param size Pointer to the variable storing the number of bytes read.
 *
 * @retval 0 If successful.
 * @retval -EIO The interface is in NOT_READY or ERROR state and there are no
 *         more data blocks in the RX queue.
 * @retval -EBUSY Returned without waiting.
 * @retval -EAGAIN Waiting period timed out.
 */
static inline int dai_z_read(const struct device *dev, void **mem_block,
			     size_t *size, int stream_id)
{
	const struct dai_driver_api *api =
		(const struct dai_driver_api *)dev->api;

	return api->read(dev, mem_block, size, stream_id);
}

/**
 * @brief Read data from the RX queue into a provided buffer
 *
 * Data received by the DAI interface is stored in the RX queue consisting of
 * memory blocks preallocated by this function from rx_mem_slab (as defined by
 * dai_configure). Calling this function removes one block from the queue
 * which is copied into the provided buffer and then freed.
 *
 * The provided buffer must be large enough to contain a full memory block
 * of data, which is parameterized for the channel via dai_configure().
 *
 * This function is otherwise equivalent to dai_read().
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param buf Destination buffer for read data, which must be at least the
 *            as large as the configured memory block size for the RX channel.
 * @param size Pointer to the variable storing the number of bytes read.
 *
 * @retval 0 If successful.
 * @retval -EIO The interface is in NOT_READY or ERROR state and there are no
 *         more data blocks in the RX queue.
 * @retval -EBUSY Returned without waiting.
 * @retval -EAGAIN Waiting period timed out.
 */
__syscall int dai_z_buf_read(const struct device *dev, void *buf, size_t *size,
			     int stream_id);

/**
 * @brief Write data to the TX queue.
 *
 * Data to be sent by the DAI interface is stored first in the TX queue. TX
 * queue consists of memory blocks preallocated by the user from tx_mem_slab
 * (as defined by dai_configure). This function takes ownership of the memory
 * block and will release it when all data are transmitted.
 *
 * If there are no free slots in the TX queue the function will block waiting
 * for the next TX memory block to be send and removed from the queue. This
 * operation can timeout as defined by dai_configure. If the timeout value is
 * set to K_NO_WAIT the function is non-blocking.
 *
 * Writing to the TX queue is only possible if the interface is in READY or
 * RUNNING state.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param mem_block Pointer to the TX memory block containing data to be sent.
 * @param size Number of bytes to write. This value has to be equal or smaller
 *        than the size of the memory block.
 *
 * @retval 0 If successful.
 * @retval -EIO The interface is not in READY or RUNNING state.
 * @retval -EBUSY Returned without waiting.
 * @retval -EAGAIN Waiting period timed out.
 */
static inline int dai_z_write(const struct device *dev, void *mem_block,
			      size_t size, int stream_id)
{
	const struct dai_driver_api *api =
		(const struct dai_driver_api *)dev->api;

	return api->write(dev, mem_block, size, stream_id);
}

/**
 * @brief Write data to the TX queue from a provided buffer
 *
 * This function acquires a memory block from the DAI channel TX queue
 * and copies the provided data buffer into it. It is otherwise equivalent
 * to dai_write().
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param buf Pointer to a buffer containing the data to transmit.
 * @param size Number of bytes to write. This value has to be equal or smaller
 *        than the size of the channel's TX memory block configuration.
 *
 * @retval 0 If successful.
 * @retval -EIO The interface is not in READY or RUNNING state.
 * @retval -EBUSY Returned without waiting.
 * @retval -EAGAIN Waiting period timed out.
 * @retval -ENOMEM No memory in TX slab queue.
 * @retval -EINVAL Size parameter larger than TX queue memory block.
 */
__syscall int dai_z_buf_write(const struct device *dev, void *buf, size_t size,
			      int stream_id);

/**
 * @brief Send a trigger command.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param dir Stream direction: RX, TX, or both, as defined by DAI_DIR_*.
 *            The DAI_DIR_BOTH value may not be supported by some drivers.
 *            For those, triggering need to be done separately for the RX
 *            and TX streams.
 * @param cmd Trigger command.
 *
 * @retval 0 If successful.
 * @retval -EINVAL Invalid argument.
 * @retval -EIO The trigger cannot be executed in the current state or a DMA
 *         channel cannot be allocated.
 * @retval -ENOMEM RX/TX memory block not available.
 * @retval -ENOSYS DAI_DIR_BOTH value is not supported.
 */
__syscall int dai_z_trigger(const struct device *dev, enum dai_dir dir,
			    int stream_id, enum dai_trigger_cmd cmd);

static inline int z_impl_dai_z_trigger(const struct device *dev,
				       enum dai_dir dir,
				       int stream_id,
				       enum dai_trigger_cmd cmd)
{
	const struct dai_driver_api *api =
		(const struct dai_driver_api *)dev->api;

	return api->trigger(dev, dir, stream_id, cmd);
}

static inline int dai_z_get_property(const struct device *dev, enum dai_dir dir,
				     int stream_id, enum dai_property prop)
{
	const struct dai_driver_api *api =
		(const struct dai_driver_api *)dev->api;

	return api->get_property(dev, dir, stream_id, prop);
}

static inline int dai_z_init(const struct device *dev)
{
	const struct dai_driver_api *api =
		(const struct dai_driver_api *)dev->api;

	return api->init(dev);
}

static inline int dai_z_deinit(const struct device *dev)
{
	const struct dai_driver_api *api =
		(const struct dai_driver_api *)dev->api;

	return api->deinit(dev);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#include <syscalls/dai.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_DAI_H_ */
