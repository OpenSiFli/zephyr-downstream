/*
 * Copyright (c) 2025 Core Devices LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_dmac

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control/sf32lb.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/toolchain.h>

#include <ll_dmac.h>

LOG_MODULE_REGISTER(dma_sf32lb, CONFIG_DMA_LOG_LEVEL);

#define DMAC_MAX_LEN DMAC_CNDTR1_NDT
#define DMAC_MAX_PL  3U

#define DMAC_MAX_CH 8U

struct dma_sf32lb_irq_ctx {
	const struct device *dev;
	uint8_t channel;
};

struct dma_sf32lb_config {
	uintptr_t dmac;
	uint8_t n_channels;
	uint8_t n_requests;
	struct sf32lb_clock_dt_spec clock;
	void (*irq_configure)(void);
	struct dma_sf32lb_channel *channels;
};

struct dma_sf32lb_channel {
	dma_callback_t callback;
	void *user_data;
	uint32_t size;
	enum dma_channel_direction direction;
};

struct dma_sf32lb_data {
	struct dma_context ctx;
	struct k_spinlock lock;
	ATOMIC_DEFINE(status, DMAC_MAX_CH);
};

static inline uint32_t dma_sf32lb_ll_channel(uint32_t channel)
{
	return channel + 1U;
}

static inline ll_dmac_channel_t *dma_sf32lb_get_channel(DMAC_TypeDef *dmac, uint32_t channel)
{
	return ll_dmac_get_channel(dmac, dma_sf32lb_ll_channel(channel));
}

static uint32_t dma_sf32lb_data_size_to_ll(uint32_t size, bool memory)
{
	switch (size) {
	case 1U:
		return memory ? LL_DMAC_MSIZE_8BIT : LL_DMAC_PSIZE_8BIT;
	case 2U:
		return memory ? LL_DMAC_MSIZE_16BIT : LL_DMAC_PSIZE_16BIT;
	case 4U:
		return memory ? LL_DMAC_MSIZE_32BIT : LL_DMAC_PSIZE_32BIT;
	default:
		__ASSERT_NO_MSG(false);
		return 0U;
	}
}

static inline void dma_sf32lb_clear_all_flags(DMAC_TypeDef *dmac, uint32_t channel)
{
	uint32_t ll_channel = dma_sf32lb_ll_channel(channel);

	ll_dmac_clear_flag_tc(dmac, ll_channel);
	ll_dmac_clear_flag_ht(dmac, ll_channel);
	ll_dmac_clear_flag_te(dmac, ll_channel);
	ll_dmac_clear_flag_gi(dmac, ll_channel);
}

static void dma_sf32lb_isr(const struct device *dev, uint8_t channel)
{
	const struct dma_sf32lb_config *config = dev->config;
	struct dma_sf32lb_data *data = dev->data;
	DMAC_TypeDef *dmac = (DMAC_TypeDef *)config->dmac;
	uint32_t ll_channel = dma_sf32lb_ll_channel(channel);
	uint32_t isr;
	uint32_t channel_flags;
	int status;

	isr = ll_dmac_get_isr(dmac);
	channel_flags = (isr >> ll_dmac_channel_flag_shift(ll_channel)) & 0xFU;
	if (ll_dmac_is_active_flag_tc(dmac, ll_channel) != 0U) {
		status = DMA_STATUS_COMPLETE;
		atomic_clear_bit(data->status, channel);
	} else if (ll_dmac_is_active_flag_ht(dmac, ll_channel) != 0U) {
		status = DMA_STATUS_HALF_COMPLETE;
	} else if (channel_flags != 0U) {
		status = -EIO;
		atomic_clear_bit(data->status, channel);
	} else {
		status = -EINPROGRESS;
	}

	if (status != -EINPROGRESS && config->channels[channel].callback != NULL) {
		config->channels[channel].callback(dev, config->channels[channel].user_data,
						   channel, status);
	}

	dma_sf32lb_clear_all_flags(dmac, channel);
}

#define DMA_SF32LB_IRQ_DEFINE(n, _)                                                                \
	static void dma_sf32lb_isr_ch##n(const struct device *dev)                                 \
	{                                                                                          \
		dma_sf32lb_isr(dev, n);                                                            \
	}

LISTIFY(8, DMA_SF32LB_IRQ_DEFINE, ())

static int check_dma_config(uint32_t channel, struct dma_config *config_dma,
			    const struct dma_sf32lb_config *config)
{
	if (channel >= config->n_channels) {
		LOG_ERR("Invalid channel (%" PRIu32 ", max %" PRIu32 ")", channel,
			config->n_channels);
		return -EINVAL;
	}

	if (config_dma->block_count != 1U) {
		LOG_ERR("Chained block transfer not supported (%" PRIu32 ", max 1)",
			config_dma->block_count);
		return -ENOTSUP;
	}

	if (config_dma->head_block->block_size > DMAC_MAX_LEN) {
		LOG_ERR("Block size exceeds maximum (%" PRIu32 ", max %lu)",
			config_dma->head_block->block_size, DMAC_MAX_LEN);
		return -EINVAL;
	}

	if (config_dma->dma_slot >= config->n_requests) {
		LOG_ERR("Invalid DMA slot (%" PRIu32 ", max %" PRIu32 ")", config_dma->dma_slot,
			config->n_requests);
		return -EINVAL;
	}

	if (config_dma->channel_priority > DMAC_MAX_PL) {
		LOG_ERR("Invalid channel priority (%" PRIu32 ", max %" PRIu32 ")",
			config_dma->channel_priority, DMAC_MAX_PL);
		return -EINVAL;
	}

	if ((config_dma->head_block->source_addr_adj == DMA_ADDR_ADJ_DECREMENT) |
	    (config_dma->head_block->dest_addr_adj == DMA_ADDR_ADJ_DECREMENT)) {
		LOG_ERR("Address decrement not supported");
		return -ENOTSUP;
	}

	if ((config_dma->source_data_size != 1U) && (config_dma->source_data_size != 2U) &&
	    (config_dma->source_data_size != 4U)) {
		LOG_ERR("Invalid source data size (%" PRIu32 ", must be 1, 2, or 4)",
			config_dma->source_data_size);
		return -EINVAL;
	}

	if ((config_dma->dest_data_size != 1U) && (config_dma->dest_data_size != 2U) &&
	    (config_dma->dest_data_size != 4U)) {
		LOG_ERR("Invalid destination data size (%" PRIu32 ", must be 1, 2, or 4)",
			config_dma->dest_data_size);
		return -EINVAL;
	}

	if (config_dma->dest_data_size != config_dma->source_data_size) {
		LOG_ERR("Destination and source sizes not equal");
		return -EINVAL;
	}

	return 0;
}

static int dma_sf32lb_config(const struct device *dev, uint32_t channel,
			     struct dma_config *config_dma)
{
	int ret;
	const struct dma_sf32lb_config *config = dev->config;
	struct dma_sf32lb_data *data = dev->data;
	DMAC_TypeDef *dmac = (DMAC_TypeDef *)config->dmac;
	ll_dmac_channel_t *chx = dma_sf32lb_get_channel(dmac, channel);
	ll_dmac_channel_config_t ch_cfg = {
		.mem2mem = LL_DMAC_MEM2MEM_DISABLE,
		.priority = FIELD_PREP(DMAC_CCR1_PL_Msk, config_dma->channel_priority),
		.msize = LL_DMAC_MSIZE_8BIT,
		.psize = LL_DMAC_PSIZE_8BIT,
		.minc = LL_DMAC_MINC_DISABLE,
		.pinc = LL_DMAC_PINC_DISABLE,
		.circ = LL_DMAC_CIRC_DISABLE,
		.dir = LL_DMAC_DIR_PERIPH_TO_MEM,
		.it_tc = LL_DMAC_IT_TC_DISABLE,
		.it_ht = LL_DMAC_IT_HT_DISABLE,
		.it_te = LL_DMAC_IT_TE_DISABLE,
	};
	uint32_t cparx;
	uint32_t cm0arx;

	ret = check_dma_config(channel, config_dma, config);
	if (ret < 0) {
		return ret;
	}

	/* configure transfer parameters */
	if (ll_dmac_is_enabled_channel(chx) != 0U) {
		LOG_ERR("Configuration not possible with DMA enabled");
		return -EIO;
	}

	if (config_dma->head_block->dest_reload_en || config_dma->head_block->source_reload_en) {
		ch_cfg.circ = LL_DMAC_CIRC_ENABLE;
	}

	if (config_dma->half_complete_callback_en) {
		ch_cfg.it_ht = LL_DMAC_IT_HT_ENABLE;
	}

	if (config_dma->dma_callback != NULL) {
		ch_cfg.it_tc = LL_DMAC_IT_TC_ENABLE;
		ch_cfg.it_te = LL_DMAC_IT_TE_ENABLE;
	}

	switch (config_dma->channel_direction) {
	case MEMORY_TO_MEMORY:
		ch_cfg.mem2mem = LL_DMAC_MEM2MEM_ENABLE;
		__fallthrough;
	case PERIPHERAL_TO_MEMORY:
		ch_cfg.psize = dma_sf32lb_data_size_to_ll(config_dma->source_data_size, false);
		ch_cfg.msize = dma_sf32lb_data_size_to_ll(config_dma->dest_data_size, true);

		if (config_dma->head_block->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			ch_cfg.pinc = LL_DMAC_PINC_ENABLE;
		}
		if (config_dma->head_block->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			ch_cfg.minc = LL_DMAC_MINC_ENABLE;
		}

		cparx = config_dma->head_block->source_address;
		cm0arx = config_dma->head_block->dest_address;
		break;
	case MEMORY_TO_PERIPHERAL:
		ch_cfg.dir = LL_DMAC_DIR_MEM_TO_PERIPH;
		ch_cfg.psize = dma_sf32lb_data_size_to_ll(config_dma->dest_data_size, false);
		ch_cfg.msize = dma_sf32lb_data_size_to_ll(config_dma->source_data_size, true);

		if (config_dma->head_block->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			ch_cfg.minc = LL_DMAC_MINC_ENABLE;
		}
		if (config_dma->head_block->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
			ch_cfg.pinc = LL_DMAC_PINC_ENABLE;
		}

		cparx = config_dma->head_block->dest_address;
		cm0arx = config_dma->head_block->source_address;
		break;
	default:
		return -ENOTSUP;
	}

	ll_dmac_config_channel(chx, &ch_cfg);

	/* single transfer */
	ll_dmac_set_burst_size(chx, 0U);

	/* configure transfer size, src/dst addresses */
	ll_dmac_set_ndt(chx, config_dma->head_block->block_size);
	ll_dmac_set_cpar(chx, cparx);
	ll_dmac_set_cm0ar(chx, cm0arx);

	/* configure request */
	K_SPINLOCK(&data->lock) {
		ll_dmac_set_channel_request(dmac, dma_sf32lb_ll_channel(channel),
					    config_dma->dma_slot);
	}

	config->channels[channel].callback = config_dma->dma_callback;
	config->channels[channel].user_data = config_dma->user_data;
	config->channels[channel].direction = config_dma->channel_direction;
	config->channels[channel].size = config_dma->source_data_size;

	return 0;
}

static int dma_sf32lb_reload(const struct device *dev, uint32_t channel, uint32_t src, uint32_t dst,
			     size_t size)
{
	const struct dma_sf32lb_config *config = dev->config;
	DMAC_TypeDef *dmac = (DMAC_TypeDef *)config->dmac;
	ll_dmac_channel_t *chx = dma_sf32lb_get_channel(dmac, channel);
	uint32_t cparx;
	uint32_t cm0arx;

	if (channel >= config->n_channels) {
		LOG_ERR("Invalid channel (%" PRIu32 ", max %" PRIu32 ")", channel,
			config->n_channels);
		return -EINVAL;
	}

	if (size > DMAC_MAX_LEN) {
		LOG_ERR("Block size exceeds maximum (%" PRIu32 ", max %lu)", size, DMAC_MAX_LEN);
		return -EINVAL;
	}

	if (ll_dmac_is_enabled_channel(chx) != 0U) {
		LOG_ERR("Channel %" PRIu32 " is busy", channel);
		return -EBUSY;
	}

	/* configure size, src/dst addresses */
	if (config->channels[channel].size == 4) {
		size >>= 2;
	} else if (config->channels[channel].size == 2) {
		size >>= 1;
	} else {
	}

	ll_dmac_set_ndt(chx, size);

	switch (config->channels[channel].direction) {
	case MEMORY_TO_MEMORY:
	case PERIPHERAL_TO_MEMORY:
		cparx = src;
		cm0arx = dst;
		break;
	case MEMORY_TO_PERIPHERAL:
		cparx = dst;
		cm0arx = src;
		break;
	default:
		__ASSERT_NO_MSG(false);
		return -ENOTSUP;
	}

	ll_dmac_set_cpar(chx, cparx);
	ll_dmac_set_cm0ar(chx, cm0arx);

	return 0;
}

static int dma_sf32lb_start(const struct device *dev, uint32_t channel)
{
	const struct dma_sf32lb_config *config = dev->config;
	struct dma_sf32lb_data *data = dev->data;
	DMAC_TypeDef *dmac = (DMAC_TypeDef *)config->dmac;
	ll_dmac_channel_t *chx = dma_sf32lb_get_channel(dmac, channel);

	if (channel >= config->n_channels) {
		LOG_ERR("Invalid channel (%" PRIu32 ", max %" PRIu32 ")", channel,
			config->n_channels);
		return -EINVAL;
	}

	if (ll_dmac_is_enabled_channel(chx) != 0U) {
		LOG_ERR("start not possible with DMA enabled");
		return -EIO;
	}

	/* clear all transfer flags */
	dma_sf32lb_clear_all_flags(dmac, channel);

	/* LL gap: ll_dmac.h has config-time IRQ bits but no post-config IRQ enable helpers. */
	if (config->channels[channel].callback != NULL) {
		sys_set_bits((mem_addr_t)&chx->CCR, DMAC_CCR1_TCIE | DMAC_CCR1_TEIE);
	}
	ll_dmac_enable_channel(chx);
	atomic_set_bit(data->status, channel);

	return 0;
}

static int dma_sf32lb_stop(const struct device *dev, uint32_t channel)
{
	const struct dma_sf32lb_config *config = dev->config;
	struct dma_sf32lb_data *data = dev->data;
	DMAC_TypeDef *dmac = (DMAC_TypeDef *)config->dmac;
	ll_dmac_channel_t *chx = dma_sf32lb_get_channel(dmac, channel);

	if (channel >= config->n_channels) {
		LOG_ERR("Invalid channel (%" PRIu32 ", max %" PRIu32 ")", channel,
			config->n_channels);
		return -EINVAL;
	}

	/* disable DMA and complete/error IRQs */
	ll_dmac_disable_channel(chx);
	/* LL gap: ll_dmac.h has config-time IRQ bits but no post-config IRQ disable helpers. */
	sys_clear_bits((mem_addr_t)&chx->CCR, DMAC_CCR1_TCIE | DMAC_CCR1_TEIE);

	atomic_clear_bit(data->status, channel);

	return 0;
}

static int dma_sf32lb_get_status(const struct device *dev, uint32_t channel,
				 struct dma_status *stat)
{
	const struct dma_sf32lb_config *config = dev->config;
	struct dma_sf32lb_data *data = dev->data;
	DMAC_TypeDef *dmac = (DMAC_TypeDef *)config->dmac;
	ll_dmac_channel_t *chx = dma_sf32lb_get_channel(dmac, channel);

	if (channel >= config->n_channels) {
		LOG_ERR("Invalid channel (%" PRIu32 ", max %" PRIu32 ")", channel,
			config->n_channels);
		return -EINVAL;
	}

	stat->dir = config->channels[channel].direction;
	/* LL gap: ll_dmac.h exposes NDT writes but not a pending-count read helper. */
	stat->pending_length = sys_read32((mem_addr_t)&chx->CNDTR) & DMAC_CNDTR1_NDT;
	stat->busy = atomic_test_bit(data->status, channel) && (stat->pending_length != 0U);

	return 0;
}

static DEVICE_API(dma, dma_sf32lb_driver_api) = {
	.config = dma_sf32lb_config,
	.reload = dma_sf32lb_reload,
	.start = dma_sf32lb_start,
	.stop = dma_sf32lb_stop,
	.get_status = dma_sf32lb_get_status,
};

static int dma_sf32lb_init(const struct device *dev)
{
	const struct dma_sf32lb_config *config = dev->config;

	if (!sf32lb_clock_is_ready_dt(&config->clock)) {
		return -ENODEV;
	}

	(void)sf32lb_clock_control_on_dt(&config->clock);

	for (uint8_t channel = 0U; channel < config->n_channels; channel++) {
		DMAC_TypeDef *dmac = (DMAC_TypeDef *)config->dmac;
		ll_dmac_channel_t *chx = dma_sf32lb_get_channel(dmac, channel);

		ll_dmac_disable_channel(chx);
		/* LL gap: ll_dmac.h has config-time IRQ bits but no post-config IRQ disable helpers. */
		sys_clear_bits((mem_addr_t)&chx->CCR,
			       DMAC_CCR1_TCIE | DMAC_CCR1_HTIE | DMAC_CCR1_TEIE);
	}

	config->irq_configure();

	return 0;
}

#define DMA_SF32LB_IRQ_CONFIGURE(n, inst)                                                          \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(inst, n, irq), DT_INST_IRQ_BY_IDX(inst, n, priority),       \
		    dma_sf32lb_isr_ch##n, DEVICE_DT_INST_GET(inst), 0);                            \
	irq_enable(DT_INST_IRQ_BY_IDX(inst, n, irq));

#define DMA_SF32LB_CONFIGURE_ALL_IRQS(inst, n) LISTIFY(n, DMA_SF32LB_IRQ_CONFIGURE, (), inst)

#define DMA_SF32LB_DEFINE(inst)                                                                    \
	static void irq_configure##inst(void)                                                      \
	{                                                                                          \
		DMA_SF32LB_CONFIGURE_ALL_IRQS(inst, DT_INST_NUM_IRQS(inst));                       \
	}                                                                                          \
                                                                                                   \
	static struct dma_sf32lb_channel channels##inst[DT_INST_PROP(inst, dma_channels)];         \
                                                                                                   \
	static const struct dma_sf32lb_config config##inst = {                                     \
		.dmac = DT_INST_REG_ADDR(inst),                                                    \
		.n_channels = DT_INST_PROP(inst, dma_channels),                                    \
		.n_requests = DT_INST_PROP(inst, dma_requests),                                    \
		.clock = SF32LB_CLOCK_DT_INST_SPEC_GET(inst),                                      \
		.irq_configure = irq_configure##inst,                                              \
		.channels = channels##inst,                                                        \
	};                                                                                         \
                                                                                                   \
	ATOMIC_DEFINE(atomic##inst, DT_INST_PROP(inst, dma_channels));                             \
                                                                                                   \
	static struct dma_sf32lb_data data##inst = {                                               \
		.ctx =                                                                             \
			{                                                                          \
				.magic = DMA_MAGIC,                                                \
				.atomic = atomic##inst,                                            \
				.dma_channels = DT_INST_PROP(inst, dma_channels),                  \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, dma_sf32lb_init, NULL, &data##inst, &config##inst,             \
			      PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY, &dma_sf32lb_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DMA_SF32LB_DEFINE)
