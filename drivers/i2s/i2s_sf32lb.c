/*
 * Copyright (c) 2026, SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_i2s

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control/sf32lb.h>
#include <zephyr/drivers/dma/sf32lb.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <ll_i2s.h>

LOG_MODULE_REGISTER(i2s_sf32lb, CONFIG_I2S_LOG_LEVEL);

#define I2S_SF32LB_QUEUE_SIZE 4U
#define I2S_SF32LB_TX_WARMUP_SIZE 1024U
#define I2S_SF32LB_TX_WARMUP_MARKER 0xa5000000U
#define I2S_SF32LB_FIFO_TX_FULL BIT(7)
#define I2S_SF32LB_FIFO_TX_EMPTY BIT(6)
#define I2S_SF32LB_WARMUP_TIMEOUT_US 100000U

struct i2s_sf32lb_queue_entry {
	void *mem_block;
	size_t size;
};

struct i2s_sf32lb_stream {
	struct i2s_config cfg;
	struct k_msgq queue;
	struct i2s_sf32lb_queue_entry queue_buffer[I2S_SF32LB_QUEUE_SIZE];
	struct i2s_sf32lb_queue_entry active;
	enum i2s_state state;
	bool cfg_valid;
	bool drain;
};

struct i2s_sf32lb_config {
	uintptr_t base;
	const struct pinctrl_dev_config *pcfg;
	struct sf32lb_clock_dt_spec clock;
	struct sf32lb_dma_dt_spec tx_dma;
	struct sf32lb_dma_dt_spec rx_dma;
	void (*irq_config_func)(void);
};

struct i2s_sf32lb_data {
	I2S_TypeDef *i2s;
	struct i2s_sf32lb_stream tx;
	struct i2s_sf32lb_stream rx;
	bool rx_dumped;
	uint32_t rx_warmup_buf[I2S_SF32LB_TX_WARMUP_SIZE / sizeof(uint32_t)];
};

/* --------------------------------------------------------------------------
 * Register access helpers (LL gap — direct register R/W)
 * -------------------------------------------------------------------------- */

static inline I2S_TypeDef *i2s_sf32lb_regs(const struct i2s_sf32lb_data *data)
{
	return data->i2s;
}

/* --------------------------------------------------------------------------
 * Controller clock planning
 * --------------------------------------------------------------------------
 * BCLK duty and the LRCK high/low duty sum are direct divisors of the clock
 * gate output. Choose the largest BCLK divisor which leaves enough clocks in
 * each frame for all valid sample bits. This selects the HXT48 path already
 * reported by RCC, sets its DEBUG_LOOP gate divider to 4, and preserves the
 * existing DEBUG_LOOP source-select and loopback bits.
 */
#define I2S_SF32LB_TX_BCLK_DUTY_MAX 0x3fU
#define I2S_SF32LB_LRCK_DUTY_MAX    0xfffU
#define I2S_SF32LB_SP_CLK_DIV_DEFAULT 4U

struct i2s_sf32lb_clock_plan {
	uint32_t gclk_hz;
	uint32_t bclk_hz;
	uint16_t bclk_duty;
	uint16_t lrck_duty_low;
	uint16_t lrck_duty_high;
};

static void i2s_sf32lb_set_sp_clk_div(I2S_TypeDef *i2s, uint8_t divider)
{
	uint32_t debug_loop = READ_REG(i2s->DEBUG_LOOP);

	debug_loop &= ~(I2S_DEBUG_LOOP_SP_CLK_DIV | I2S_DEBUG_LOOP_SP_CLK_DIV_UPDATE);
	debug_loop |= (divider << I2S_DEBUG_LOOP_SP_CLK_DIV_Pos) &
		      I2S_DEBUG_LOOP_SP_CLK_DIV;
	WRITE_REG(i2s->DEBUG_LOOP, debug_loop | I2S_DEBUG_LOOP_SP_CLK_DIV_UPDATE);
}

static int i2s_sf32lb_plan_controller_clock(uint32_t source_clk_hz,
					     const struct i2s_config *cfg,
					     struct i2s_sf32lb_clock_plan *plan)
{
	uint32_t min_frame_bclk_count;
	uint32_t bclk_duty;
	uint32_t gclk_hz;
	uint32_t lrck_period;

	if (source_clk_hz == 0U || cfg->frame_clk_freq == 0U || cfg->channels == 0U ||
	    cfg->word_size == 0U || plan == NULL) {
		return -EINVAL;
	}

	/* This initial planner only supports standard I2S serial timing. */
	if ((cfg->format & I2S_FMT_DATA_FORMAT_MASK) != I2S_FMT_DATA_FORMAT_I2S) {
		return -ENOTSUP;
	}

	if (cfg->channels > UINT32_MAX / cfg->word_size) {
		return -EINVAL;
	}
	min_frame_bclk_count = cfg->channels * cfg->word_size;

	if (source_clk_hz % I2S_SF32LB_SP_CLK_DIV_DEFAULT != 0U) {
		return -ENOTSUP;
	}
	gclk_hz = source_clk_hz / I2S_SF32LB_SP_CLK_DIV_DEFAULT;

	if (gclk_hz % cfg->frame_clk_freq != 0U) {
		return -ENOTSUP;
	}
	lrck_period = gclk_hz / cfg->frame_clk_freq;
	if (lrck_period == 0U || (lrck_period / 2U) > I2S_SF32LB_LRCK_DUTY_MAX) {
		return -ENOTSUP;
	}

	for (bclk_duty = I2S_SF32LB_TX_BCLK_DUTY_MAX; bclk_duty > 0U; bclk_duty--) {
		if (lrck_period % bclk_duty == 0U &&
		    lrck_period / bclk_duty >= min_frame_bclk_count) {
			break;
		}
	}
	if (bclk_duty == 0U) {
		return -ENOTSUP;
	}

	plan->gclk_hz = gclk_hz;
	plan->bclk_hz = gclk_hz / bclk_duty;
	plan->bclk_duty = bclk_duty;
	plan->lrck_duty_low = lrck_period / 2U;
	plan->lrck_duty_high = lrck_period - plan->lrck_duty_low;

	return 0;
}

/* --------------------------------------------------------------------------
 * Timing mode translation: Zephyr i2s_fmt_t → LL I2S timing
 * -------------------------------------------------------------------------- */

static uint32_t i2s_sf32lb_fmt_to_ll_timing(i2s_fmt_t format)
{
	switch (format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		return LL_I2S_TIMING_I2S;
	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
		return LL_I2S_TIMING_LEFT_J;
	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		return LL_I2S_TIMING_RIGHT_J;
	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
	case I2S_FMT_DATA_FORMAT_PCM_LONG:
		return LL_I2S_TIMING_DSP;
	default:
		return LL_I2S_TIMING_I2S;
	}
}

static uint32_t i2s_sf32lb_fmt_to_ll_lrck_pol(i2s_fmt_t format)
{
	uint32_t data_format = format & I2S_FMT_DATA_FORMAT_MASK;

	/* Left-Justified and Right-Justified use inverted LRCK polarity */
	if (data_format == I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED ||
	    data_format == I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED) {
		return LL_I2S_LRCK_POL_INVERT;
	}

	/* Standard I2S and DSP modes use normal polarity */
	return LL_I2S_LRCK_POL_NORMAL;
}

static uint32_t i2s_sf32lb_is_slave(i2s_opt_t options)
{
	if ((options & I2S_OPT_BIT_CLK_TARGET) != 0U ||
	    (options & I2S_OPT_FRAME_CLK_TARGET) != 0U) {
		return LL_I2S_SLAVE;
	}

	return LL_I2S_MASTER;
}

/* --------------------------------------------------------------------------
 * TX path configuration (LL-based)
 * -------------------------------------------------------------------------- */

static void i2s_sf32lb_config_tx(I2S_TypeDef *i2s, const struct i2s_config *cfg,
				 const struct i2s_sf32lb_clock_plan *clock_plan)
{
	ll_i2s_tx_timing_config_t tx_timing = {
		.timing = i2s_sf32lb_fmt_to_ll_timing(cfg->format),
		.role = i2s_sf32lb_is_slave(cfg->options),
		.lrck_pol = i2s_sf32lb_fmt_to_ll_lrck_pol(cfg->format),
	};
	ll_i2s_tx_pcm_format_config_t tx_pcm_fmt = {
		.data_width = cfg->word_size,
		.track_flag = (cfg->channels == 1U) ? I2S_TX_PCM_FORMAT_TRACK_FLAG : 0U,
	};

	/* Serial timing */
	ll_i2s_config_tx_timing(i2s, &tx_timing);

	/* PCM format: data width and mono/stereo */
	ll_i2s_config_tx_pcm_format(i2s, &tx_pcm_fmt);

	/* Audio data width (AUDIO_TX_FORMAT) — same as word_size */
	ll_i2s_set_tx_audio_data_width(i2s, cfg->word_size);

	/* TX PCM sample clock period follows the complete LRCK period. */
	ll_i2s_set_tx_sample_clk(i2s,
				 clock_plan->lrck_duty_low + clock_plan->lrck_duty_high);

	/* BCLK and LRCK dividers */
	ll_i2s_set_tx_bclk_div(i2s, clock_plan->bclk_duty);
	ll_i2s_set_tx_lrck_div(i2s, clock_plan->lrck_duty_low,
			       clock_plan->lrck_duty_high);

	/* Channel select: TX right <- source right; TX left <- source left. */
	ll_i2s_set_tx_ch_sel(i2s, LL_I2S_RIGHT_CH_SEL_RIGHT,
			       LL_I2S_LEFT_CH_SEL_LEFT);

	/* Volume: 0dB (4 = 0dB per HAL convention) */
	ll_i2s_set_tx_volume(i2s, 4U);

	/* Balance: disabled */
	ll_i2s_set_tx_balance(i2s, 0U, LL_I2S_BALANCE_DISABLE);

	/* Re-sample smooth: disabled */
	ll_i2s_set_tx_rs_smooth(i2s, 0U);
}

/* --------------------------------------------------------------------------
 * RX path configuration (LL-based)
 * -------------------------------------------------------------------------- */

static void i2s_sf32lb_config_rx(I2S_TypeDef *i2s, const struct i2s_config *cfg,
				 const struct i2s_sf32lb_clock_plan *clock_plan)
{
	ll_i2s_rx_timing_config_t rx_timing = {
		.timing = i2s_sf32lb_fmt_to_ll_timing(cfg->format),
		.role = i2s_sf32lb_is_slave(cfg->options),
		.lrck_pol = i2s_sf32lb_fmt_to_ll_lrck_pol(cfg->format),
	};

	/* Serial timing */
	ll_i2s_config_rx_timing(i2s, &rx_timing);

	/* RX PCM width is encoded as its literal bit width. */
	ll_i2s_set_rx_pcm_data_width(i2s, cfg->word_size);

	/* RX PCM sample clock period follows the complete LRCK period. */
	ll_i2s_set_rx_rs_clk_div(i2s,
				  clock_plan->lrck_duty_low + clock_plan->lrck_duty_high);

	/* BCLK and LRCK dividers */
	ll_i2s_set_rx_bclk_div(i2s, clock_plan->bclk_duty);
	ll_i2s_set_rx_lrck_div(i2s, clock_plan->lrck_duty_low,
			       clock_plan->lrck_duty_high);

	/* Channel select: record right <- RX right; record left <- RX left. */
	ll_i2s_set_rx_ch_sel(i2s, LL_I2S_RIGHT_CH_SEL_RIGHT,
			       LL_I2S_LEFT_CH_SEL_LEFT);

	/* RECORD_FORMAT supports 8- and 16-bit samples only. */
	ll_i2s_set_record_format(i2s, (cfg->word_size == 16U) ? 1U : 0U,
				    (cfg->channels == 1U) ? 1U : 0U);

	/* Route the I2S serial input pin to the receive FIFO. */
	ll_i2s_select_record_src(i2s, LL_I2S_RECORD_SRC_I2S_AUDIO);

	/* Re-sample: disabled */
	ll_i2s_set_rx_rs_smooth(i2s, 0U);
}

/* --------------------------------------------------------------------------
 * PIO data transfer (polling FIFO flags)
 * -------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------
 * DMA stream engine
 * -------------------------------------------------------------------------- */

static k_timeout_t i2s_sf32lb_timeout(int32_t timeout)
{
	return (timeout == SYS_FOREVER_MS) ? K_FOREVER : K_MSEC(timeout);
}

static void i2s_sf32lb_free_active(struct i2s_sf32lb_stream *stream)
{
	if (stream->active.mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->active.mem_block);
		stream->active.mem_block = NULL;
	}
}

static void i2s_sf32lb_purge_queue(struct i2s_sf32lb_stream *stream)
{
	struct i2s_sf32lb_queue_entry entry;

	while (k_msgq_get(&stream->queue, &entry, K_NO_WAIT) == 0) {
		k_mem_slab_free(stream->cfg.mem_slab, entry.mem_block);
	}
}

static void i2s_sf32lb_stop_stream(const struct i2s_sf32lb_config *config,
				   struct i2s_sf32lb_data *data, bool tx, bool purge_queue)
{
	struct i2s_sf32lb_stream *stream = tx ? &data->tx : &data->rx;
	const struct sf32lb_dma_dt_spec *dma = tx ? &config->tx_dma : &config->rx_dma;

	(void)sf32lb_dma_stop_dt(dma);
	if (tx) {
		ll_i2s_enable_dma_tx(data->i2s);
		ll_i2s_disable_tx(data->i2s);
	} else {
		ll_i2s_enable_dma_rx(data->i2s);
		ll_i2s_disable_rx(data->i2s);
	}
	i2s_sf32lb_free_active(stream);
	if (purge_queue) {
		i2s_sf32lb_purge_queue(stream);
	}
}

static int i2s_sf32lb_start_dma(const struct device *dev, bool tx);

static void i2s_sf32lb_dump_rx_block(const struct i2s_sf32lb_stream *stream)
{
	const uint32_t *words = stream->active.mem_block;
	size_t index;
	size_t word_count = stream->active.size / sizeof(*words);

	for (index = 0U; index < word_count; index++) {
		uint32_t word = words[index];

		LOG_DBG("RX[%02u]=0x%08x low=0x%04x high=0x%04x", index, word,
			(uint16_t)word, (uint16_t)(word >> 16));
	}
}

static void i2s_sf32lb_tx_dma_callback(const struct device *dma_dev, void *arg,
					uint32_t channel, int status)
{
	const struct device *dev = arg;
	const struct i2s_sf32lb_config *config = dev->config;
	struct i2s_sf32lb_data *data = dev->data;
	struct i2s_sf32lb_stream *stream = &data->tx;
	int ret;
	// LOG_DBG("TX DMA callback: status=%d state=%d active=%p queued=%u", status,
	// 	stream->state, stream->active.mem_block, k_msgq_num_used_get(&stream->queue));

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);
	/* SF32LB DMAC leaves its channel enabled after transfer complete. */
	(void)sf32lb_dma_stop_dt(&config->tx_dma);
	i2s_sf32lb_free_active(stream);
	if (status < 0) {
		LOG_ERR("TX DMA failed: %d", status);
		ll_i2s_disable_tx(data->i2s);
		stream->state = I2S_STATE_ERROR;
		return;
	}

	if (stream->state == I2S_STATE_STOPPING &&
	    (!stream->drain || k_msgq_num_used_get(&stream->queue) == 0U)) {
		ll_i2s_enable_dma_tx(data->i2s);
		ll_i2s_disable_tx(data->i2s);
		stream->state = I2S_STATE_READY;
		return;
	}
	if (k_msgq_num_used_get(&stream->queue) == 0U) {
		/* Keep RUNNING so a subsequent write() can arm the next block. */
		ll_i2s_enable_dma_tx(data->i2s);
		return;
	}

	ret = i2s_sf32lb_start_dma(dev, true);
	if (ret < 0) {
		LOG_ERR("Could not start next TX DMA block: %d", ret);
		ll_i2s_disable_tx(data->i2s);
		stream->state = I2S_STATE_ERROR;
	}
}

static void i2s_sf32lb_rx_dma_callback(const struct device *dma_dev, void *arg,
					uint32_t channel, int status)
{
	const struct device *dev = arg;
	const struct i2s_sf32lb_config *config = dev->config;
	struct i2s_sf32lb_data *data = dev->data;
	struct i2s_sf32lb_stream *stream = &data->rx;
	int ret;

	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);
	/* SF32LB DMAC leaves its channel enabled after transfer complete. */
	(void)sf32lb_dma_stop_dt(&config->rx_dma);
	// LOG_DBG("RX DMA callback: status=%d state=%d active=%p queued=%u", status,
	// 	stream->state, stream->active.mem_block, k_msgq_num_used_get(&stream->queue));
	if (status < 0) {
		LOG_ERR("RX DMA failed: %d", status);
		i2s_sf32lb_free_active(stream);
		ll_i2s_disable_rx(data->i2s);
		stream->state = I2S_STATE_ERROR;
		return;
	}
	if (!data->rx_dumped) {
		// data->rx_dumped = true;
		i2s_sf32lb_dump_rx_block(stream);
	}

	ret = k_msgq_put(&stream->queue, &stream->active, K_NO_WAIT);
	if (ret < 0) {
		LOG_ERR("RX completed queue full: %d", ret);
		i2s_sf32lb_free_active(stream);
		ll_i2s_disable_rx(data->i2s);
		stream->state = I2S_STATE_ERROR;
		return;
	}
	stream->active.mem_block = NULL;

	if (stream->state == I2S_STATE_STOPPING) {
		LOG_DBG("RX DMA completion transitions STOPPING to READY");
		ll_i2s_enable_dma_rx(data->i2s);
		ll_i2s_disable_rx(data->i2s);
		stream->state = I2S_STATE_READY;
		return;
	}

	ret = i2s_sf32lb_start_dma(dev, false);
	if (ret < 0) {
		LOG_ERR("Could not start next RX DMA block: %d", ret);
		ll_i2s_disable_rx(data->i2s);
		stream->state = I2S_STATE_ERROR;
	}
}

static int i2s_sf32lb_start_dma(const struct device *dev, bool tx)
{
	const struct i2s_sf32lb_config *config = dev->config;
	struct i2s_sf32lb_data *data = dev->data;
	struct i2s_sf32lb_stream *stream = tx ? &data->tx : &data->rx;
	const struct sf32lb_dma_dt_spec *dma = tx ? &config->tx_dma : &config->rx_dma;
	struct dma_block_config block = {0};
	struct dma_config dma_cfg = {
		.source_data_size = sizeof(uint32_t),
		.dest_data_size = sizeof(uint32_t),
		.block_count = 1U,
		.user_data = (void *)dev,
		.head_block = &block,
	};
	int ret;
	// LOG_DBG("Start %s DMA: state=%d active=%p queued=%u", tx ? "TX" : "RX",
	// 	stream->state, stream->active.mem_block, k_msgq_num_used_get(&stream->queue));

	if (stream->active.mem_block != NULL) {
		return -EBUSY;
	}

	if (tx) {
		ret = k_msgq_get(&stream->queue, &stream->active, K_NO_WAIT);
		if (ret < 0) {
			return ret;
		}
		block.source_address = (uintptr_t)stream->active.mem_block;
		block.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		block.dest_address = (uintptr_t)&data->i2s->TX_DMA_ENTRY;
		block.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
		dma_cfg.dma_callback = i2s_sf32lb_tx_dma_callback;
	} else {
		ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->active.mem_block, K_NO_WAIT);
		if (ret < 0) {
			return -ENOMEM;
		}
		stream->active.size = stream->cfg.block_size;
		block.source_address = (uintptr_t)&data->i2s->RX_DMA_ENTRY;
		block.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		block.dest_address = (uintptr_t)stream->active.mem_block;
		block.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		dma_cfg.dma_callback = i2s_sf32lb_rx_dma_callback;
	}

	/* dma_config programs an initial transfer; dma_reload sets the byte count. */
	block.block_size = sizeof(uint32_t);
	sf32lb_dma_config_init_dt(dma, &dma_cfg);
	ret = sf32lb_dma_config_dt(dma, &dma_cfg);
	if (ret < 0) {
		i2s_sf32lb_free_active(stream);
		return ret;
	}
	ret = sf32lb_dma_reload_dt(dma, block.source_address, block.dest_address,
				   stream->active.size);
	if (ret < 0) {
		i2s_sf32lb_free_active(stream);
		return ret;
	}
	ret = sf32lb_dma_start_dt(dma);
	if (ret < 0) {
		i2s_sf32lb_free_active(stream);
		return ret;
	}

	/* DMA_MASK uses one to mask a request. Clear it only after DMAC is armed. */
	if (tx) {
		ll_i2s_disable_dma_tx(data->i2s);
	} else {
		ll_i2s_disable_dma_rx(data->i2s);
	}
	return 0;
}

/* --------------------------------------------------------------------------
 * Zephyr I2S driver API
 * -------------------------------------------------------------------------- */

static int i2s_sf32lb_configure(const struct device *dev, enum i2s_dir dir,
				const struct i2s_config *cfg)
{
	const struct i2s_sf32lb_config *config = dev->config;
	struct i2s_sf32lb_data *data = dev->data;
	I2S_TypeDef *i2s = data->i2s;
	uint32_t clk_rate;
	struct i2s_sf32lb_clock_plan clock_plan;
	int ret;

	if (cfg == NULL || cfg->frame_clk_freq == 0U) {
		return -EINVAL;
	}

	if (cfg->channels == 0U || cfg->word_size == 0U) {
		return -EINVAL;
	}
	if (cfg->word_size != 8U && cfg->word_size != 16U) {
		return -ENOTSUP;
	}

	/* Validate direction-specific state */
	if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
		if (data->tx.state == I2S_STATE_RUNNING ||
		    data->tx.state == I2S_STATE_STOPPING) {
			return -EBUSY;
		}
	}

	if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
		if (data->rx.state == I2S_STATE_RUNNING ||
		    data->rx.state == I2S_STATE_STOPPING) {
			return -EBUSY;
		}
	}

	/* Get source clock rate */
	ret = sf32lb_clock_control_get_rate_dt(&config->clock, &clk_rate);
	if (ret < 0) {
		return ret;
	}
	LOG_DBG("I2S source clock rate: %u Hz", clk_rate);

	/* Build an exact controller clock plan from HXT48 after the gate divider. */
	LOG_DBG("I2S requested sample rate: %u Hz, channels=%u, word_size=%u",
		cfg->frame_clk_freq, cfg->channels, cfg->word_size);
	ret = i2s_sf32lb_plan_controller_clock(clk_rate, cfg, &clock_plan);
	if (ret < 0) {
		LOG_ERR("Cannot create exact I2S clock plan: fs=%u Hz, gclk=%u Hz, err=%d",
			cfg->frame_clk_freq, clk_rate, ret);
		return ret;
	}
	LOG_DBG("I2S clock plan: gclk=%u Hz, bclk=%u Hz, bclk_duty=%u, lrck=%u/%u",
		clock_plan.gclk_hz, clock_plan.bclk_hz, clock_plan.bclk_duty,
		clock_plan.lrck_duty_low, clock_plan.lrck_duty_high);

	/* Cache configuration */
	if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
		data->tx.cfg = *cfg;
		data->tx.cfg_valid = true;
		data->tx.state = I2S_STATE_READY;
	}

	if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
		data->rx.cfg = *cfg;
		data->rx.cfg_valid = true;
		data->rx.state = I2S_STATE_READY;
	}

	/* Mirror config if only one direction was configured */
	if (!data->tx.cfg_valid && data->rx.cfg_valid) {
		data->tx.cfg = data->rx.cfg;
	}
	if (!data->rx.cfg_valid && data->tx.cfg_valid) {
		data->rx.cfg = data->tx.cfg;
	}

	/* Disable TX/RX before reconfiguration */
	ll_i2s_disable_tx(i2s);
	ll_i2s_disable_rx(i2s);
	i2s_sf32lb_set_sp_clk_div(i2s, I2S_SF32LB_SP_CLK_DIV_DEFAULT);

	/* Configure TX path */
	i2s_sf32lb_config_tx(i2s, &data->tx.cfg, &clock_plan);

	/* RX controller mode is independent of the receive data direction. */
	i2s_sf32lb_config_rx(i2s, &data->rx.cfg, &clock_plan);
	ll_i2s_set_sl_merge(i2s, LL_I2S_SL_MERGE_SHARED);

	/* Keep DMA requests masked until each DMAC channel is armed at START. */
	ll_i2s_enable_dma_tx(i2s);
	ll_i2s_enable_dma_rx(i2s);

	LOG_DBG("I2S configured: fs=%u Hz, word=%u bit, ch=%u, bclk_duty=%u",
		cfg->frame_clk_freq, cfg->word_size, cfg->channels, clock_plan.bclk_duty);

	return 0;
}

static int i2s_sf32lb_write(const struct device *dev, void *mem_block, size_t size)
{
	struct i2s_sf32lb_data *data = dev->data;
	struct i2s_sf32lb_stream *stream = &data->tx;
	struct i2s_sf32lb_queue_entry entry = {
		.mem_block = mem_block,
		.size = size,
	};
	int ret;

	if (mem_block == NULL || size == 0U || !stream->cfg_valid ||
	    size != stream->cfg.block_size || (size % sizeof(uint32_t)) != 0U) {
		return -EINVAL;
	}
	if (stream->state != I2S_STATE_READY && stream->state != I2S_STATE_RUNNING) {
		return -EIO;
	}

	ret = k_msgq_put(&stream->queue, &entry, i2s_sf32lb_timeout(stream->cfg.timeout));
	if (ret < 0) {
		return ret;
	}
	if (stream->state == I2S_STATE_RUNNING && stream->active.mem_block == NULL) {
		ret = i2s_sf32lb_start_dma(dev, true);
		if (ret < 0 && ret != -EBUSY) {
			stream->state = I2S_STATE_ERROR;
			return ret;
		}
	}

	return 0;
}

static int i2s_sf32lb_read(const struct device *dev, void **mem_block, size_t *size)
{
	const struct i2s_sf32lb_config *config = dev->config;
	struct i2s_sf32lb_data *data = dev->data;
	struct i2s_sf32lb_stream *stream = &data->rx;
	struct i2s_sf32lb_queue_entry entry;
	struct dma_status dma_status = {0};
	int ret;

	if (mem_block == NULL || size == NULL || !stream->cfg_valid) {
		return -EINVAL;
	}
	ret = k_msgq_get(&stream->queue, &entry, i2s_sf32lb_timeout(stream->cfg.timeout));
	if (ret < 0) {
		if (sf32lb_dma_get_status_dt(&config->rx_dma, &dma_status) == 0) {
			LOG_ERR("RX read timeout: tx_state=%d rx_state=%d active=%p queued=%u "
				"dma_pending=%u dma_dir=%d mask=0x%08x fifo=0x%08x rx_en=%u",
				data->tx.state,
				stream->state, stream->active.mem_block,
				k_msgq_num_used_get(&stream->queue), dma_status.pending_length,
				dma_status.dir, data->i2s->DMA_MASK, data->i2s->FIFO_STATUS,
				(data->i2s->AUDIO_RX_FUNC_EN & I2S_AUDIO_RX_FUNC_EN_RX_EN) != 0U);
		} else {
			LOG_ERR("RX read timeout: tx_state=%d rx_state=%d active=%p queued=%u",
				data->tx.state, stream->state, stream->active.mem_block,
				k_msgq_num_used_get(&stream->queue));
		}
		return ret;
	}
	*mem_block = entry.mem_block;
	*size = entry.size;

	return 0;
}

static int i2s_sf32lb_warmup(const struct device *dev, uint32_t source_clk_hz);

static int i2s_sf32lb_trigger(const struct device *dev, enum i2s_dir dir,
			      enum i2s_trigger_cmd cmd)
{
	const struct i2s_sf32lb_config *config = dev->config;
	struct i2s_sf32lb_data *data = dev->data;
	struct i2s_sf32lb_stream *stream;
	bool tx;
	int ret;

	if (dir == I2S_DIR_BOTH && cmd == I2S_TRIGGER_START) {
		uint32_t clk_rate;

		if (!data->tx.cfg_valid || !data->rx.cfg_valid ||
		    data->tx.state != I2S_STATE_READY || data->rx.state != I2S_STATE_READY) {
			return -EIO;
		}

		/* Warm up both TX and RX paths; proceed once FIFO is drained. */
		ret = sf32lb_clock_control_get_rate_dt(&config->clock, &clk_rate);
		if (ret < 0) {
			return ret;
		}
		ret = i2s_sf32lb_warmup(dev, clk_rate);
		if (ret < 0) {
			LOG_ERR("I2S warm-up failed: %d", ret);
			return ret;
		}

		data->tx.drain = false;
		data->tx.state = I2S_STATE_RUNNING;
		data->rx.drain = false;
		data->rx.state = I2S_STATE_RUNNING;
		ret = i2s_sf32lb_start_dma(dev, false);
		if (ret < 0) {
			data->tx.state = I2S_STATE_READY;
			data->rx.state = I2S_STATE_READY;
			return ret;
		}
		ll_i2s_enable_rx(data->i2s, LL_I2S_INTF_I2S);
		ret = i2s_sf32lb_start_dma(dev, true);
		if (ret < 0) {
			i2s_sf32lb_stop_stream(config, data, false, true);
			data->tx.state = I2S_STATE_READY;
			data->rx.state = I2S_STATE_READY;
			return ret;
		}
		ll_i2s_enable_tx(data->i2s, LL_I2S_INTF_I2S);
		return 0;
	}

	for (tx = true; tx || dir != I2S_DIR_TX; tx = false) {
		if (!((tx && dir == I2S_DIR_RX) || (!tx && dir == I2S_DIR_TX))) {
			stream = tx ? &data->tx : &data->rx;
			switch (cmd) {
		case I2S_TRIGGER_START:
			if (!stream->cfg_valid || stream->state != I2S_STATE_READY) {
				return -EIO;
			}
			stream->drain = false;
			stream->state = I2S_STATE_RUNNING;
			ret = i2s_sf32lb_start_dma(dev, tx);
			if (ret < 0) {
				stream->state = I2S_STATE_READY;
				return ret;
			}
			if (tx) {
				ll_i2s_enable_tx(data->i2s, LL_I2S_INTF_I2S);
			} else {
				ll_i2s_enable_rx(data->i2s, LL_I2S_INTF_I2S);
			}
			break;
		case I2S_TRIGGER_STOP:
			if (stream->state != I2S_STATE_RUNNING) {
				return -EIO;
			}
			stream->state = I2S_STATE_STOPPING;
			stream->drain = false;
			LOG_DBG("I2S %s STOP: state=STOPPING", tx ? "TX" : "RX");
			if (tx) {
				i2s_sf32lb_purge_queue(stream);
			}
			break;
		case I2S_TRIGGER_DRAIN:
			if (stream->state != I2S_STATE_RUNNING) {
				return -EIO;
			}
			stream->state = I2S_STATE_STOPPING;
			stream->drain = true;
			LOG_DBG("I2S %s DRAIN: state=STOPPING", tx ? "TX" : "RX");
			break;
		case I2S_TRIGGER_DROP:
			if (!stream->cfg_valid || stream->state == I2S_STATE_NOT_READY) {
				return -EIO;
			}
			i2s_sf32lb_stop_stream(config, data, tx, true);
			stream->state = I2S_STATE_READY;
			break;
		case I2S_TRIGGER_PREPARE:
			if (stream->state != I2S_STATE_ERROR) {
				return -EIO;
			}
			i2s_sf32lb_stop_stream(config, data, tx, true);
			stream->state = I2S_STATE_READY;
			break;
			default:
				return -ENOTSUP;
			}
		}
		if (dir != I2S_DIR_BOTH || !tx) {
			break;
		}
	}

	return 0;
}

static const struct i2s_config *i2s_sf32lb_config_get(const struct device *dev,
						      enum i2s_dir dir)
{
	struct i2s_sf32lb_data *data = dev->data;

	if (dir == I2S_DIR_TX) {
		if (!data->tx.cfg_valid) {
			return NULL;
		}
		return &data->tx.cfg;
	}

	if (dir == I2S_DIR_RX) {
		if (!data->rx.cfg_valid) {
			return NULL;
		}
		return &data->rx.cfg;
	}

	return NULL;
}

/* --------------------------------------------------------------------------
 * Interrupt handler (reserved for future interrupt-driven mode)
 * -------------------------------------------------------------------------- */

static void i2s_sf32lb_isr(const struct device *dev)
{
	struct i2s_sf32lb_data *data = dev->data;
	uint32_t status;
	LOG_DBG("I2S ISR: TX state=%d RX state=%d", data->tx.state, data->rx.state);

	status = ll_i2s_get_irq_status(data->i2s);

	if (ll_i2s_is_active_flag_rx_overflow(data->i2s) != 0U) {
		LOG_WRN("RX FIFO overflow");
	}

	if (ll_i2s_is_active_flag_tx_underflow(data->i2s) != 0U) {
		LOG_WRN("TX FIFO underflow");
	}

	ARG_UNUSED(status);
}

/* --------------------------------------------------------------------------
 * Initialization
 * -------------------------------------------------------------------------- */

static int i2s_sf32lb_warmup(const struct device *dev, uint32_t source_clk_hz)
{
	const struct i2s_sf32lb_config *config = dev->config;
	struct i2s_sf32lb_data *data = dev->data;
	const struct i2s_config warmup_cfg = {
		.word_size = 16U,
		.channels = 2U,
		.format = I2S_FMT_DATA_FORMAT_I2S,
		.options = I2S_OPT_FRAME_CLK_CONTROLLER | I2S_OPT_BIT_CLK_CONTROLLER,
		.frame_clk_freq = 8000U,
	};
	struct i2s_sf32lb_clock_plan clock_plan;
	struct dma_block_config rx_block = {0};
	struct dma_config rx_dma_cfg = {
		.source_data_size = sizeof(uint32_t),
		.dest_data_size = sizeof(uint32_t),
		.block_count = 1U,
		.head_block = &rx_block,
		.channel_direction = PERIPHERAL_TO_MEMORY,
	};
	uint32_t fifo_status;
	uint32_t elapsed_us = 0U;
	uint32_t marker;
	int ret;

	ret = i2s_sf32lb_plan_controller_clock(source_clk_hz, &warmup_cfg, &clock_plan);
	if (ret < 0) {
		return ret;
	}
	i2s_sf32lb_set_sp_clk_div(data->i2s, I2S_SF32LB_SP_CLK_DIV_DEFAULT);
	i2s_sf32lb_config_tx(data->i2s, &warmup_cfg, &clock_plan);
	i2s_sf32lb_config_rx(data->i2s, &warmup_cfg, &clock_plan);
	ll_i2s_set_sl_merge(data->i2s, LL_I2S_SL_MERGE_SHARED);

	/* Arm an RX discard DMA block before TX starts generating BCK/LRCK. */
	rx_block.block_size = sizeof(uint32_t);
	rx_block.source_address = (uintptr_t)&data->i2s->RX_DMA_ENTRY;
	rx_block.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	rx_block.dest_address = (uintptr_t)data->rx_warmup_buf;
	rx_block.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	sf32lb_dma_config_init_dt(&config->rx_dma, &rx_dma_cfg);
	ret = sf32lb_dma_config_dt(&config->rx_dma, &rx_dma_cfg);
	if (ret < 0) {
		return ret;
	}
	ret = sf32lb_dma_reload_dt(&config->rx_dma, rx_block.source_address,
				   rx_block.dest_address, sizeof(data->rx_warmup_buf));
	if (ret < 0) {
		return ret;
	}
	ret = sf32lb_dma_start_dt(&config->rx_dma);
	if (ret < 0) {
		return ret;
	}
	ll_i2s_disable_dma_rx(data->i2s);

	/* Now enable both directions so RX ramp-up overlaps TX marker output. */
	*(volatile uint32_t *)0x50009500 = 0x00040104; // 临时切到pll，这个时钟没起振，起到暂停i2s时钟达到同时使能tx和rx的目的。
	ll_i2s_enable_tx(data->i2s, LL_I2S_INTF_I2S);
	ll_i2s_enable_rx(data->i2s, LL_I2S_INTF_I2S);
	*(volatile uint32_t *)0x50009500 = 0x00040100; // 切回hxt48时钟

	for (marker = 0U; marker < I2S_SF32LB_TX_WARMUP_SIZE / sizeof(uint32_t); marker++) {
		while ((ll_i2s_get_fifo_status(data->i2s) & I2S_SF32LB_FIFO_TX_FULL) != 0U) {
			if (elapsed_us++ >= I2S_SF32LB_WARMUP_TIMEOUT_US) {
				ll_i2s_disable_rx(data->i2s);
				ll_i2s_disable_tx(data->i2s);
				return -ETIMEDOUT;
			}
			k_busy_wait(1U);
		}
		data->i2s->TX_DMA_ENTRY = I2S_SF32LB_TX_WARMUP_MARKER | marker;
	}

	while ((ll_i2s_get_fifo_status(data->i2s) & I2S_SF32LB_FIFO_TX_EMPTY) == 0U) {
		if (elapsed_us++ >= I2S_SF32LB_WARMUP_TIMEOUT_US) {
			ll_i2s_disable_rx(data->i2s);
			ll_i2s_disable_tx(data->i2s);
			return -ETIMEDOUT;
		}
		k_busy_wait(1U);
	}

	/* Stop the discard RX DMA and disable both directions. */
	(void)sf32lb_dma_stop_dt(&config->rx_dma);
	// ll_i2s_disable_rx(data->i2s);
	// ll_i2s_disable_tx(data->i2s);

	fifo_status = ll_i2s_get_fifo_status(data->i2s);
	while(!(fifo_status&0x00000004))
	{
		volatile uint32_t* rx_entry = (volatile uint32_t *)0x50009440;
		uint32_t rx_data = *rx_entry;
		fifo_status = ll_i2s_get_fifo_status(data->i2s);
	}
	LOG_DBG("I2S warm-up complete: fifo=0x%02x", fifo_status & 0xffU);

	return 0;
}

static int i2s_sf32lb_init(const struct device *dev)
{
	const struct i2s_sf32lb_config *config = dev->config;
	struct i2s_sf32lb_data *data = dev->data;
	int ret;

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("pinctrl_apply_state failed: %d", ret);
		return ret;
	}

	if (!sf32lb_clock_is_ready_dt(&config->clock)) {
		LOG_ERR("I2S clock not ready");
		return -ENODEV;
	}

	ret = sf32lb_clock_control_on_dt(&config->clock);
	if (ret < 0) {
		LOG_ERR("Failed to enable I2S clock: %d", ret);
		return ret;
	}

	data->i2s = (I2S_TypeDef *)config->base;
	if (!sf32lb_dma_is_ready_dt(&config->tx_dma) ||
	    !sf32lb_dma_is_ready_dt(&config->rx_dma)) {
		LOG_ERR("I2S DMA controller not ready");
		return -ENODEV;
	}

	/* Reset the I2S peripheral to a known state */
	ll_i2s_disable_tx(data->i2s);
	ll_i2s_disable_rx(data->i2s);

	k_msgq_init(&data->tx.queue, (char *)data->tx.queue_buffer,
		    sizeof(data->tx.queue_buffer[0]), I2S_SF32LB_QUEUE_SIZE);
	k_msgq_init(&data->rx.queue, (char *)data->rx.queue_buffer,
		    sizeof(data->rx.queue_buffer[0]), I2S_SF32LB_QUEUE_SIZE);
	data->tx.state = I2S_STATE_NOT_READY;
	data->rx.state = I2S_STATE_NOT_READY;
	data->tx.cfg_valid = false;
	data->rx.cfg_valid = false;
	data->rx_dumped = false;

	config->irq_config_func();

	LOG_DBG("I2S initialized, base=0x%08lx", config->base);

	return 0;
}

static DEVICE_API(i2s, i2s_sf32lb_driver_api) = {
	.configure = i2s_sf32lb_configure,
	.read = i2s_sf32lb_read,
	.write = i2s_sf32lb_write,
	.config_get = i2s_sf32lb_config_get,
	.trigger = i2s_sf32lb_trigger,
};

/* --------------------------------------------------------------------------
 * Device instantiation macro
 * -------------------------------------------------------------------------- */

#define I2S_SF32LB_DEFINE(n)                                                                      \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void i2s_sf32lb_irq_config_func_##n(void)                                           \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), i2s_sf32lb_isr,              \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
	static struct i2s_sf32lb_data i2s_sf32lb_data_##n;                                         \
	static const struct i2s_sf32lb_config i2s_sf32lb_config_##n = {                            \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                          \
		.clock = SF32LB_CLOCK_DT_INST_SPEC_GET(n),                                         \
		.tx_dma = SF32LB_DMA_DT_INST_SPEC_GET_BY_NAME(n, tx),                               \
		.rx_dma = SF32LB_DMA_DT_INST_SPEC_GET_BY_NAME(n, rx),                               \
		.irq_config_func = i2s_sf32lb_irq_config_func_##n,                                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, i2s_sf32lb_init, NULL, &i2s_sf32lb_data_##n,                      \
			      &i2s_sf32lb_config_##n, POST_KERNEL, CONFIG_I2S_INIT_PRIORITY,       \
			      &i2s_sf32lb_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2S_SF32LB_DEFINE)
