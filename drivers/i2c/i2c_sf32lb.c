/*
 * Copyright (c) 2025, Qingsong Gou <gouqs@hotmail.com>
 * Copyright (c) 2025, Haoran Jiang <halfsweet@halfsweet.cn>
 * Copyright (c) 2025, SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_i2c

#include "i2c-priv.h"

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control/sf32lb.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/dma/sf32lb.h>
#include <zephyr/sys/sys_io.h>

LOG_MODULE_REGISTER(i2c_sf32lb, CONFIG_I2C_LOG_LEVEL);

#include <ll_i2c.h>

#define SF32LB_I2C_TIMEOUT_MAX_US (30000)

#define SF32LB_I2C_DMA_MAX_LEN (512U)

struct i2c_sf32lb_config {
	uintptr_t base;
	const struct pinctrl_dev_config *pincfg;
	struct sf32lb_clock_dt_spec clock;
	uint32_t bitrate;
	void (*irq_cfg_func)(void);
	bool dma_used;
	struct sf32lb_dma_dt_spec dma_rx;
	struct sf32lb_dma_dt_spec dma_tx;
};

struct i2c_sf32lb_data {
	struct k_mutex lock;
	uint8_t rw_flags;
	struct k_sem i2c_compl;
	struct i2c_msg *current_msg;
	uint8_t *buf_ptr;
	uint32_t remaining;
	bool is_tx;
	int error;
};

static inline I2C_TypeDef *i2c_sf32lb_regs(const struct i2c_sf32lb_config *config)
{
	return (I2C_TypeDef *)config->base;
}

static inline uintptr_t i2c_sf32lb_fifo_addr(const struct i2c_sf32lb_config *config)
{
	return (uintptr_t)&i2c_sf32lb_regs(config)->FIFO;
}

static inline uint32_t i2c_sf32lb_get_status(I2C_TypeDef *i2c)
{
	/* LL gap: ll_i2c exposes per-flag readers, not an aggregate SR snapshot. */
	return sys_read32((mem_addr_t)&i2c->SR);
}

static inline void i2c_sf32lb_clear_status(I2C_TypeDef *i2c, uint32_t flags)
{
	/* LL gap: the address phase clears the complete latched SR snapshot. */
	sys_write32(flags, (mem_addr_t)&i2c->SR);
}

static inline void i2c_sf32lb_disable_all_irqs(I2C_TypeDef *i2c)
{
	/* LL gap: no helper clears the full IER register in one operation. */
	sys_write32(0, (mem_addr_t)&i2c->IER);
}

static inline void i2c_sf32lb_write_data(I2C_TypeDef *i2c, uint8_t data)
{
	/* LL gap: ll_i2c_transmit_byte() also starts TB; this path composes TCR separately. */
	sys_write32(data, (mem_addr_t)&i2c->DBR);
}

static inline void i2c_sf32lb_write_tcr(I2C_TypeDef *i2c, uint32_t tcr)
{
	/* LL gap: no helper writes the combined START/TB/NACK/STOP command atomically. */
	sys_write32(tcr, (mem_addr_t)&i2c->TCR);
}

static inline void i2c_sf32lb_enable_msde(I2C_TypeDef *i2c)
{
	/* LL gap: ll_i2c_config_master_runtime() also rewrites SCLPP. */
	sys_set_bits((mem_addr_t)&i2c->CR, I2C_CR_MSDE);
}

static inline void i2c_sf32lb_disable_msde(I2C_TypeDef *i2c)
{
	/* LL gap: no standalone MSDE clear helper. */
	sys_clear_bits((mem_addr_t)&i2c->CR, I2C_CR_MSDE);
}

static inline void i2c_sf32lb_config_speed_mode(I2C_TypeDef *i2c, uint32_t mode)
{
	mem_addr_t cr = (mem_addr_t)&i2c->CR;
	uint32_t value;

	/* LL gap: ll_i2c_config_timing() would also rewrite LCR/WCR timing registers. */
	value = sys_read32(cr);
	value &= ~I2C_CR_MODE;
	value |= mode;
	sys_write32(value, cr);
}

static inline uint32_t i2c_sf32lb_get_speed_mode(I2C_TypeDef *i2c)
{
	/* LL gap: no speed-mode getter exists without reading CR. */
	return sys_read32((mem_addr_t)&i2c->CR) & I2C_CR_MODE;
}

static inline void i2c_sf32lb_request_reset(I2C_TypeDef *i2c)
{
	/* LL gap: no software reset request helper in ll_i2c.h. */
	sys_set_bits((mem_addr_t)&i2c->CR, I2C_CR_RSTREQ);
}

static inline uint32_t i2c_sf32lb_is_reset_requested(I2C_TypeDef *i2c)
{
	/* LL gap: no software reset status helper in ll_i2c.h. */
	return sys_read32((mem_addr_t)&i2c->CR) & I2C_CR_RSTREQ;
}

static inline void i2c_sf32lb_enable_transfer_irqs(I2C_TypeDef *i2c, bool tx)
{
	if (tx) {
		ll_i2c_enable_it_te(i2c);
	} else {
		ll_i2c_enable_it_rf(i2c);
	}
	ll_i2c_enable_it_msd(i2c);
	ll_i2c_enable_it_bed(i2c);
}

static inline void i2c_sf32lb_enable_dma_irqs(I2C_TypeDef *i2c)
{
	ll_i2c_enable_it_dmadone(i2c);
	ll_i2c_enable_it_bed(i2c);
}

static inline void i2c_sf32lb_disable_dma_irqs(I2C_TypeDef *i2c)
{
	ll_i2c_disable_it_dmadone(i2c);
	ll_i2c_disable_it_bed(i2c);
}

static void i2c_sf32lb_tx_helper(const struct device *dev, uint32_t sr)
{
	const struct i2c_sf32lb_config *config = dev->config;
	struct i2c_sf32lb_data *data = dev->data;
	I2C_TypeDef *i2c = i2c_sf32lb_regs(config);
	uint32_t tcr;

	if (IS_BIT_SET(sr, I2C_SR_TE_Pos)) {
		ll_i2c_clear_flag_te(i2c);
		if (IS_BIT_SET(sr, I2C_SR_NACK_Pos)) {
			data->error = -EIO;
			i2c_sf32lb_disable_all_irqs(i2c);
			data->current_msg = NULL;
			k_sem_give(&data->i2c_compl);
			return;
		}

		if (data->remaining > 0) {
			i2c_sf32lb_write_data(i2c, *data->buf_ptr);
			data->buf_ptr++;
			data->remaining--;

			tcr = I2C_TCR_TB;
			if (data->remaining == 0 && i2c_is_stop_op(data->current_msg)) {
				tcr |= I2C_TCR_STOP;
			}
			i2c_sf32lb_write_tcr(i2c, tcr);
		} else {
			i2c_sf32lb_disable_all_irqs(i2c);
			data->current_msg = NULL;
			k_sem_give(&data->i2c_compl);
		}
	}

	if (IS_BIT_SET(sr, I2C_SR_MSD_Pos) && (data->remaining == 0)) {
		ll_i2c_clear_flag_msd(i2c);
		i2c_sf32lb_disable_all_irqs(i2c);
		data->current_msg = NULL;
		k_sem_give(&data->i2c_compl);
	}
}

static void i2c_sf32lb_rx_helper(const struct device *dev, uint32_t sr)
{
	const struct i2c_sf32lb_config *config = dev->config;
	struct i2c_sf32lb_data *data = dev->data;
	I2C_TypeDef *i2c = i2c_sf32lb_regs(config);
	uint32_t tcr;

	if (IS_BIT_SET(sr, I2C_SR_RF_Pos)) {
		ll_i2c_clear_flag_rf(i2c);

		if (data->remaining > 0) {
			if (IS_BIT_SET(sr, I2C_SR_NACK_Pos)) {
				data->error = -EIO;
				data->current_msg = NULL;
				k_sem_give(&data->i2c_compl);
				return;
			}
			*data->buf_ptr = ll_i2c_receive_byte(i2c);
			data->buf_ptr++;
			data->remaining--;

			tcr = I2C_TCR_TB;
			if (data->remaining == 0) {
				if (i2c_is_stop_op(data->current_msg)) {
					tcr |= I2C_TCR_STOP;
				}
				tcr |= I2C_TCR_NACK;
			}
			i2c_sf32lb_write_tcr(i2c, tcr);
		}
	}

	if (IS_BIT_SET(sr, I2C_SR_MSD_Pos) && (data->remaining == 0)) {
		ll_i2c_clear_flag_msd(i2c);
		i2c_sf32lb_disable_all_irqs(i2c);
		data->current_msg = NULL;
		k_sem_give(&data->i2c_compl);
	}
}

static void i2c_sf32lb_isr(const struct device *dev)
{
	const struct i2c_sf32lb_config *config = dev->config;
	struct i2c_sf32lb_data *data = dev->data;
	I2C_TypeDef *i2c = i2c_sf32lb_regs(config);
	uint32_t sr = i2c_sf32lb_get_status(i2c);

	if (IS_BIT_SET(sr, I2C_SR_BED_Pos)) {
		ll_i2c_clear_flag_bed(i2c);
		data->error = -EIO;
		i2c_sf32lb_disable_all_irqs(i2c);
		data->current_msg = NULL;
		k_sem_give(&data->i2c_compl);
		return;
	}
	if (config->dma_used) {
		if (IS_BIT_SET(sr, I2C_SR_DMADONE_Pos)) {
			ll_i2c_clear_flag_dmadone(i2c);
			ll_i2c_disable_dma(i2c);
			k_sem_give(&data->i2c_compl);
		}
	} else {
		if (data->is_tx) {
			i2c_sf32lb_tx_helper(dev, sr);
		} else {
			i2c_sf32lb_rx_helper(dev, sr);
		}
	}
}

static int i2c_sf32lb_send_addr(const struct device *dev, uint16_t addr, struct i2c_msg *msg)
{
	int ret = 0;
	const struct i2c_sf32lb_config *cfg = dev->config;
	I2C_TypeDef *i2c = i2c_sf32lb_regs(cfg);
	uint32_t tcr = 0;

	tcr |= I2C_TCR_START;
	tcr |= I2C_TCR_TB;

	if ((msg->len == 0) && i2c_is_stop_op(msg)) {
		tcr |= I2C_TCR_MA | I2C_TCR_STOP;
	}

	ll_i2c_transmit_address7(i2c, addr,
				 i2c_is_read_op(msg) ? LL_I2C_ADDR_DIR_READ
						     : LL_I2C_ADDR_DIR_WRITE);
	i2c_sf32lb_write_tcr(i2c, tcr);

	if (!WAIT_FOR(ll_i2c_is_active_flag_te(i2c), SF32LB_I2C_TIMEOUT_MAX_US, NULL)) {
		LOG_ERR("Abort timed out(I2C_SR: 0x%08x)", i2c_sf32lb_get_status(i2c));
		return -EIO;
	}

	i2c_sf32lb_clear_status(i2c, i2c_sf32lb_get_status(i2c));

	if (ll_i2c_get_ack_status(i2c)) {
		/* Wait for MSD(Master Stop Detected) to set, it appears slower than NACK */
		WAIT_FOR(ll_i2c_is_active_flag_msd(i2c), SF32LB_I2C_TIMEOUT_MAX_US, NULL);
		ret = -EIO;
	}

	if ((msg->len == 0) && i2c_is_stop_op(msg)) {
		if (!WAIT_FOR(!ll_i2c_is_active_flag_ub(i2c), SF32LB_I2C_TIMEOUT_MAX_US,
			      NULL)) {
			LOG_ERR("Stop timed out (I2C_SR:0x%08x)",
				i2c_sf32lb_get_status(i2c));
		}
	}

	return ret;
}

static int i2c_sf32lb_dma_tx_config(const struct device *dev, struct i2c_msg *msg)
{
	const struct i2c_sf32lb_config *config = dev->config;
	struct dma_config tx_dma_cfg = {0};
	struct dma_block_config dma_blk = {0};
	int err;

	sf32lb_dma_config_init_dt(&config->dma_tx, &tx_dma_cfg);

	tx_dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	tx_dma_cfg.block_count = 1U;
	tx_dma_cfg.source_data_size = 1U;
	tx_dma_cfg.dest_data_size = 1U;

	tx_dma_cfg.head_block = &dma_blk;
	dma_blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	dma_blk.source_address = (uint32_t)msg->buf;
	dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	dma_blk.dest_address = i2c_sf32lb_fifo_addr(config);
	dma_blk.block_size = msg->len;
	err = sf32lb_dma_config_dt(&config->dma_tx, &tx_dma_cfg);
	if (err < 0) {
		LOG_ERR("Error configuring Tx DMA (%d)", err);
		return err;
	}

	return 0;
}

static int i2c_sf32lb_dma_rx_config(const struct device *dev, struct i2c_msg *msg)
{
	const struct i2c_sf32lb_config *config = dev->config;
	struct dma_config rx_dma_cfg = {0};
	struct dma_block_config dma_blk = {0};
	int err;

	sf32lb_dma_config_init_dt(&config->dma_rx, &rx_dma_cfg);

	rx_dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	rx_dma_cfg.block_count = 1U;
	rx_dma_cfg.source_data_size = 1U;
	rx_dma_cfg.dest_data_size = 1U;

	rx_dma_cfg.head_block = &dma_blk;
	dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	dma_blk.source_address = i2c_sf32lb_fifo_addr(config);
	dma_blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	dma_blk.dest_address = (uint32_t)msg->buf;
	dma_blk.block_size = msg->len;

	err = sf32lb_dma_config_dt(&config->dma_rx, &rx_dma_cfg);
	if (err < 0) {
		LOG_ERR("Error configuring Rx DMA (%d)", err);
		return err;
	}

	return 0;
}

static int i2c_sf32lb_master_send_dma(const struct device *dev, uint16_t addr, struct i2c_msg *msg)
{
	int ret;
	const struct i2c_sf32lb_config *config = dev->config;
	struct i2c_sf32lb_data *data = dev->data;
	I2C_TypeDef *i2c = i2c_sf32lb_regs(config);
	bool addr_sent = (data->rw_flags != (msg->flags & I2C_MSG_RW_MASK));
	bool stop_needed = i2c_is_stop_op(msg);

	data->rw_flags = msg->flags & I2C_MSG_RW_MASK;
	data->error = 0;

	if (msg->len > SF32LB_I2C_DMA_MAX_LEN) {
		LOG_ERR("DMA length %d exceeds max %d",
			msg->len, SF32LB_I2C_DMA_MAX_LEN);
		return -ENOTSUP;
	}

	if (addr_sent) {
		ret = i2c_sf32lb_send_addr(dev, addr, msg);
		if (ret < 0) {
			return ret;
		}
	}

	if (msg->len == 0) {
		/* Zero-length message already handled in send_addr */
		return ret;
	}

	if (stop_needed) {
		ll_i2c_config_dma_last(i2c, 1, 0);
	}

	ll_i2c_clear_flag_dmadone(i2c);
	i2c_sf32lb_enable_dma_irqs(i2c);
	i2c_sf32lb_enable_msde(i2c);

	ret = i2c_sf32lb_dma_tx_config(dev, msg);
	if (ret < 0) {
		return ret;
	}

	ret = sf32lb_dma_start_dt(&config->dma_tx);
	if (ret < 0) {
		return ret;
	}

	ll_i2c_enable_dma(i2c);
	ll_i2c_set_dma_count(i2c, msg->len);
	ll_i2c_request_dma_tx(i2c);

	ret = k_sem_take(&data->i2c_compl, K_MSEC(SF32LB_I2C_TIMEOUT_MAX_US / 1000));
	if (ret < 0) {
		LOG_ERR("master send timeout");
		sf32lb_dma_stop_dt(&config->dma_tx);
		ll_i2c_disable_dma(i2c);
		i2c_sf32lb_disable_dma_irqs(i2c);
		ll_i2c_config_dma_last(i2c, 0, 0);
		i2c_sf32lb_disable_msde(i2c);
		return ret;
	}
	ll_i2c_disable_dma(i2c);
	ll_i2c_clear_flag_dmadone(i2c);
	sf32lb_dma_stop_dt(&config->dma_tx);

	/* Wait for bus idle if stop was issued */
	if (stop_needed) {
		if (!WAIT_FOR(!ll_i2c_is_active_flag_ub(i2c), SF32LB_I2C_TIMEOUT_MAX_US,
			      NULL)) {
			LOG_ERR("Wait for bus idle timeout");
			return -ETIMEDOUT;
		}
		ll_i2c_config_dma_last(i2c, 0, 0);
		i2c_sf32lb_disable_msde(i2c);
	}

	if (data->error != 0) {
		ret = data->error;
		data->error = 0;
	}

	return ret;
}

static int i2c_sf32lb_master_recv_dma(const struct device *dev, uint16_t addr, struct i2c_msg *msg)
{
	const struct i2c_sf32lb_config *config = dev->config;
	struct i2c_sf32lb_data *data = dev->data;
	I2C_TypeDef *i2c = i2c_sf32lb_regs(config);
	bool addr_sent = (data->rw_flags != (msg->flags & I2C_MSG_RW_MASK));
	bool stop_needed = i2c_is_stop_op(msg);
	int ret;

	data->rw_flags = msg->flags & I2C_MSG_RW_MASK;
	data->error = 0;

	if (msg->len > SF32LB_I2C_DMA_MAX_LEN) {
		return -ENOTSUP;
	}

	if (addr_sent) {
		ret = i2c_sf32lb_send_addr(dev, addr, msg);
		if (ret < 0) {
			return ret;
		}
	}

	if (msg->len == 0) {
		return 0;
	}

	if (stop_needed) {
		ll_i2c_config_dma_last(i2c, 1, 1);
	}

	ll_i2c_clear_flag_dmadone(i2c);
	i2c_sf32lb_enable_dma_irqs(i2c);
	i2c_sf32lb_enable_msde(i2c);

	ret = i2c_sf32lb_dma_rx_config(dev, msg);
	if (ret < 0) {
		return ret;
	}

	ret = sf32lb_dma_start_dt(&config->dma_rx);
	if (ret < 0) {
		return ret;
	}

	ll_i2c_enable_dma(i2c);
	ll_i2c_set_dma_count(i2c, msg->len);
	ll_i2c_request_dma_rx(i2c);

	ret = k_sem_take(&data->i2c_compl, K_MSEC(SF32LB_I2C_TIMEOUT_MAX_US / 1000));
	if (ret < 0) {
		LOG_ERR("master recv timeout");
		ll_i2c_disable_dma(i2c);
		ll_i2c_clear_flag_dmadone(i2c);
		sf32lb_dma_stop_dt(&config->dma_rx);
		i2c_sf32lb_disable_all_irqs(i2c);
		return ret;
	}

	ll_i2c_disable_dma(i2c);
	ll_i2c_clear_flag_dmadone(i2c);
	sf32lb_dma_stop_dt(&config->dma_rx);

	if (stop_needed) {
		if (!WAIT_FOR(!ll_i2c_is_active_flag_ub(i2c), SF32LB_I2C_TIMEOUT_MAX_US,
			      NULL)) {
			LOG_ERR("Stop timed out (I2C_SR:0x%08x)",
				i2c_sf32lb_get_status(i2c));
		}
		ll_i2c_config_dma_last(i2c, 0, 0);
	}

	if (data->error != 0) {
		ret = data->error;
		data->error = 0;
	}

	return ret;
}

static int i2c_sf32lb_master_send(const struct device *dev, uint16_t addr, struct i2c_msg *msg)
{
	int ret = 0;
	const struct i2c_sf32lb_config *cfg = dev->config;
	struct i2c_sf32lb_data *data = dev->data;
	I2C_TypeDef *i2c = i2c_sf32lb_regs(cfg);
	uint32_t tcr = I2C_TCR_TB;
	bool stop_needed = i2c_is_stop_op(msg);
	bool addr_sent = (data->rw_flags != (msg->flags & I2C_MSG_RW_MASK));

	data->rw_flags = msg->flags & I2C_MSG_RW_MASK;

	if (addr_sent) {
		ret = i2c_sf32lb_send_addr(dev, addr, msg);
		if (ret < 0) {
			return ret;
		}
	}

	if (msg->len == 0) {
		/* Zero-length message already handled in send_addr */
		return ret;
	}

	data->current_msg = msg;
	data->buf_ptr = msg->buf;
	data->remaining = msg->len;
	data->is_tx = true;
	data->error = 0;

	ll_i2c_clear_flag_te(i2c);

	i2c_sf32lb_write_data(i2c, *data->buf_ptr);
	data->buf_ptr++;
	data->remaining--;

	if (data->remaining == 0 && stop_needed) {
		tcr |= I2C_TCR_STOP;
	}
	i2c_sf32lb_write_tcr(i2c, tcr);

	i2c_sf32lb_enable_transfer_irqs(i2c, true);

	if (k_sem_take(&data->i2c_compl, K_MSEC(SF32LB_I2C_TIMEOUT_MAX_US / 1000)) != 0) {
		LOG_ERR("master sent timeout");
		i2c_sf32lb_disable_all_irqs(i2c);
		data->current_msg = NULL;
		return -ETIMEDOUT;
	}

	i2c_sf32lb_disable_all_irqs(i2c);

	if (data->error != 0) {
		ret = data->error;
	}

	return ret;
}

static int i2c_sf32lb_master_recv(const struct device *dev, uint16_t addr, struct i2c_msg *msg)
{
	int ret;
	const struct i2c_sf32lb_config *cfg = dev->config;
	struct i2c_sf32lb_data *data = dev->data;
	I2C_TypeDef *i2c = i2c_sf32lb_regs(cfg);
	uint32_t tcr = I2C_TCR_TB;
	bool stop_needed = i2c_is_stop_op(msg);

	data->rw_flags = msg->flags & I2C_MSG_RW_MASK;

	ret = i2c_sf32lb_send_addr(dev, addr, msg);
	if (ret < 0) {
		return ret;
	}

	if (msg->len == 0) {
		return ret;
	}

	data->current_msg = msg;
	data->buf_ptr = msg->buf;
	data->remaining = msg->len;
	data->is_tx = false;
	data->error = 0;

	ll_i2c_clear_flag_rf(i2c);

	if (data->remaining == 1) {
		if (stop_needed) {
			tcr |= I2C_TCR_STOP;
		}
		tcr |= I2C_TCR_NACK;
	}
	i2c_sf32lb_write_tcr(i2c, tcr);

	i2c_sf32lb_enable_msde(i2c);
	i2c_sf32lb_enable_transfer_irqs(i2c, false);

	if (k_sem_take(&data->i2c_compl, K_MSEC(SF32LB_I2C_TIMEOUT_MAX_US / 1000)) != 0) {
		LOG_ERR("master recv timeout");
		i2c_sf32lb_disable_all_irqs(i2c);
		data->current_msg = NULL;
		return -ETIMEDOUT;
	}

	i2c_sf32lb_disable_all_irqs(i2c);

	if (data->error != 0) {
		ret = data->error;
	}

	return ret;
}

static int i2c_sf32lb_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_sf32lb_config *cfg = dev->config;
	struct i2c_sf32lb_data *data = dev->data;
	I2C_TypeDef *i2c = i2c_sf32lb_regs(cfg);
	uint32_t mode;

	if (!(I2C_MODE_CONTROLLER & dev_config)) {
		return -ENOTSUP;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		mode = LL_I2C_MODE_STANDARD;
		break;

	case I2C_SPEED_FAST:
		mode = LL_I2C_MODE_FAST;
		break;

	case I2C_SPEED_FAST_PLUS:
		mode = LL_I2C_MODE_HS_STD_FALLBK;
		break;

	case I2C_SPEED_HIGH:
		mode = LL_I2C_MODE_HS_FAST_FALLBK;
		break;

	default:
		LOG_ERR("Unsupported I2C speed requested:%d", I2C_SPEED_GET(dev_config));
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	i2c_sf32lb_config_speed_mode(i2c, mode);
	/* Avoid sharing the same address with targets, 0x7c is a reserved address */
	ll_i2c_set_slave_address(i2c, 0x7CU);
	k_mutex_unlock(&data->lock);

	return 0;
}

static int i2c_sf32lb_get_config(const struct device *dev, uint32_t *dev_config)
{
	const struct i2c_sf32lb_config *cfg = dev->config;
	I2C_TypeDef *i2c = i2c_sf32lb_regs(cfg);
	uint32_t mode = i2c_sf32lb_get_speed_mode(i2c);

	*dev_config = I2C_MODE_CONTROLLER;

	switch (mode) {
	case LL_I2C_MODE_STANDARD:
		*dev_config |= I2C_SPEED_SET(I2C_SPEED_STANDARD);
		break;
	case LL_I2C_MODE_FAST:
		*dev_config |= I2C_SPEED_SET(I2C_SPEED_FAST);
		break;
	case LL_I2C_MODE_HS_STD_FALLBK:
		*dev_config |= I2C_SPEED_SET(I2C_SPEED_FAST_PLUS);
		break;
	case LL_I2C_MODE_HS_FAST_FALLBK:
		*dev_config |= I2C_SPEED_SET(I2C_SPEED_HIGH);
		break;
	default:
		return -ERANGE;
	}

	return 0;
}

static int i2c_sf32lb_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			       uint16_t addr)
{
	const struct i2c_sf32lb_config *cfg = dev->config;
	struct i2c_sf32lb_data *data = dev->data;
	I2C_TypeDef *i2c = i2c_sf32lb_regs(cfg);
	int ret = 0;

	k_mutex_lock(&data->lock, K_FOREVER);

	ll_i2c_enable(i2c);
	i2c_sf32lb_enable_msde(i2c);

	if (ll_i2c_is_active_flag_ub(i2c)) {
		k_mutex_unlock(&data->lock);
		LOG_ERR("Bus busy");
		return -EBUSY;
	};

	for (uint8_t i = 0U; i < num_msgs; i++) {
		if (I2C_MSG_ADDR_10_BITS & msgs->flags) {
			ret = -ENOTSUP;
			break;
		}

		if (msgs[i].flags & I2C_MSG_READ) {
			if (cfg->dma_used) {
				ret = i2c_sf32lb_master_recv_dma(dev, addr, &msgs[i]);
			} else {
				ret = i2c_sf32lb_master_recv(dev, addr, &msgs[i]);
			}
			if (ret < 0) {
				break;
			}
		} else {
			if (cfg->dma_used) {
				ret = i2c_sf32lb_master_send_dma(dev, addr, &msgs[i]);
			} else {
				ret = i2c_sf32lb_master_send(dev, addr, &msgs[i]);
			}
			if (ret < 0) {
				break;
			}
		}
	}

	ll_i2c_disable(i2c);

	data->rw_flags = I2C_MSG_READ;

	k_mutex_unlock(&data->lock);

	return 0;
}

static int i2c_sf32lb_recover_bus(const struct device *dev)
{
	const struct i2c_sf32lb_config *config = dev->config;
	I2C_TypeDef *i2c = i2c_sf32lb_regs(config);

	i2c_sf32lb_request_reset(i2c);
	while (i2c_sf32lb_is_reset_requested(i2c)) {
	}

	return 0;
}

static DEVICE_API(i2c, i2c_sf32lb_driver_api) = {
	.configure = i2c_sf32lb_configure,
	.get_config = i2c_sf32lb_get_config,
	.transfer = i2c_sf32lb_transfer,
	.recover_bus = i2c_sf32lb_recover_bus,
};

static int i2c_sf32lb_init(const struct device *dev)
{
	const struct i2c_sf32lb_config *config = dev->config;
	struct i2c_sf32lb_data *data = dev->data;
	int ret;

	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	if (!sf32lb_clock_is_ready_dt(&config->clock)) {
		return -ENODEV;
	}

	if (config->dma_used) {
		if (!sf32lb_dma_is_ready_dt(&config->dma_tx)) {
			LOG_ERR("Tx DMA channel not ready");
			return -ENODEV;
		}

		if (!sf32lb_dma_is_ready_dt(&config->dma_rx)) {
			LOG_ERR("Rx DMA channel not ready");
			return -ENODEV;
		}

		ll_i2c_enable_it_dmadone(i2c_sf32lb_regs(config));
	}
	ret = sf32lb_clock_control_on_dt(&config->clock);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_sf32lb_configure(dev, I2C_MODE_CONTROLLER | i2c_map_dt_bitrate(config->bitrate));
	if (ret < 0) {
		return ret;
	}

	ll_i2c_enable(i2c_sf32lb_regs(config));
	ll_i2c_enable_scl(i2c_sf32lb_regs(config));

	data->rw_flags = I2C_MSG_READ;

	config->irq_cfg_func();

	return ret;
}

#define I2C_SF32LB_DEFINE(n)                                                                       \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void i2c_sf32lb_irq_config_func_##n(void)                                           \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), i2c_sf32lb_isr,             \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	};                                                                                         \
	static struct i2c_sf32lb_data i2c_sf32lb_data_##n = {                                      \
		.lock = Z_MUTEX_INITIALIZER(i2c_sf32lb_data_##n.lock),                             \
		.i2c_compl = Z_SEM_INITIALIZER(i2c_sf32lb_data_##n.i2c_compl, 0, 1),               \
	};                                                                                         \
	static const struct i2c_sf32lb_config i2c_sf32lb_config_##n = {                            \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
		.clock = SF32LB_CLOCK_DT_INST_SPEC_GET(n),                                         \
		.bitrate = DT_INST_PROP_OR(n, clock_frequency, 100000),                            \
		.irq_cfg_func = i2c_sf32lb_irq_config_func_##n,                                    \
		.dma_used = DT_INST_NODE_HAS_PROP(n, dmas),                                        \
		.dma_tx = SF32LB_DMA_DT_INST_SPEC_GET_BY_NAME_OR(n, tx, {}),                       \
		.dma_rx = SF32LB_DMA_DT_INST_SPEC_GET_BY_NAME_OR(n, rx, {}),                       \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, i2c_sf32lb_init, NULL, &i2c_sf32lb_data_##n,                      \
			      &i2c_sf32lb_config_##n, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,       \
			      &i2c_sf32lb_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_SF32LB_DEFINE)
