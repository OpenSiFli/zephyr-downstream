/*
 * Copyright (c) 2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_crc

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control/sf32lb.h>
#include <zephyr/drivers/crc.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/sys_io.h>

#include <ll_crc.h>

/* Poll timeout in microseconds (10 ms) */
#define CRC_SF32LB_TIMEOUT_US 10000U

struct crc_sf32lb_config {
	uintptr_t base;
	struct sf32lb_clock_dt_spec clock;
};

struct crc_sf32lb_data {
	struct k_sem lock;
	uint8_t width;
	uint32_t xor_out;
};

static inline uint32_t crc_sf32lb_mask(uint8_t width)
{
	if (width >= 32U) {
		return UINT32_MAX;
	}

	return BIT_MASK(width);
}

static int crc_sf32lb_prepare_config(const struct crc_ctx *ctx, uint32_t *polysize, uint8_t *width,
				     uint32_t *xor_out)
{
	switch (ctx->type) {
	case CRC8:
		__fallthrough;
	case CRC8_CCITT:
		__fallthrough;
	case CRC8_ROHC:
		if (ctx->polynomial > UINT8_MAX) {
			return -EINVAL;
		}

		*polysize = LL_CRC_POLYSIZE_8B;
		*width = 8U;
		*xor_out = 0U;
		break;
	case CRC16:
		__fallthrough;
	case CRC16_CCITT:
		__fallthrough;
	case CRC16_ANSI:
		__fallthrough;
	case CRC16_ITU_T:
		if (ctx->polynomial > UINT16_MAX) {
			return -EINVAL;
		}

		*polysize = LL_CRC_POLYSIZE_16B;
		*width = 16U;
		*xor_out = 0U;
		break;
	case CRC32_IEEE:
		*polysize = LL_CRC_POLYSIZE_32B;
		*width = 32U;
		*xor_out = 0xFFFFFFFFU;
		break;
	case CRC32_C:
		__fallthrough;
	case CRC32_K_4_2:
		*polysize = LL_CRC_POLYSIZE_32B;
		*width = 32U;
		*xor_out = 0U;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static void crc_sf32lb_unlock(const struct device *dev)
{
	struct crc_sf32lb_data *data = dev->data;

	k_sem_give(&data->lock);
}

static uint32_t crc_sf32lb_get_result(const struct device *dev)
{
	const struct crc_sf32lb_config *config = dev->config;
	struct crc_sf32lb_data *data = dev->data;
	CRC_TypeDef *crc = (CRC_TypeDef *)config->base;
	uint32_t raw;
	uint32_t mask;

	raw = ll_crc_read_result(crc) ^ data->xor_out;
	mask = crc_sf32lb_mask(data->width);

	return raw & mask;
}

static int crc_sf32lb_begin(const struct device *dev, struct crc_ctx *ctx)
{
	const struct crc_sf32lb_config *config = dev->config;
	struct crc_sf32lb_data *data = dev->data;
	CRC_TypeDef *crc = (CRC_TypeDef *)config->base;
	ll_crc_ctrl_config_t ll_cfg = {
		.data_size = LL_CRC_DATASIZE_8B,
		.rev_in = LL_CRC_REV_IN_NONE,
		.rev_out = LL_CRC_REV_OUT_DISABLE,
	};
	uint32_t polysize;
	uint8_t width;
	uint32_t xor_out;
	uint32_t mask;
	int ret;

	if ((ctx == NULL) || (ctx->state != CRC_STATE_IDLE)) {
		return -EINVAL;
	}

	k_sem_take(&data->lock, K_FOREVER);

	ret = crc_sf32lb_prepare_config(ctx, &polysize, &width, &xor_out);
	if (ret != 0) {
		crc_sf32lb_unlock(dev);
		return ret;
	}

	data->width = width;
	data->xor_out = xor_out;
	mask = crc_sf32lb_mask(width);

	ll_cfg.poly_size = polysize;

	if ((ctx->reversed & CRC_FLAG_REVERSE_INPUT) != 0U) {
		ll_cfg.rev_in = LL_CRC_REV_IN_BY_BYTE;
	}

	if ((ctx->reversed & CRC_FLAG_REVERSE_OUTPUT) != 0U) {
		ll_cfg.rev_out = LL_CRC_REV_OUT_ENABLE;
	}

	ll_crc_config_ctrl(crc, &ll_cfg);
	ll_crc_set_init(crc, ctx->seed & mask);
	ll_crc_set_poly(crc, ctx->polynomial & mask);

	/* Reset data register to the provided seed */
	ll_crc_reset(crc);
	/* LL gap: ll_crc_reset() asserts RESET but does not expose a deassert helper. */
	sys_clear_bits((mem_addr_t)&crc->CR, CRC_CR_RESET);

	ctx->state = CRC_STATE_IN_PROGRESS;
	ctx->result = ctx->seed & mask;

	return 0;
}

static int crc_sf32lb_update(const struct device *dev, struct crc_ctx *ctx, const void *buffer,
			     size_t bufsize)
{
	const struct crc_sf32lb_config *config = dev->config;
	CRC_TypeDef *crc = (CRC_TypeDef *)config->base;
	const uint8_t *data_buf = buffer;

	if (ctx->state != CRC_STATE_IN_PROGRESS) {
		return -EINVAL;
	}

	if ((bufsize > 0U) && (data_buf == NULL)) {
		ctx->state = CRC_STATE_IDLE;
		crc_sf32lb_unlock(dev);
		return -EINVAL;
	}

	size_t aligned_len = ROUND_DOWN(bufsize, sizeof(uint32_t));
	size_t idx = 0U;

	for (; idx < aligned_len; idx += sizeof(uint32_t)) {
		uint32_t data = sys_get_le32(&data_buf[idx]);

		ll_crc_push_data32(crc, data);

		if (!WAIT_FOR(ll_crc_is_active_flag_done(crc) != 0U, CRC_SF32LB_TIMEOUT_US,
			      NULL)) {
			ctx->state = CRC_STATE_IDLE;
			crc_sf32lb_unlock(dev);
			return -ETIMEDOUT;
		}
	}

	/* Now we'll handle data that isn't 4-byte aligned */
	if (idx < bufsize) {
		uint32_t rem = 0U;
		size_t rem_bytes = bufsize - idx;

		sys_get_le(&rem, &data_buf[idx], rem_bytes);

		if (rem_bytes == sizeof(uint8_t)) {
			ll_crc_push_data8(crc, (uint8_t)rem);
		} else if (rem_bytes == sizeof(uint16_t)) {
			ll_crc_push_data16(crc, (uint16_t)rem);
		} else {
			ll_crc_push_data24(crc, rem);
		}

		if (!WAIT_FOR(ll_crc_is_active_flag_done(crc) != 0U, CRC_SF32LB_TIMEOUT_US,
			      NULL)) {
			ctx->state = CRC_STATE_IDLE;
			crc_sf32lb_unlock(dev);
			return -ETIMEDOUT;
		}
	}

	ctx->result = crc_sf32lb_get_result(dev);

	return 0;
}

static int crc_sf32lb_finish(const struct device *dev, struct crc_ctx *ctx)
{
	if (ctx->state != CRC_STATE_IN_PROGRESS) {
		return -EINVAL;
	}

	ctx->result = crc_sf32lb_get_result(dev);
	ctx->state = CRC_STATE_IDLE;

	crc_sf32lb_unlock(dev);

	return 0;
}

static DEVICE_API(crc, crc_sf32lb_driver_api) = {
	.begin = crc_sf32lb_begin,
	.update = crc_sf32lb_update,
	.finish = crc_sf32lb_finish,
};

static int crc_sf32lb_init(const struct device *dev)
{
	const struct crc_sf32lb_config *config = dev->config;
	struct crc_sf32lb_data *data = dev->data;
	int ret;

	if (!sf32lb_clock_is_ready_dt(&config->clock)) {
		return -ENODEV;
	}

	ret = sf32lb_clock_control_on_dt(&config->clock);
	if (ret != 0) {
		return ret;
	}

	k_sem_init(&data->lock, 1, 1);
	data->width = 32U;
	data->xor_out = 0U;

	return 0;
}

#define CRC_SF32LB_INIT(inst)                                                                      \
	static struct crc_sf32lb_data crc_sf32lb_data_##inst;                                      \
	static const struct crc_sf32lb_config crc_sf32lb_config_##inst = {                         \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.clock = SF32LB_CLOCK_DT_INST_SPEC_GET(inst),                                      \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, crc_sf32lb_init, NULL, &crc_sf32lb_data_##inst,                \
			      &crc_sf32lb_config_##inst, POST_KERNEL,                              \
			      CONFIG_CRC_DRIVER_INIT_PRIORITY, &crc_sf32lb_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CRC_SF32LB_INIT)
