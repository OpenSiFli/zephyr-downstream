/*
 * Copyright (c) 2025, Qingsong Gou <gouqs@hotmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_gpadc

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/clock_control/sf32lb.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sys/sys_io.h>

#include <ll_gpadc.h>

LOG_MODULE_REGISTER(adc_sf32lb, CONFIG_ADC_LOG_LEVEL);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#define SYS_CFG_ANAU_CR offsetof(HPSYS_CFG_TypeDef, ANAU_CR)

#define ADC_MAX_CH (8U)

#define ADC_SF32LB_DEFAULT_VREF_INTERNAL 3300

#define SF32LB_ADC_WAIT_TIME_US 200

struct adc_sf32lb_data {
	struct adc_context ctx;
	const struct device *dev;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint32_t channels;
};

struct adc_sf32lb_config {
	uintptr_t base;
	uintptr_t cfg_base;
	const struct pinctrl_dev_config *pcfg;
	struct sf32lb_clock_dt_spec clock;
	void (*irq_config_func)(void);
};

static void adc_sf32lb_isr(const struct device *dev)
{
	const struct adc_sf32lb_config *config = dev->config;
	struct adc_sf32lb_data *data = dev->data;
	GPADC_TypeDef *gpadc = (GPADC_TypeDef *)config->base;
	uint16_t channel;
	uint32_t channels;

	if (ll_gpadc_is_active_flag_irq_raw(gpadc) == 0U) {
		return;
	}

	ll_gpadc_clear_flag_irq(gpadc);

	channels = data->channels;
	while (channels) {
		channel = find_lsb_set(channels) - 1;
		*data->buffer++ = ll_gpadc_get_slot_data(gpadc, channel);

		channels &= ~BIT(channel);
	}

	adc_context_on_sampling_done(&data->ctx, dev);
}

static int adc_sf32lb_channel_setup(const struct device *dev,
				    const struct adc_channel_cfg *channel_cfg)
{
	const struct adc_sf32lb_config *config = dev->config;
	GPADC_TypeDef *gpadc = (GPADC_TypeDef *)config->base;
	ll_gpadc_slot_config_t slot_config = {
		.slot_enable = 1U,
	};
	uint8_t channel_id;

	channel_id = channel_cfg->channel_id;

	if (channel_cfg->channel_id >= ADC_MAX_CH) {
		LOG_ERR("Channel %d is not valid", channel_cfg->channel_id);
		return -EINVAL;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("Acquisition time is not supported");
		return -ENOTSUP;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Gain is not supported");
		return -ENOTSUP;
	}

	if (channel_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("External reference is not supported");
		return -ENOTSUP;
	}

	slot_config.p_channel = channel_id;
	if (channel_cfg->differential) {
		slot_config.n_channel = channel_id;
	}

	ll_gpadc_config_slot(gpadc, channel_id, &slot_config);

	return 0;
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct adc_sf32lb_data *data = CONTAINER_OF(ctx, struct adc_sf32lb_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static int check_buffer_size(const struct adc_sequence *sequence, uint8_t active_channels)
{
	size_t needed_buffer_size;

	needed_buffer_size = active_channels * sizeof(uint16_t);
	if (sequence->options) {
		needed_buffer_size *= (1U + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed_buffer_size) {
		LOG_ERR("Provided buffer is too small (%u/%u)", sequence->buffer_size,
			needed_buffer_size);
		return -ENOMEM;
	}
	return 0;
}

static void adc_sf32lb_start_conversion(const struct device *dev)
{
	const struct adc_sf32lb_config *const cfg = dev->config;
	GPADC_TypeDef *gpadc = (GPADC_TypeDef *)cfg->base;

	ll_gpadc_request_start(gpadc);
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_sf32lb_data *data = CONTAINER_OF(ctx, struct adc_sf32lb_data, ctx);

	adc_sf32lb_start_conversion(data->dev);
}

static int start_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_sf32lb_data *data = dev->data;
	uint8_t num_active_channels;
	int error;

	data->channels = sequence->channels;

	num_active_channels = sys_count_bits(&data->channels, sizeof(data->channels));
	error = check_buffer_size(sequence, num_active_channels);
	if (error < 0) {
		return error;
	}

	data->buffer = sequence->buffer;
	data->repeat_buffer = sequence->buffer;

	adc_context_start_read(&data->ctx, sequence);

	error = adc_context_wait_for_completion(&data->ctx);

	return error;
}

static int adc_sf32lb_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_sf32lb_data *data = dev->data;
	int error;

	if (sequence->resolution != 12U) {
		LOG_ERR("Resolution %d is not supported", sequence->resolution);
		return -ENOTSUP;
	}

	if (sequence->oversampling) {
		LOG_ERR("Oversampling is not supported");
		return -ENOTSUP;
	}

	if (sequence->calibrate) {
		LOG_ERR("Calibration is not supported");
		return -ENOTSUP;
	}

	adc_context_lock(&data->ctx, false, NULL);
	error = start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}

#ifdef CONFIG_ADC_ASYNC
static int adc_sf32lb_read_async(const struct device *dev, const struct adc_sequence *sequence,
				 struct k_poll_signal *async)
{
	struct adc_sf32lb_data *data = dev->data;
	int error;

	if (sequence->resolution != 12U) {
		LOG_ERR("Resolution %d is not supported", sequence->resolution);
		return -ENOTSUP;
	}

	if (sequence->oversampling) {
		LOG_ERR("Oversampling is not supported");
		return -ENOTSUP;
	}

	if (sequence->calibrate) {
		LOG_ERR("Calibration is not supported");
		return -ENOTSUP;
	}

	adc_context_lock(&data->ctx, true, async);
	error = start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}
#endif /* CONFIG_ADC_ASYNC */

static DEVICE_API(adc, adc_sf32lb_driver_api) = {
	.channel_setup = adc_sf32lb_channel_setup,
	.read = adc_sf32lb_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = adc_sf32lb_read_async,
#endif
	.ref_internal = ADC_SF32LB_DEFAULT_VREF_INTERNAL,
};

static int adc_sf32lb_init(const struct device *dev)
{
	const struct adc_sf32lb_config *config = dev->config;
	struct adc_sf32lb_data *data = dev->data;
	GPADC_TypeDef *gpadc = (GPADC_TypeDef *)config->base;
	ll_gpadc_mode_config_t mode_config = {
		.op_mode = LL_GPADC_OP_MODE_SINGLE,
		.init_time = 8U,
	};
	ll_gpadc_clock_config_t clock_config = {
		.data_samp_dly = 2U,
	};
	ll_gpadc_trigger_config_t trigger_config = {
		.timer_enable = 0U,
	};
	ll_gpadc_slot_config_t slot_config = {
		.slot_enable = 0U,
	};
	int ret;

	if (!sf32lb_clock_is_ready_dt(&config->clock)) {
		return -ENODEV;
	}

	ret = sf32lb_clock_control_on_dt(&config->clock);
	if (ret < 0) {
		return ret;
	}

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	/* LL gap: HPSYS_CFG ANAU bandgap control is not covered by ll_gpadc.h. */
	sys_set_bit(config->cfg_base + SYS_CFG_ANAU_CR, HPSYS_CFG_ANAU_CR_EN_BG_Pos);

	/* LL gap: GPIO trigger enable has no GPADC LL helper. */
	sys_clear_bits((mem_addr_t)&gpadc->ADC_CTRL_REG, GPADC_ADC_CTRL_REG_GPIO_TRIG_EN);
	ll_gpadc_config_trigger(gpadc, &trigger_config);
	ll_gpadc_config_mode(gpadc, &mode_config);
	ll_gpadc_config_clock(gpadc, &clock_config);
	ll_gpadc_disable_core(gpadc);

	/* enable ref ldo */
	/* LL gap: use narrow register updates for SE and V18 instead of full analog reconfig. */
	sys_set_bits((mem_addr_t)&gpadc->ADC_CFG_REG1,
		     GPADC_ADC_CFG_REG1_ANAU_GPADC_SE | GPADC_ADC_CFG_REG1_ANAU_GPADC_EN_V18);
	ll_gpadc_enable_ldoref(gpadc);
	k_busy_wait(SF32LB_ADC_WAIT_TIME_US); /* wait for stable */
	/* disable all slots */
	for (uint8_t i = 0; i < 8U; i++) {
		ll_gpadc_config_slot(gpadc, i, &slot_config);
	}

	config->irq_config_func();

	data->dev = dev;

	adc_context_unlock_unconditionally(&data->ctx);

	return ret;
}

#define ADC_SF32LB_DEFINE(n)                                                                       \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void adc_sf32lb_irq_config_func_##n(void);                                          \
	static struct adc_sf32lb_data adc_sf32lb_data_##n = {                                      \
		ADC_CONTEXT_INIT_TIMER(adc_sf32lb_data_##n, ctx),                                  \
		ADC_CONTEXT_INIT_LOCK(adc_sf32lb_data_##n, ctx),                                   \
		ADC_CONTEXT_INIT_SYNC(adc_sf32lb_data_##n, ctx),                                   \
	};                                                                                         \
	static const struct adc_sf32lb_config adc_sf32lb_config_##n = {                            \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.cfg_base = DT_REG_ADDR(DT_INST_PHANDLE(n, sifli_cfg)),                            \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.clock = SF32LB_CLOCK_DT_INST_SPEC_GET(n),                                         \
		.irq_config_func = adc_sf32lb_irq_config_func_##n,                                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, adc_sf32lb_init, NULL, &adc_sf32lb_data_##n,                      \
			      &adc_sf32lb_config_##n, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,       \
			      &adc_sf32lb_driver_api);                                             \
	static void adc_sf32lb_irq_config_func_##n(void)                                           \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), adc_sf32lb_isr,             \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

DT_INST_FOREACH_STATUS_OKAY(ADC_SF32LB_DEFINE)
