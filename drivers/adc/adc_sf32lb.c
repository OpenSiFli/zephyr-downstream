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
#include <zephyr/sys/util.h>
#include <zephyr/sys/sys_io.h>

#include <ll_gpadc.h>
#include <ll_hpsys_cfg.h>

#if defined(CONFIG_ADC_SF32LB_CALIBRATION)
#include <zephyr/drivers/otp/sifli_sf32lb52x_efuse.h>
#endif

#if defined(CONFIG_ADC_SF32LB_CALIBRATION) && defined(CONFIG_NVMEM)
#include <zephyr/nvmem.h>
#endif

LOG_MODULE_REGISTER(adc_sf32lb, CONFIG_ADC_LOG_LEVEL);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#define ADC_CONTEXT_ENABLE_ON_COMPLETE
#include "adc_context.h"

#define SYS_CFG_ANAU_CR offsetof(HPSYS_CFG_TypeDef, ANAU_CR)
#define SYS_CFG_IDR     offsetof(HPSYS_CFG_TypeDef, IDR)

#define ADC_MAX_CH (8U)

#define ADC_SF32LB_RESOLUTION 12U
#define ADC_SF32LB_FULL_SCALE BIT(ADC_SF32LB_RESOLUTION)
#define ADC_SF32LB_MAX_RAW    (ADC_SF32LB_FULL_SCALE - 1U)

#define ADC_SF32LB_DEFAULT_VREF_INTERNAL 3300
#define ADC_SF32LB_DEFAULT_OFFSET_MCODE  822000
#define ADC_SF32LB_DEFAULT_RATIO_UV      1068
#define ADC_SF32LB_DEFAULT_VBAT_FACTOR_MILLI 2010
#define ADC_SF32LB_VBAT_CHANNEL 7U
#define ADC_SF32LB_ANAU_CMM_VALUE       0x10U

#define SF32LB_ADC_WAIT_TIME_US 200

#ifndef HAL_CHIP_REV_ID_A4
#define HAL_CHIP_REV_ID_A4 0x07
#endif

#ifndef HAL_CHIP_REV_ID_B4
#define HAL_CHIP_REV_ID_B4 0x0f
#endif

#if defined(CONFIG_ADC_SF32LB_CALIBRATION)
struct adc_sf32lb_calib {
	int32_t offset_mcode;
	uint32_t ratio_uv;
	uint32_t vbat_factor_milli;
	bool factory_valid;
	bool ldovref_valid;
	uint8_t ldovref_sel;
};
#endif /* CONFIG_ADC_SF32LB_CALIBRATION */

struct adc_sf32lb_data {
	struct adc_context ctx;
	const struct device *dev;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint32_t channels;
#if defined(CONFIG_ADC_SF32LB_CALIBRATION)
	struct adc_sf32lb_calib calib;
	bool calib_loaded;
	bool ldovref_applied;
#endif
};

struct adc_sf32lb_config {
	uintptr_t base;
	uintptr_t cfg_base;
	const struct pinctrl_dev_config *pcfg;
	struct sf32lb_clock_dt_spec clock;
	void (*irq_config_func)(void);
#if defined(CONFIG_ADC_SF32LB_CALIBRATION)
	enum sf32lb52x_efuse_adc_calib_source calibration_source;
	bool avdd_1v8;
#endif
#if defined(CONFIG_ADC_SF32LB_CALIBRATION) && defined(CONFIG_NVMEM)
	struct nvmem_cell calibration_cell;
	bool has_calibration_cell;
#endif
};

#if defined(CONFIG_ADC_SF32LB_CALIBRATION)
static void adc_sf32lb_calib_set_default(struct adc_sf32lb_calib *calib)
{
	*calib = (struct adc_sf32lb_calib){
		.offset_mcode = ADC_SF32LB_DEFAULT_OFFSET_MCODE,
		.ratio_uv = ADC_SF32LB_DEFAULT_RATIO_UV,
		.vbat_factor_milli = ADC_SF32LB_DEFAULT_VBAT_FACTOR_MILLI,
	};
}

static int32_t adc_sf32lb_raw_to_mv(const struct adc_sf32lb_calib *calib, uint16_t raw)
{
	const int64_t delta_mcode = (int64_t)raw * 1000 - calib->offset_mcode;

	return (int32_t)DIV_ROUND_CLOSEST(delta_mcode * calib->ratio_uv, 1000000);
}

static uint16_t adc_sf32lb_mv_to_raw(int32_t mv)
{
	int64_t raw;

	raw = DIV_ROUND_CLOSEST((int64_t)mv * ADC_SF32LB_FULL_SCALE,
				ADC_SF32LB_DEFAULT_VREF_INTERNAL);

	if (raw < 0) {
		return 0U;
	}

	if (raw > ADC_SF32LB_MAX_RAW) {
		return ADC_SF32LB_MAX_RAW;
	}

	return (uint16_t)raw;
}

__maybe_unused static int adc_sf32lb_calib_apply_factory(struct adc_sf32lb_calib *calib,
					  const struct sf32lb52x_efuse_adc_calib *factory,
					  bool letter_series)
{
	const uint32_t reg_gap = factory->reg1 > factory->reg2 ?
					 factory->reg1 - factory->reg2 :
					 factory->reg2 - factory->reg1;
	const uint32_t mv_gap = factory->mv1 > factory->mv2 ?
					factory->mv1 - factory->mv2 :
					factory->mv2 - factory->mv1;
	uint32_t ratio_uv;
	int64_t offset_mcode;

	if (factory->reg1 == 0U || factory->reg2 == 0U || factory->mv1 == 0U ||
	    factory->mv2 == 0U || reg_gap == 0U || mv_gap == 0U) {
		return -ENODATA;
	}

	ratio_uv = (uint32_t)DIV_ROUND_CLOSEST((uint64_t)mv_gap * 1000U, reg_gap);
	if (ratio_uv == 0U) {
		return -ENODATA;
	}

	offset_mcode = (int64_t)factory->reg1 * 1000 -
		       DIV_ROUND_CLOSEST((int64_t)factory->mv1 * 1000000, (int64_t)ratio_uv);

	*calib = (struct adc_sf32lb_calib){
		.offset_mcode = (int32_t)offset_mcode,
		.ratio_uv = ratio_uv,
		.vbat_factor_milli = ADC_SF32LB_DEFAULT_VBAT_FACTOR_MILLI,
		.factory_valid = true,
		.ldovref_valid = letter_series && factory->ldovref_flag != 0U,
		.ldovref_sel = factory->ldovref_sel,
	};

	if (factory->vbat_reg != 0U && factory->vbat_mv != 0U) {
		const int32_t vbat_adc_mv = adc_sf32lb_raw_to_mv(calib, factory->vbat_reg);

		if (vbat_adc_mv > 0) {
			calib->vbat_factor_milli = (uint32_t)DIV_ROUND_CLOSEST(
				(uint64_t)factory->vbat_mv * 1000U, (uint32_t)vbat_adc_mv);
		}
	}

	return 0;
}

__maybe_unused static bool adc_sf32lb_is_letter_series(const struct adc_sf32lb_config *config)
{
	const uint32_t idr = sys_read32(config->cfg_base + SYS_CFG_IDR);
	const uint8_t rev_id = FIELD_GET(HPSYS_CFG_IDR_REVID, idr);

	return rev_id == HAL_CHIP_REV_ID_A4 || rev_id == HAL_CHIP_REV_ID_B4;
}

static void adc_sf32lb_apply_ldovref(const struct device *dev)
{
	const struct adc_sf32lb_config *config = dev->config;
	struct adc_sf32lb_data *data = dev->data;
	GPADC_TypeDef *gpadc = (GPADC_TypeDef *)config->base;
	uint32_t reg;

	if (data->ldovref_applied || !data->calib.ldovref_valid) {
		return;
	}

	reg = sys_read32((mem_addr_t)&gpadc->ADC_CFG_REG1);
	reg &= ~GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOVREF_SEL;
	reg |= ((uint32_t)data->calib.ldovref_sel << GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOVREF_SEL_Pos) &
	       GPADC_ADC_CFG_REG1_ANAU_GPADC_LDOVREF_SEL;
	sys_write32(reg, (mem_addr_t)&gpadc->ADC_CFG_REG1);
	k_busy_wait(SF32LB_ADC_WAIT_TIME_US);

	data->ldovref_applied = true;
}

static void adc_sf32lb_ensure_calibration(const struct device *dev)
{
	__maybe_unused const struct adc_sf32lb_config *config = dev->config;
	struct adc_sf32lb_data *data = dev->data;
	bool retry_later = false;

	if (data->calib_loaded) {
		return;
	}

	adc_sf32lb_calib_set_default(&data->calib);

#if defined(CONFIG_SF32LB52X_EFUSE_FIELDS) && defined(CONFIG_NVMEM)
	if (config->has_calibration_cell) {
		enum sf32lb52x_efuse_adc_calib_source source;
		struct sf32lb52x_efuse_adc_calib factory;
		uint8_t bank1[SF32LB52X_EFUSE_BANK1_SIZE];
		bool letter_series;
		int ret;

		letter_series = adc_sf32lb_is_letter_series(config);
		source = sf32lb52x_efuse_resolve_adc_calib_source(config->calibration_source,
								  letter_series, config->avdd_1v8);

		ret = nvmem_cell_read(&config->calibration_cell, bank1, 0, sizeof(bank1));
		if (ret < 0) {
			LOG_WRN("Failed to read ADC calibration NVMEM cell: %d", ret);
			retry_later = true;
		} else {
			ret = sf32lb52x_efuse_decode_adc_calib(bank1, source, &factory);
			if (ret < 0) {
				LOG_WRN("Invalid ADC factory calibration: %d", ret);
			} else {
				ret = adc_sf32lb_calib_apply_factory(&data->calib, &factory,
								     letter_series);
				if (ret < 0) {
					LOG_WRN("Failed to apply ADC factory calibration: %d", ret);
				} else {
					LOG_DBG("ADC factory calibration loaded");
				}
			}
		}
	}
#endif

	data->calib_loaded = !retry_later;
	adc_sf32lb_apply_ldovref(dev);
}

static uint16_t adc_sf32lb_convert_sample(struct adc_sf32lb_data *data, uint8_t channel,
					  uint16_t raw)
{
	int32_t mv;

	raw = MIN(raw, ADC_SF32LB_MAX_RAW);
	mv = adc_sf32lb_raw_to_mv(&data->calib, raw);

	if (channel == ADC_SF32LB_VBAT_CHANNEL) {
		mv = (int32_t)DIV_ROUND_CLOSEST((int64_t)mv * data->calib.vbat_factor_milli,
						1000);
	}

	return adc_sf32lb_mv_to_raw(mv);
}
#else
static inline void adc_sf32lb_ensure_calibration(const struct device *dev)
{
	ARG_UNUSED(dev);
}

static inline uint16_t adc_sf32lb_convert_sample(struct adc_sf32lb_data *data, uint8_t channel,
						 uint16_t raw)
{
	ARG_UNUSED(data);
	ARG_UNUSED(channel);

	return raw;
}
#endif /* CONFIG_ADC_SF32LB_CALIBRATION */

static void adc_sf32lb_disable_analog(const struct device *dev)
{
	const struct adc_sf32lb_config *config = dev->config;
	GPADC_TypeDef *gpadc = (GPADC_TypeDef *)config->base;

	/* Mute analog front-end when ADC is idle. */
	sys_set_bits((mem_addr_t)&gpadc->ADC_CFG_REG1, GPADC_ADC_CFG_REG1_ANAU_GPADC_MUTE);
	ll_gpadc_disable_core(gpadc);
	ll_gpadc_disable_ldoref(gpadc);
}

static void adc_sf32lb_enable_analog(const struct device *dev)
{
	const struct adc_sf32lb_config *config = dev->config;
	GPADC_TypeDef *gpadc = (GPADC_TypeDef *)config->base;

	/* Unmute analog front-end before conversion. */
	sys_clear_bits((mem_addr_t)&gpadc->ADC_CFG_REG1, GPADC_ADC_CFG_REG1_ANAU_GPADC_MUTE);

	if (!ll_gpadc_is_enabled_ldoref(gpadc)) {
		ll_gpadc_enable_ldoref(gpadc);
		k_busy_wait(SF32LB_ADC_WAIT_TIME_US);
	}

	if (!ll_gpadc_is_enabled_core(gpadc)) {
		ll_gpadc_enable_core(gpadc);
		k_busy_wait(SF32LB_ADC_WAIT_TIME_US);
	}
}

static void adc_context_on_complete(struct adc_context *ctx, int status)
{
	struct adc_sf32lb_data *data = CONTAINER_OF(ctx, struct adc_sf32lb_data, ctx);

	ARG_UNUSED(status);
	adc_sf32lb_disable_analog(data->dev);
}

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
		*data->buffer++ =
			adc_sf32lb_convert_sample(data, channel,
						  ll_gpadc_get_slot_data(gpadc, channel));

		channels &= ~BIT(channel);
	}

	ll_gpadc_disable_it_eoc(gpadc);
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

	ll_gpadc_enable_it_eoc(gpadc);
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

	adc_sf32lb_ensure_calibration(dev);
	adc_sf32lb_enable_analog(dev);

	adc_context_start_read(&data->ctx, sequence);

	error = adc_context_wait_for_completion(&data->ctx);
	if (error < 0) {
		adc_sf32lb_disable_analog(dev);
	}

	return error;
}

static int adc_sf32lb_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_sf32lb_data *data = dev->data;
	int error;

	if (sequence->resolution != ADC_SF32LB_RESOLUTION) {
		LOG_ERR("Resolution %d is not supported", sequence->resolution);
		return -ENOTSUP;
	}

	if (sequence->oversampling) {
		LOG_ERR("Oversampling is not supported");
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

	if (sequence->resolution != ADC_SF32LB_RESOLUTION) {
		LOG_ERR("Resolution %d is not supported", sequence->resolution);
		return -ENOTSUP;
	}

	if (sequence->oversampling) {
		LOG_ERR("Oversampling is not supported");
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
	/*
	 * Timing basis for ADC_CTRL_REG2 fields:
	 * - PCLK is 120 MHz on this platform.
	 * - Factory ADC calibration is done around 240 kHz ADCCLK.
	 * - Datasheet formula:
	 *   fADCCLK = fPCLK / (DATA_SAMP_DLY + CONV_WIDTH + SAMP_WIDTH + 2)
	 * - With DATA_SAMP_DLY=2, CONV_WIDTH=251, SAMP_WIDTH=238:
	 *   fADCCLK = 120000000 / (2 + 251 + 238 + 2) = 243407 Hz,
	 *   which is close to the 240 kHz factory-calibration operating point.
	 */
	ll_gpadc_clock_config_t clock_config = {
		.data_samp_dly = 2U,
		.conv_width = 251U,
		.samp_width = 238U,
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

	/* Allow input path to settle after pinmux configuration. */
	k_busy_wait(SF32LB_ADC_WAIT_TIME_US);

	/* Enable the ANAU bandgap shared by GPADC. */
	ll_cfg_anau_bandgap_enable((HPSYS_CFG_TypeDef *)config->cfg_base);

	/* Disable GPIO-triggered sampling; timer trigger is configured next. */
	ll_gpadc_disable_gpio_trigger(gpadc);
	ll_gpadc_config_trigger(gpadc, &trigger_config);
	ll_gpadc_config_mode(gpadc, &mode_config);
	ll_gpadc_config_clock(gpadc, &clock_config);

	/*
	 * Update only key ADC_CFG_REG1 fields instead of full analog reconfig.
	 * - ANAU_GPADC_EN_V18 is only used when GPADC is powered by an external 1.8V supply,
	 *   so it is intentionally not forced here.
	 * - ANAU_GPADC_CMM affects conversion precision; set it explicitly to the validated value.
	 */
	ll_gpadc_config_analog_cfg1(gpadc, ADC_SF32LB_ANAU_CMM_VALUE, 1U);
	adc_sf32lb_disable_analog(dev);

	/* disable all slots */
	for (uint8_t i = 0; i < 8U; i++) {
		ll_gpadc_config_slot(gpadc, i, &slot_config);
	}

	/* Reset value enables EOC interrupt; disable when idle. */
	ll_gpadc_disable_it_eoc(gpadc);
	config->irq_config_func();

	data->dev = dev;

	adc_context_unlock_unconditionally(&data->ctx);

	return ret;
}

#if defined(CONFIG_ADC_SF32LB_CALIBRATION)
#define ADC_SF32LB_CALIB_CONFIG(n)                                                                \
	.calibration_source = DT_INST_ENUM_IDX_OR(n, sifli_calibration_source,                     \
						  SF32LB52X_EFUSE_ADC_CALIB_AUTO),                 \
	.avdd_1v8 = DT_INST_PROP(n, sifli_avdd_1v8),
#else
#define ADC_SF32LB_CALIB_CONFIG(n)
#endif

#if defined(CONFIG_ADC_SF32LB_CALIBRATION) && defined(CONFIG_NVMEM)
#define ADC_SF32LB_NVMEM_CONFIG(n)                                                                \
	.calibration_cell = NVMEM_CELL_INST_GET_BY_NAME_OR(n, calibration, {0}),                   \
	.has_calibration_cell = DT_INST_NVMEM_CELLS_HAS_NAME(n, calibration),
#else
#define ADC_SF32LB_NVMEM_CONFIG(n)
#endif

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
		ADC_SF32LB_CALIB_CONFIG(n)                                                        \
		ADC_SF32LB_NVMEM_CONFIG(n)                                                        \
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
