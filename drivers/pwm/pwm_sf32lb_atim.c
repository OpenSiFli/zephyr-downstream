/*
 * Copyright (c) 2025, Qingsong Gou <gouqs@hotmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_atim_pwm

#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/clock_control/sf32lb.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <ll_atim.h>

LOG_MODULE_REGISTER(pwm_sf32lb_atim, CONFIG_PWM_LOG_LEVEL);

#define MAX_CH_NUM        4U

struct pwm_sf32lb_atim_config {
	ATIM_TypeDef *tim;
	const struct pinctrl_dev_config *pincfg;
	struct sf32lb_clock_dt_spec clock;
	uint32_t prescaler;
};

static int pwm_sf32lb_atim_set_cycles(const struct device *dev, uint32_t channel,
				      uint32_t period_cycles, uint32_t pulse_cycles,
				      pwm_flags_t flags)
{
	const struct pwm_sf32lb_atim_config *cfg = dev->config;
	ATIM_TypeDef *tim = cfg->tim;

	if (channel >= MAX_CH_NUM) {
		return -EINVAL;
	}

	/* disable the channel */
	ll_atim_disable_channel(tim, channel + 1U);

	ll_atim_set_auto_reload(tim, period_cycles - 1U);
	ll_atim_set_compare(tim, channel + 1U, pulse_cycles);

	ll_atim_set_channel_oc_mode(tim, channel + 1U, LL_ATIM_OC_MODE_PWM1);
	ll_atim_set_channel_preload(tim, channel + 1U, 1U);

	ll_atim_set_channel_polarity(tim, channel + 1U,
				     (flags & PWM_POLARITY_INVERTED) ? 1U : 0U);

	/* enable the channel */
	ll_atim_enable_channel(tim, channel + 1U);

	return 0;
}

static int pwm_sf32lb_atim_get_cycles_per_sec(const struct device *dev, uint32_t channel,
					      uint64_t *cycles)
{
	const struct pwm_sf32lb_atim_config *cfg = dev->config;
	uint32_t clk_rate;

	if (sf32lb_clock_control_get_rate_dt(&cfg->clock, &clk_rate)) {
		return -EIO;
	}

	*cycles = clk_rate / (cfg->prescaler + 1);

	return 0;
}

static DEVICE_API(pwm, pwm_sf32lb_atim_api) = {
	.set_cycles = pwm_sf32lb_atim_set_cycles,
	.get_cycles_per_sec = pwm_sf32lb_atim_get_cycles_per_sec,
};

static int pwm_sf32lb_atim_init(const struct device *dev)
{
	const struct pwm_sf32lb_atim_config *cfg = dev->config;
	ATIM_TypeDef *tim = cfg->tim;
	int err;

	err = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	if (!sf32lb_clock_is_ready_dt(&cfg->clock)) {
		return -ENODEV;
	}

	err = sf32lb_clock_control_on_dt(&cfg->clock);
	if (err < 0) {
		return err;
	}

	ll_atim_set_prescaler(tim, (uint16_t)cfg->prescaler);
	ll_atim_generate_update(tim);
	ll_atim_enable_auto_reload_preload(tim);
	ll_atim_enable(tim);
	ll_atim_enable_main_output(tim);

	return err;
}

#define PWM_SF32LB_ATIM_DEFINE(n)                                                                  \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct pwm_sf32lb_atim_config pwm_sf32lb_atim_config_##n = {                  \
		.tim = (ATIM_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(n)),                             \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
		.clock = SF32LB_CLOCK_DT_INST_PARENT_SPEC_GET(n),                                  \
		.prescaler = DT_PROP(DT_INST_PARENT(n), sifli_prescaler),                          \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, pwm_sf32lb_atim_init, NULL, NULL,                                 \
			      &pwm_sf32lb_atim_config_##n, POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,  \
			      &pwm_sf32lb_atim_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_SF32LB_ATIM_DEFINE)
