/*
 * Copyright (c) 2025, Qingsong Gou <gouqs@hotmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_gpt_pwm

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/clock_control/sf32lb.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>

#include <register.h>

#define GPT_CCMRX(ch) (((ch) < 2U) ? GPT_CCMR1 : GPT_CCMR2)

#define CCRX(ch)      (GPT_CCR1 + ((ch) << 2U))

#define GPT_CR1   offsetof(GPT_TypeDef, CR1)
#define GPT_PSC   offsetof(GPT_TypeDef, PSC)
#define GPT_ARR   offsetof(GPT_TypeDef, ARR)
#define GPT_CCR1  offsetof(GPT_TypeDef, CCR1)
#define GPT_CCER  offsetof(GPT_TypeDef, CCER)
#define GPT_CCMR1 offsetof(GPT_TypeDef, CCMR1)
#define GPT_CCMR2 offsetof(GPT_TypeDef, CCMR2)
#define GPT_EGR   offsetof(GPT_TypeDef, EGR)


#ifdef CONFIG_PWM_CAPTURE
#define GPT_SR    offsetof(GPT_TypeDef, SR)
#define GPT_SMCR  offsetof(GPT_TypeDef, SMCR)
#define GPT_DIER  offsetof(GPT_TypeDef, DIER)

#define GPT_SLAVEMODE_RESET FIELD_PREP(GPT_SMCR_SMS, 0x4U)

#define GPT_SMCR_TS_TI1FP1   (5U << GPT_SMCR_TS_Pos)
#define GPT_SMCR_TS_TI2FP2   (6U << GPT_SMCR_TS_Pos)
#endif

/* PWM output mode 1 (needed by pwm_sf32lb_set_cycles even without capture) */
#define GPT_CCMR1_OC1M_PWM1 FIELD_PREP(GPT_CCMR1_OC1M, 6U)
#define GPT_CCMR1_OC2M_PWM1 FIELD_PREP(GPT_CCMR1_OC2M, 6U)
#define GPT_CCMR2_OC3M_PWM1 FIELD_PREP(GPT_CCMR2_OC3M, 6U)
#define GPT_CCMR2_OC4M_PWM1 FIELD_PREP(GPT_CCMR2_OC4M, 6U)

#define MAX_CH_NUM (4U)

LOG_MODULE_REGISTER(pwm_sf32lb, CONFIG_PWM_LOG_LEVEL);

#ifdef CONFIG_PWM_CAPTURE
/* complementary channel mapping used for pulse/period capture */
static const uint32_t complementary_channel[] = {2, 1, 4, 3};

struct pwm_sf32lb_capture_data {
	pwm_capture_callback_handler_t callback;
	void *user_data;
	uint32_t overflows;
	bool capture_period;
	bool capture_pulse;
	bool continuous;
	uint8_t channel;
	bool enabled;
	uint32_t request_id;
	/* pending result for work handler */
	uint32_t pending_period;
	uint32_t pending_pulse;
	int pending_status;
	uint32_t pending_request_id;
	bool pending;
	bool first_capture;
};

struct pwm_sf32lb_data {
	struct pwm_sf32lb_capture_data capture;
	struct k_work work;
	const struct device *dev;
};
static void pwm_sf32lb_work_handler(struct k_work *work);

#else
struct pwm_sf32lb_data { int dummy; };
#endif

struct pwm_sf32lb_config {
	uintptr_t base;
	const struct pinctrl_dev_config *pcfg;
	struct sf32lb_clock_dt_spec clock;
	uint16_t prescaler;
#ifdef CONFIG_PWM_CAPTURE
	void (*irq_config_func)(const struct device *dev);
#endif
};

static int pwm_sf32lb_set_cycles(const struct device *dev, uint32_t channel, uint32_t period_cycles,
				 uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct pwm_sf32lb_config *config = dev->config;
	uint8_t pos;

	pos = channel * 4U;

	if (channel >= MAX_CH_NUM) {
		LOG_ERR("Invalid PWM channel: %u. Must be 0-3.", channel);
		return -EINVAL;
	}

	LOG_DBG("Setting PWM period_cycles: %d, pulse_cycles: %d", period_cycles, pulse_cycles);

	if (pulse_cycles > period_cycles) {
		LOG_ERR("PWM pulse exceeds period: %u > %u", pulse_cycles, period_cycles);
		return -ENOTSUP;
	}

	if (period_cycles == 0U) {
		sys_clear_bit(config->base + GPT_CCER, pos);
		return 0;
	}

	sys_clear_bit(config->base + GPT_CCER, pos);
	sys_clear_bits(config->base + GPT_CCER, GPT_CCER_CC1P << pos);

	if (flags & PWM_POLARITY_INVERTED) {
		sys_set_bits(config->base + GPT_CCER, GPT_CCER_CC1P << pos);
	}

	sys_write32(period_cycles - 1, config->base + GPT_ARR);
	sys_write32(pulse_cycles, config->base + CCRX(channel));

	switch (channel) {
	case 0:
		sys_clear_bits(config->base + GPT_CCMRX(channel), GPT_CCMR1_OC1M);
		sys_set_bits(config->base + GPT_CCMRX(channel), GPT_CCMR1_OC1PE);
		sys_set_bits(config->base + GPT_CCMRX(channel), GPT_CCMR1_OC1M_PWM1);
		break;
	case 1:
		sys_clear_bits(config->base + GPT_CCMRX(channel), GPT_CCMR1_OC2M);
		sys_set_bits(config->base + GPT_CCMRX(channel), GPT_CCMR1_OC2PE);
		sys_set_bits(config->base + GPT_CCMRX(channel), GPT_CCMR1_OC2M_PWM1);
		break;
	case 2:
		sys_clear_bits(config->base + GPT_CCMRX(channel), GPT_CCMR2_OC3M);
		sys_set_bits(config->base + GPT_CCMRX(channel), GPT_CCMR2_OC3PE);
		sys_set_bits(config->base + GPT_CCMRX(channel), GPT_CCMR2_OC3M_PWM1);
		break;
	case 3:
		sys_clear_bits(config->base + GPT_CCMRX(channel), GPT_CCMR2_OC4M);
		sys_set_bits(config->base + GPT_CCMRX(channel), GPT_CCMR2_OC4PE);
		sys_set_bits(config->base + GPT_CCMRX(channel), GPT_CCMR2_OC4M_PWM1);
		break;
	default:
		return -EINVAL;
	}

	/*
	 * OCxPE (CCR preload) is enabled above, so the new CCR only takes effect
	 * at the next update event. Generate an update now (UG) so the new
	 * period/pulse applies immediately instead of after up to one full
	 * counter period. This is required for the pwm_gpio_loopback test,
	 * which samples the output right after pwm_set().
	 */
	sys_set_bit(config->base + GPT_EGR, GPT_EGR_UG_Pos);

	sys_set_bit(config->base + GPT_CCER, pos);

	return 0;
}

static int pwm_sf32lb_get_cycles_per_sec(const struct device *dev, uint32_t channel,
					 uint64_t *cycles)
{
	const struct pwm_sf32lb_config *config = dev->config;
	uint32_t clock_freq;
	uint32_t prescaler;
	int ret;

	if (channel >= MAX_CH_NUM) {
		LOG_ERR("Invalid PWM channel: %u. Must be 0-3.", channel);
		return -EINVAL;
	}

	ret = sf32lb_clock_control_get_rate_dt(&config->clock, &clock_freq);
	if (ret < 0) {
		return ret;
	}

	prescaler = sys_read32(config->base + GPT_PSC);
	*cycles = (uint64_t)(clock_freq / (prescaler + 1U));

	return ret;
}

static int pwm_sf32lb_init(const struct device *dev)
{
	const struct pwm_sf32lb_config *config = dev->config;
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
		LOG_ERR("Failed to configure pins");
		return ret;
	}

	sys_write32(config->prescaler, config->base + GPT_PSC);
	sys_write32(UINT32_MAX, config->base + GPT_ARR);
	sys_set_bit(config->base + GPT_EGR, GPT_EGR_UG_Pos);

	sys_set_bit(config->base + GPT_CR1, GPT_CR1_CEN_Pos);

#ifdef CONFIG_PWM_CAPTURE
	config->irq_config_func(dev);

	struct pwm_sf32lb_data *data = dev->data;
	data->dev = dev;
	k_work_init(&data->work, pwm_sf32lb_work_handler);
#endif /* CONFIG_PWM_CAPTURE */
	return ret;
}

#ifdef CONFIG_PWM_CAPTURE
static void pwm_sf32lb_isr(const struct device *dev);

#define IRQ_CONFIG_FUNC(n)                                                      \
	static void pwm_sf32lb_irq_config_func_##n(const struct device *dev)        \
	{                                                                          \
		IRQ_CONNECT(DT_IRQN(DT_INST_PARENT(n)), DT_IRQ(DT_INST_PARENT(n), priority), \
				pwm_sf32lb_isr, DEVICE_DT_INST_GET(n), 0);                 \
		irq_enable(DT_IRQN(DT_INST_PARENT(n)));                                 \
	}
#define CAPTURE_INIT(n) .irq_config_func = pwm_sf32lb_irq_config_func_##n,
#else
#define IRQ_CONFIG_FUNC(n)
#define CAPTURE_INIT(n)
#endif

#ifdef CONFIG_PWM_CAPTURE

static void set_channel_ccxs(const struct device *dev, uint32_t channel, uint32_t ccxs_val)
{
	const struct pwm_sf32lb_config *config = dev->config;

	switch (channel) {
	case 0:
		sys_clear_bits(config->base + GPT_CCMRX(channel), GPT_CCMR1_OC1M);
		sys_clear_bits(config->base + GPT_CCMRX(channel), GPT_CCMR1_CC1S);
		sys_set_bits(config->base + GPT_CCMRX(channel), FIELD_PREP(GPT_CCMR1_CC1S, ccxs_val));
		break;
	case 1:
		sys_clear_bits(config->base + GPT_CCMRX(channel), GPT_CCMR1_OC2M);
		sys_clear_bits(config->base + GPT_CCMRX(channel), GPT_CCMR1_CC2S);
		sys_set_bits(config->base + GPT_CCMRX(channel), FIELD_PREP(GPT_CCMR1_CC2S, ccxs_val));
		break;
	case 2:
		sys_clear_bits(config->base + GPT_CCMRX(channel), GPT_CCMR2_OC3M);
		sys_clear_bits(config->base + GPT_CCMRX(channel), GPT_CCMR2_CC3S);
		sys_set_bits(config->base + GPT_CCMRX(channel), FIELD_PREP(GPT_CCMR2_CC3S, ccxs_val));
		break;
	case 3:
		sys_clear_bits(config->base + GPT_CCMRX(channel), GPT_CCMR2_OC4M);
		sys_clear_bits(config->base + GPT_CCMRX(channel), GPT_CCMR2_CC4S);
		sys_set_bits(config->base + GPT_CCMRX(channel), FIELD_PREP(GPT_CCMR2_CC4S, ccxs_val));
		break;
	default:
		break;
	}
}

static void set_channel_polarity(const struct device *dev, uint32_t channel, bool inverted)
{
	const struct pwm_sf32lb_config *config = dev->config;
	uint32_t pos = channel * 4U;

	sys_clear_bits(config->base + GPT_CCER, GPT_CCER_CC1P << pos);
	if (inverted) {
		sys_set_bits(config->base + GPT_CCER, GPT_CCER_CC1P << pos);
	}
}

static void init_capture_channels(const struct device *dev, uint32_t channel,
				pwm_flags_t flags)
{
	const struct pwm_sf32lb_config *config = dev->config;
	bool is_inverted = (flags & PWM_POLARITY_MASK) == PWM_POLARITY_INVERTED;
	uint32_t comp_idx;

	if (channel >= MAX_CH_NUM) {
		return;
	}

	comp_idx = complementary_channel[channel] - 1U;

	set_channel_ccxs(dev, channel, 1U);
	set_channel_polarity(dev, channel, is_inverted);

	if (comp_idx < MAX_CH_NUM && comp_idx != channel) {
		set_channel_ccxs(dev, comp_idx, 2U);
		set_channel_polarity(dev, comp_idx, !is_inverted);
	}

	switch (channel) {
	case 0:
		sys_clear_bits(config->base + GPT_SMCR, GPT_SMCR_TS);
		sys_set_bits(config->base + GPT_SMCR, GPT_SMCR_TS_TI1FP1);
		sys_clear_bits(config->base + GPT_SMCR, GPT_SMCR_SMS);
		sys_set_bits(config->base + GPT_SMCR, GPT_SLAVEMODE_RESET);
		break;
	case 1:
		sys_clear_bits(config->base + GPT_SMCR, GPT_SMCR_TS);
		sys_set_bits(config->base + GPT_SMCR, GPT_SMCR_TS_TI2FP2);
		sys_clear_bits(config->base + GPT_SMCR, GPT_SMCR_SMS);
		sys_set_bits(config->base + GPT_SMCR, GPT_SLAVEMODE_RESET);
		break;
	default:
		break;
	}
}

static void pwm_sf32lb_work_handler(struct k_work *work)
{
	struct pwm_sf32lb_data *data = CONTAINER_OF(work, struct pwm_sf32lb_data, work);
	struct pwm_sf32lb_capture_data *cpt = &data->capture;

	if (!cpt->pending) {
		return;
	}

	uint32_t period = cpt->pending_period;
	uint32_t pulse = cpt->pending_pulse;
	int status = cpt->pending_status;
	uint8_t channel = cpt->channel;

	cpt->pending = false;

	pwm_capture_callback_handler_t cb = cpt->callback;
	void *ud = cpt->user_data;
	uint32_t request_id = cpt->request_id;
	const struct device *pdev = data->dev;

	if (cpt->pending_request_id != request_id) {
		return;
	}

	if (cb) {
		cb(pdev, channel,
		   cpt->capture_period ? period : 0u,
		   cpt->capture_pulse ? pulse : 0u,
		   status, ud);
	}

	if (!cpt->continuous) {
		unsigned int key = irq_lock();
		cpt->callback = NULL;
		cpt->user_data = NULL;
		irq_unlock(key);
	}
}

static int pwm_sf32lb_configure_capture(const struct device *dev,
				       uint32_t channel, pwm_flags_t flags,
				       pwm_capture_callback_handler_t cb,
				       void *user_data)
{
	struct pwm_sf32lb_data *data = dev->data;
	struct pwm_sf32lb_capture_data *cpt = &data->capture;

	if (channel >= 2U) {
		LOG_ERR("PWM capture only supported on channel 0 or 1 "
			"(no HW reset-trigger source for channel %u)", channel);
		return -ENOTSUP;
	}

	if (cpt->enabled) {
		return -EBUSY;
	}

	if (!(flags & PWM_CAPTURE_TYPE_MASK)) {
		LOG_ERR("No PWM capture type specified");
		return -EINVAL;
	}

	unsigned int key = irq_lock();
	cpt->callback = cb;
	cpt->user_data = user_data;
	cpt->capture_period = (flags & PWM_CAPTURE_TYPE_PERIOD) ? true : false;
	cpt->capture_pulse = (flags & PWM_CAPTURE_TYPE_PULSE) ? true : false;
	cpt->continuous = (flags & PWM_CAPTURE_MODE_CONTINUOUS) ? true : false;
	cpt->request_id++;
	irq_unlock(key);

	init_capture_channels(dev, channel, flags);

	return 0;
}

static int pwm_sf32lb_enable_capture(const struct device *dev, uint32_t channel)
{
	const struct pwm_sf32lb_config *config = dev->config;
	struct pwm_sf32lb_data *data = dev->data;
	struct pwm_sf32lb_capture_data *cpt = &data->capture;
	uint32_t pos = channel * 4U;
	uint32_t comp_idx;
	uint32_t comp_pos;

	if (channel >= 2U) {
		LOG_ERR("PWM input capture only supports CH1/CH2");
		return -ENOTSUP;
	}

	comp_idx = complementary_channel[channel] - 1U;
	comp_pos = comp_idx * 4U;

	if (!cpt->callback) {
		LOG_ERR("PWM capture not configured");
		return -EINVAL;
	}

	if (cpt->enabled) {
		return -EBUSY;
	}

	cpt->channel = channel;
	cpt->overflows = 0u;
	cpt->first_capture = true;

	unsigned int key = irq_lock();
	cpt->enabled = true;
	irq_unlock(key);

	sys_clear_bits(config->base + GPT_SR,
		       GPT_SR_UIF | GPT_SR_CC1IF | GPT_SR_CC2IF |
		       GPT_SR_CC3IF | GPT_SR_CC4IF);
	if (comp_idx < MAX_CH_NUM && comp_idx != channel) {
		sys_clear_bits(config->base + GPT_SR, GPT_SR_CC1IF << comp_idx);
	}

	sys_set_bits(config->base + GPT_DIER, GPT_DIER_CC1IE << channel);
	sys_set_bit(config->base + GPT_DIER, GPT_DIER_UIE_Pos);

	sys_set_bit(config->base + GPT_CCER, pos);

	if (comp_idx < MAX_CH_NUM && comp_idx != channel) {
		sys_set_bit(config->base + GPT_CCER, comp_pos);
	}
	return 0;
}

static int pwm_sf32lb_disable_capture(const struct device *dev, uint32_t channel)
{
	const struct pwm_sf32lb_config *config = dev->config;
	struct pwm_sf32lb_data *data = dev->data;
	struct pwm_sf32lb_capture_data *cpt = &data->capture;
	uint32_t pos = channel * 4U;
	uint32_t comp_idx;
	uint32_t comp_pos;

	if (channel >= MAX_CH_NUM) {
		LOG_ERR("PWM capture only exists on channels 0..3");
		return -ENOTSUP;
	}

	sys_clear_bits(config->base + GPT_DIER, GPT_DIER_CC1IE << channel);
	sys_clear_bit(config->base + GPT_DIER, GPT_DIER_UIE_Pos);
	sys_clear_bit(config->base + GPT_CCER, pos);
	sys_clear_bits(config->base + GPT_SR, GPT_SR_CC1IF << channel);

	comp_idx = complementary_channel[channel] - 1U;
	if (comp_idx < MAX_CH_NUM && comp_idx != channel) {
		comp_pos = comp_idx * 4U;
		sys_clear_bit(config->base + GPT_CCER, comp_pos);
		sys_clear_bits(config->base + GPT_SR, GPT_SR_CC1IF << comp_idx);
	}

	if (channel < 2U) {
		sys_clear_bits(config->base + GPT_SMCR, GPT_SMCR_SMS);
		sys_clear_bits(config->base + GPT_SMCR, GPT_SMCR_TS);
	}

	unsigned int key = irq_lock();
	cpt->enabled = false;
	irq_unlock(key);
	return 0;
}

static void pwm_sf32lb_isr(const struct device *dev)
{
	const struct pwm_sf32lb_config *config = dev->config;
	struct pwm_sf32lb_data *data = dev->data;
	struct pwm_sf32lb_capture_data *cpt = &data->capture;
	uint8_t channel = cpt->channel;
	int status = 0;
	uint32_t sr = sys_read32(config->base + GPT_SR);

	if (channel >= MAX_CH_NUM) {
		/* invalid channel index, clear interrupts and bail */
		sys_clear_bits(config->base + GPT_SR, GPT_SR_CC1IF);
		return;
	}

	/* Update (overflow) */
	if (sr & GPT_SR_UIF) {
		sys_clear_bit(config->base + GPT_SR, GPT_SR_UIF_Pos);
		cpt->overflows++;
	}

	/* Capture event for channel */
	uint32_t capture_flag = GPT_SR_CC1IF << channel;
	if (sr & capture_flag) {
		uint32_t period = sys_read32(config->base + CCRX(channel));
		uint32_t comp_idx = complementary_channel[channel] - 1U;
		uint32_t pulse = sys_read32(config->base + CCRX(comp_idx));
		sys_clear_bits(config->base + GPT_SR, capture_flag);

		if (cpt->first_capture) {
			cpt->first_capture = false;
			cpt->overflows = 0u;
			return;
		}
		if (cpt->overflows) {
			status = -ERANGE;
		}

		if (!cpt->continuous) {
			sys_clear_bit(config->base + GPT_CCER, channel * 4U);

			sys_clear_bits(config->base + GPT_DIER, GPT_DIER_CC1IE << channel);

			cpt->enabled = false;
		} else {
			cpt->overflows = 0u;
		}

		cpt->pending_period = period;
		cpt->pending_pulse = pulse;
		cpt->pending_status = status;
		cpt->pending_request_id = cpt->request_id;
		cpt->pending = true;
		k_work_submit(&data->work);
	}
}

#endif

static DEVICE_API(pwm, pwm_sf32lb_driver_api) = {
	.set_cycles = pwm_sf32lb_set_cycles,
	.get_cycles_per_sec = pwm_sf32lb_get_cycles_per_sec,
#ifdef CONFIG_PWM_CAPTURE
	.configure_capture = pwm_sf32lb_configure_capture,
	.enable_capture = pwm_sf32lb_enable_capture,
	.disable_capture = pwm_sf32lb_disable_capture,
#endif /* CONFIG_PWM_CAPTURE */
};

#define PWM_SF32LB_DEFINE(n)                                                                       \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	IRQ_CONFIG_FUNC(n)																			\
	static struct pwm_sf32lb_data pwm_sf32lb_data_##n;                                       \
	static const struct pwm_sf32lb_config pwm_sf32lb_config_##n = {                            \
		.base = DT_REG_ADDR(DT_INST_PARENT(n)),                                            \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.clock = SF32LB_CLOCK_DT_INST_PARENT_SPEC_GET(n),                                  \
		.prescaler = DT_PROP(DT_INST_PARENT(n), sifli_prescaler),                          \
		CAPTURE_INIT(n)																	\
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, pwm_sf32lb_init, NULL, &pwm_sf32lb_data_##n,                                      \
			      &pwm_sf32lb_config_##n, POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,       \
			      &pwm_sf32lb_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_SF32LB_DEFINE)
