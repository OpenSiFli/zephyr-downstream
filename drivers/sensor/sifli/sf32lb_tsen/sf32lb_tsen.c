/*
 * Copyright (c) 2025, Qingsong Gou <gouqs@hotmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_tsen

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control/sf32lb.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <ll_hpsys_cfg.h>
#include <ll_tsen.h>

LOG_MODULE_REGISTER(sf32lb_tsen, CONFIG_SENSOR_LOG_LEVEL);

struct sf32lb_tsen_config {
	TSEN_TypeDef *tsen;
	HPSYS_CFG_TypeDef *cfg;
	struct sf32lb_clock_dt_spec clock;
};

struct sf32lb_tsen_data {
	struct k_mutex mutex;
	uint32_t last_temp;
};

static int sf32lb_tsen_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct sf32lb_tsen_config *config = dev->config;
	struct sf32lb_tsen_data *data = dev->data;

	k_mutex_lock(&data->mutex, K_FOREVER);

	while (!ll_tsen_get_interrupt_raw(config->tsen)) {
		k_msleep(1);
	}

	data->last_temp = ll_tsen_read_data(config->tsen);

	ll_tsen_clear_interrupt(config->tsen);

	k_mutex_unlock(&data->mutex);

	return 0;
}

static int sf32lb_tsen_channel_get(const struct device *dev, enum sensor_channel chan,
				   struct sensor_value *val)
{
	struct sf32lb_tsen_data *data = dev->data;
	float temp;

	if (chan != SENSOR_CHAN_DIE_TEMP) {
		return -ENOTSUP;
	}

	temp = ((int32_t)data->last_temp + 3000) * 749.2916 / 10100 - 277; /* see manual 8.2.3.2 */

	return sensor_value_from_float(val, temp);
}

static DEVICE_API(sensor, sf32lb_tsen_driver_api) = {
	.sample_fetch = sf32lb_tsen_sample_fetch,
	.channel_get = sf32lb_tsen_channel_get,
};

static int sf32lb_tsen_init(const struct device *dev)
{
	const struct sf32lb_tsen_config *config = dev->config;
	struct sf32lb_tsen_data *data = dev->data;
	int ret;

	if (!sf32lb_clock_is_ready_dt(&config->clock)) {
		return -ENODEV;
	}

	ret = sf32lb_clock_control_on_dt(&config->clock);
	if (ret < 0) {
		return ret;
	}

	ll_cfg_anau_bandgap_enable(config->cfg);

	ll_tsen_assert_reset(config->tsen);
	ll_tsen_enable(config->tsen);
	ll_tsen_power_up(config->tsen);
	ll_tsen_release_reset(config->tsen);
	k_busy_wait(20);
	ll_tsen_start(config->tsen);

	k_mutex_init(&data->mutex);

	return ret;
}

#define SF32LB_TSEN_DEFINE(inst)                                                                   \
	static struct sf32lb_tsen_data sf32lb_tsen_data_##inst;                                    \
	static const struct sf32lb_tsen_config sf32lb_tsen_config_##inst = {                       \
		.tsen = (TSEN_TypeDef *)DT_INST_REG_ADDR(inst),                                       \
		.cfg = (HPSYS_CFG_TypeDef *)DT_REG_ADDR(DT_INST_PHANDLE(inst, sifli_cfg)),           \
		.clock = SF32LB_CLOCK_DT_INST_SPEC_GET(inst),                                      \
	};                                                                                         \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, sf32lb_tsen_init, NULL, &sf32lb_tsen_data_##inst,       \
				     &sf32lb_tsen_config_##inst, POST_KERNEL,                      \
				     CONFIG_SENSOR_INIT_PRIORITY, &sf32lb_tsen_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SF32LB_TSEN_DEFINE)
