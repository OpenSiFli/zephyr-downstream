/*
 * Copyright (c) 2025, Qingsong Gou <gouqs@hotmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT sifli_sf32lb_wdt

#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>

#include <ll_hpsys_cfg.h>
#include <ll_pmuc.h>
#include <ll_wdt.h>

LOG_MODULE_REGISTER(wdt_sf32lb, CONFIG_WDT_LOG_LEVEL);

#define WDT_CVR0_MAX 0xFFFFFF

/* Assume LRC10 clocks WDT (LRC32 support to be added in the future) */
#define WDT_CLK_KHZ 10

#define WDT_WINDOW_MS_MAX (WDT_CVR0_MAX / WDT_CLK_KHZ)

struct wdt_sf32lb_config {
	WDT_TypeDef *wdt;
	uintptr_t pmuc;
	uintptr_t cfg;
	bool reset_all;
};

struct wdt_sf32lb_data {
	bool timeout_valid;
};

static inline bool wdt_sf32lb_is_enabled(const struct device *dev)
{
	const struct wdt_sf32lb_config *config = dev->config;

	return ll_wdt_is_active(config->wdt) != 0U;
}

static int wdt_sf32lb_setup(const struct device *dev, uint8_t options)
{
	const struct wdt_sf32lb_config *config = dev->config;

	if (options != 0U) {
		LOG_ERR("Options not supported");
		return -ENOTSUP;
	}

	if (wdt_sf32lb_is_enabled(dev)) {
		LOG_ERR("Setup not allowed with watchdog enabled");
		return -EBUSY;
	}

	ll_wdt_feed(config->wdt);

	return 0;
}

static int wdt_sf32lb_disable(const struct device *dev)
{
	const struct wdt_sf32lb_config *config = dev->config;
	struct wdt_sf32lb_data *data = dev->data;

	if (!wdt_sf32lb_is_enabled(dev)) {
		LOG_ERR("Watchdog already disabled");
		return -EFAULT;
	}
	data->timeout_valid = false;
	ll_wdt_disable(config->wdt);

	return 0;
}

static int wdt_sf32lb_install_timeout(const struct device *dev,
				      const struct wdt_timeout_cfg *wdt_cfg)
{
	const struct wdt_sf32lb_config *config = dev->config;
	struct wdt_sf32lb_data *data = dev->data;

	if (wdt_sf32lb_is_enabled(dev)) {
		LOG_ERR("Timeout install not allowed with watchdog enabled");
		return -EBUSY;
	}

	if (wdt_cfg->flags != WDT_FLAG_RESET_SOC) {
		LOG_ERR("Only SoC reset supported");
		return -ENOTSUP;
	}

	if (wdt_cfg->callback != NULL) {
		LOG_ERR("Callback not supported");
		return -ENOTSUP;
	}

	if (wdt_cfg->window.min != 0U) {
		LOG_ERR("Window mode not supported!");
		return -ENOTSUP;
	};

	if (wdt_cfg->window.max > WDT_WINDOW_MS_MAX) {
		return -EINVAL;
	}

	data->timeout_valid = true;

	ll_wdt_set_timeout1(config->wdt, wdt_cfg->window.max * WDT_CLK_KHZ);

	return 0;
}

static int wdt_sf32lb_feed(const struct device *dev, int channel_id)
{
	const struct wdt_sf32lb_config *config = dev->config;
	struct wdt_sf32lb_data *data = dev->data;

	if (!data->timeout_valid) {
		LOG_ERR("No valid timeout installed");
		return -EINVAL;
	}

	ll_wdt_feed(config->wdt);

	return 0;
}

static DEVICE_API(wdt, wdt_sf32lb_api) = {
	.setup = wdt_sf32lb_setup,
	.disable = wdt_sf32lb_disable,
	.install_timeout = wdt_sf32lb_install_timeout,
	.feed = wdt_sf32lb_feed,
};

static int wdt_sf32lb_init(const struct device *dev)
{
	const struct wdt_sf32lb_config *config = dev->config;

	ll_wdt_set_response_mode(config->wdt, LL_WDT_RESPONSE_RESET);

	ll_pmuc_enable_wakeup_source((PMUC_TypeDef *)config->pmuc, LL_PMUC_WKUP_WDT1);

	if (config->reset_all) {
		ll_cfg_wdt1_reboot_set((HPSYS_CFG_TypeDef *)config->cfg);
	} else {
		ll_cfg_wdt1_reboot_clear((HPSYS_CFG_TypeDef *)config->cfg);
	}

	return 0;
}

#define WDT_SF32LB_INIT(index)                                                                     \
	static const struct wdt_sf32lb_config wdt_sf32lb_config_##index = {                        \
		.wdt = (WDT_TypeDef *)DT_INST_REG_ADDR(index),                                                   \
		.pmuc = DT_REG_ADDR(DT_INST_PHANDLE(index, sifli_pmuc)),                           \
		.cfg = DT_REG_ADDR(DT_INST_PHANDLE(index, sifli_cfg)),                             \
		.reset_all = DT_INST_PROP(index, sifli_reset_all),                                 \
	};                                                                                         \
	static struct wdt_sf32lb_data wdt_sf32lb_data_##index;                                     \
	DEVICE_DT_INST_DEFINE(index, wdt_sf32lb_init, NULL, &wdt_sf32lb_data_##index,              \
			      &wdt_sf32lb_config_##index, POST_KERNEL,                             \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &wdt_sf32lb_api);

DT_INST_FOREACH_STATUS_OKAY(WDT_SF32LB_INIT)
