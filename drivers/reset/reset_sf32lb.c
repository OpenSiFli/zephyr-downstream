/*
 * Copyright (c) 2025 Qingsong Gou <gouqs@hotmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_rcc_rctl

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/reset.h>

#include <ll_rcc.h>

#define SF32LB_RESET_REG(id) (((id) >> 5U) & 0x1U)
#define SF32LB_RESET_BIT(id) ((id) & 0x1FU)

struct sf32lb_reset_config {
	HPSYS_RCC_TypeDef *rcc;
};

static int sf32lb_reset_status(const struct device *dev, uint32_t id, uint8_t *status)
{
	const struct sf32lb_reset_config *config = dev->config;
	uint32_t mask = BIT(SF32LB_RESET_BIT(id));

	*status = !!ll_rcc_is_module_in_reset(config->rcc, SF32LB_RESET_REG(id), mask);

	return 0;
}

static int sf32lb_reset_line_assert(const struct device *dev, uint32_t id)
{
	const struct sf32lb_reset_config *config = dev->config;
	uint32_t mask = BIT(SF32LB_RESET_BIT(id));

	ll_rcc_reset_module(config->rcc, SF32LB_RESET_REG(id), mask);

	return 0;
}

static int sf32lb_reset_line_deassert(const struct device *dev, uint32_t id)
{
	const struct sf32lb_reset_config *config = dev->config;
	uint32_t mask = BIT(SF32LB_RESET_BIT(id));

	ll_rcc_release_reset(config->rcc, SF32LB_RESET_REG(id), mask);

	return 0;
}

static int sf32lb_reset_line_toggle(const struct device *dev, uint32_t id)
{
	sf32lb_reset_line_assert(dev, id);
	sf32lb_reset_line_deassert(dev, id);

	return 0;
}

static DEVICE_API(reset, sf32lb_reset_api) = {
	.status = sf32lb_reset_status,
	.line_assert = sf32lb_reset_line_assert,
	.line_deassert = sf32lb_reset_line_deassert,
	.line_toggle = sf32lb_reset_line_toggle,
};

static const struct sf32lb_reset_config sf32lb_reset_cfg = {
	.rcc = (HPSYS_RCC_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(0)),
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, &sf32lb_reset_cfg, PRE_KERNEL_1,
		      CONFIG_RESET_INIT_PRIORITY, &sf32lb_reset_api);
