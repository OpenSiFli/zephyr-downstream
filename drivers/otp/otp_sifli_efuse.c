/*
 * Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_efuse

#include <stddef.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/otp.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/minmax.h>
#include <zephyr/sys/util.h>

#include <ll_efuse.h>
#include <ll_pmuc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(otp_sifli_efuse, CONFIG_OTP_LOG_LEVEL);

/* Timeout for eFuse read operation */
#define EFUSE_READ_TIMEOUT_US 10000

struct otp_sifli_efuse_config {
	EFUSEC_TypeDef *efuse;
	uintptr_t pmuc_base;
	uint8_t *cache;
	const uint32_t *bank_offsets;
	size_t bank_size;
	size_t bank_num;
};

struct otp_sifli_efuse_data {
	bool cached;
	struct k_mutex lock;
};

/**
 * @brief Read a single bank from eFuse hardware
 *
 * @param config Device configuration
 * @param bank Bank number
 * @param data Output buffer (must be at least bank_size bytes)
 * @return 0 on success, negative errno on failure
 */
static int efuse_read_bank(const struct otp_sifli_efuse_config *config, uint8_t bank, uint8_t *data)
{
	uint32_t org_vout;
	uint32_t new_vout;
	uint32_t timeout = 0;
	uint32_t val;
	uint32_t bank_data_offset;

	if (bank >= config->bank_num) {
		return -EINVAL;
	}

	/* Adjust HPSYS LDO voltage before reading */
	org_vout = ll_pmuc_get_hpsys_vout((PMUC_TypeDef *)config->pmuc_base);
	new_vout = clamp(org_vout + 3, 0xe, 0xf);
	ll_pmuc_set_hpsys_vout((PMUC_TypeDef *)config->pmuc_base, new_vout);
	k_busy_wait(20);

	/* Select bank and set READ mode (MODE=0), see manual 13.3.4 */
	ll_efuse_set_bank(config->efuse, bank);
	ll_efuse_set_mode(config->efuse, LL_EFUSE_MODE_READ);

	/* Start read operation */
	ll_efuse_start(config->efuse);

	/* Wait for read completion */
	while (!ll_efuse_is_done(config->efuse)) {
		k_busy_wait(1);
		timeout++;
		if (timeout > EFUSE_READ_TIMEOUT_US) {
			LOG_ERR("eFuse read timeout for bank %u", bank);
			ll_pmuc_set_hpsys_vout((PMUC_TypeDef *)config->pmuc_base, org_vout);
			return -ETIMEDOUT;
		}
	}

	/* Clear done flag */
	ll_efuse_clear_done(config->efuse);

	/* Get bank data register offset from config */
	bank_data_offset = config->bank_offsets[bank];

	/* Read bank data */
	for (size_t i = 0; i < config->bank_size / sizeof(uint32_t); i++) {
		val = ll_efuse_read_word_at(config->efuse, bank_data_offset + i * sizeof(uint32_t));
		sys_put_le32(val, &data[i * sizeof(uint32_t)]);
	}

	/* Restore original LDO voltage */
	ll_pmuc_set_hpsys_vout((PMUC_TypeDef *)config->pmuc_base, org_vout);

	return 0;
}

/**
 * @brief Load all eFuse banks into cache
 */
static int efuse_load_cache(const struct device *dev)
{
	const struct otp_sifli_efuse_config *config = dev->config;
	struct otp_sifli_efuse_data *data = dev->data;
	int ret;

	for (size_t bank = 0; bank < config->bank_num; bank++) {
		ret = efuse_read_bank(config, bank, &config->cache[bank * config->bank_size]);
		if (ret < 0) {
			LOG_ERR("Failed to read eFuse bank %zu: %d", bank, ret);
			return ret;
		}
	}

	data->cached = true;
	LOG_DBG("eFuse cache loaded successfully");

	return 0;
}

static int otp_sifli_efuse_read(const struct device *dev, off_t offset, void *buf, size_t len)
{
	const struct otp_sifli_efuse_config *config = dev->config;
	struct otp_sifli_efuse_data *data = dev->data;
	size_t total_size = config->bank_size * config->bank_num;
	int ret = 0;

	if (offset < 0 || (offset + len) > total_size) {
		return -EINVAL;
	}

	if (len == 0) {
		return 0;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* Load cache if not already cached */
	if (!data->cached) {
		ret = efuse_load_cache(dev);
		if (ret < 0) {
			k_mutex_unlock(&data->lock);
			return ret;
		}
	}

	/* Copy from cache to output buffer */
	memcpy(buf, &config->cache[offset], len);

	k_mutex_unlock(&data->lock);

	return 0;
}

static int otp_sifli_efuse_init(const struct device *dev)
{
	struct otp_sifli_efuse_data *data = dev->data;
	int ret;

	k_mutex_init(&data->lock);
	data->cached = false;

	/* Pre-load cache at initialization for better read performance */
	ret = efuse_load_cache(dev);
	if (ret < 0) {
		LOG_WRN("Failed to pre-load eFuse cache: %d", ret);
		/* Non-fatal: cache will be loaded on first read */
	}

	LOG_INF("SiFli eFuse OTP driver initialized");

	return 0;
}

static DEVICE_API(otp, otp_sifli_efuse_api) = {
	.read = otp_sifli_efuse_read,
};

#define OTP_SIFLI_EFUSE_BANK_SIZE(n) DT_INST_PROP(n, sifli_bank_size)
#define OTP_SIFLI_EFUSE_BANK_NUM(n)  DT_INST_PROP_LEN(n, sifli_bank_offsets)

#define OTP_SIFLI_EFUSE_INIT(n)                                                                    \
	static uint8_t otp_sifli_efuse_cache_##n[OTP_SIFLI_EFUSE_BANK_SIZE(n) *                    \
						 OTP_SIFLI_EFUSE_BANK_NUM(n)];                     \
                                                                                                   \
	static const uint32_t otp_sifli_efuse_bank_offsets_##n[] =                                 \
		DT_INST_PROP(n, sifli_bank_offsets);                                               \
                                                                                                   \
	static struct otp_sifli_efuse_data otp_sifli_efuse_data_##n;                               \
                                                                                                   \
	static const struct otp_sifli_efuse_config otp_sifli_efuse_config_##n = {                  \
		.efuse = (EFUSEC_TypeDef *)DT_INST_REG_ADDR(n),                                                       \
		.pmuc_base = DT_REG_ADDR(DT_INST_PHANDLE(n, sifli_pmuc)),                          \
		.cache = otp_sifli_efuse_cache_##n,                                                \
		.bank_offsets = otp_sifli_efuse_bank_offsets_##n,                                  \
		.bank_size = OTP_SIFLI_EFUSE_BANK_SIZE(n),                                         \
		.bank_num = OTP_SIFLI_EFUSE_BANK_NUM(n),                                           \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, otp_sifli_efuse_init, NULL, &otp_sifli_efuse_data_##n,            \
			      &otp_sifli_efuse_config_##n, POST_KERNEL, CONFIG_OTP_INIT_PRIORITY,  \
			      &otp_sifli_efuse_api);

DT_INST_FOREACH_STATUS_OKAY(OTP_SIFLI_EFUSE_INIT)
