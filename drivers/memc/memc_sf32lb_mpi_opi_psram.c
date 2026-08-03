/*
 * Copyright (c) 2025 SiFli Technologies Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_mpi_opi_psram

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/sf32lb.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <ll_mpi.h>

LOG_MODULE_REGISTER(memc_sf32lb_mpi_opi_psram, CONFIG_MEMC_LOG_LEVEL);

/* MPI register offsets */

/* OPI PSRAM commands */
#define OPSRAM_CMD_READ    0x00U
#define OPSRAM_CMD_WRITE   0x80U
#define OPSRAM_CMD_MRREAD  0x40U
#define OPSRAM_CMD_MRWRITE 0xC0U
#define OPSRAM_CMD_RESET   0xFFU

/*
 * Use CMSIS definitions from register.h where available.
 * These provide MPI_xxx_Pos, MPI_xxx_Msk, and MPI_xxx convenience macros.
 *
 * Naming conventions from CMSIS:
 * - DCR: RBSIZE (not RSIZE), DQSE (not DQSEN), CSLMAX/CSLMIN (not CSMAX/CSMIN)
 * - MISCR: RXCLKDLY (not RXDLY), RXCLKINV (not RXINV)
 * - APM32CR: TCPHR/TCPHW (not RDCYC/WRCYC)
 */

/* Mode values for CCR */
#define CCR_MODE_NONE   0U
#define CCR_MODE_SINGLE 1U
#define CCR_MODE_DUAL   2U
#define CCR_MODE_QUAD   3U
#define CCR_MODE_OCT    7U

/* Address size values */
#define CCR_ADSIZE_8    0U
#define CCR_ADSIZE_16   1U
#define CCR_ADSIZE_24   2U
#define CCR_ADSIZE_32   3U

struct memc_sf32lb_mpi_opi_psram_config {
	MPI_TypeDef *mpi;
	uintptr_t psram_base;
	uint32_t size;
	struct sf32lb_clock_dt_spec clock;
	const struct pinctrl_dev_config *pcfg;
	const struct device *power_supply;
};

struct memc_sf32lb_mpi_opi_psram_data {
	uint8_t sck_delay;
	uint8_t dqs_delay;
	uint8_t rd_latency;
	uint8_t wr_latency;
};

static void mpi_delay_us(uint32_t us)
{
	k_busy_wait(us);
}

static int mpi_wait_complete(MPI_TypeDef *mpi)
{
	int retries = 10000;

	while (!ll_mpi_get_transfer_complete_flag(mpi)) {
		if (--retries <= 0) {
			LOG_ERR("MPI transfer timeout");
			return -ETIMEDOUT;
		}
	}
	ll_mpi_clear_transfer_complete_flag(mpi);
	return 0;
}

static void mpi_qspi_init(const struct device *dev)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	MPI_TypeDef *mpi = cfg->mpi;

	ll_mpi_set_timing(mpi, 0xFFU);
	ll_mpi_set_comm_interval(mpi, 0x50005000U);
	ll_mpi_set_alt_bytes(mpi, LL_MPI_CS_1, 0xFFU);
	ll_mpi_set_ahb_alt_bytes(mpi, 0xFFU);
}

static int mpi_calibrate_delay(const struct device *dev)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	struct memc_sf32lb_mpi_opi_psram_data *data = dev->data;
	MPI_TypeDef *mpi = cfg->mpi;
	uint32_t delay;

	/* Set prescaler to 2 for calibration */
	ll_mpi_set_clock_div(mpi, 2U);

	/* Clear SCK inversion */
	ll_mpi_set_sck_invert(mpi, 0U);

	/* Enable calibration */
	ll_mpi_calibration_enable(mpi);

	/* Wait for calibration to complete (with timeout) */
	mpi_delay_us(20);
	{
		int retries = 1000;

		while (!ll_mpi_is_calibration_done(mpi)) {
			if (--retries <= 0) {
				ll_mpi_calibration_disable(mpi);
				LOG_ERR("MPI calibration timeout (DLL2 not locked?)");
				return -ETIMEDOUT;
			}
			mpi_delay_us(1);
		}
	}

	/* Read delay value */
	delay = ll_mpi_get_calibration_delay(mpi);

	/* Disable calibration */
	ll_mpi_calibration_disable(mpi);

	if (delay < 4) {
		LOG_ERR("MPI calibration result too small: %u", delay);
		return -EINVAL;
	}

	/* Calculate SCK and DQS delays (SF32LB52X specific) */
	data->sck_delay = (uint8_t)(delay - 1);
	data->dqs_delay = (uint8_t)(delay - 4);

	/* Restore prescaler to 1 */
	ll_mpi_set_clock_div(mpi, 1U);

	LOG_DBG("Calibration: delay=%u, sck=%u, dqs=%u", delay, data->sck_delay, data->dqs_delay);

	return 0;
}

static void mpi_set_delays(const struct device *dev)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	struct memc_sf32lb_mpi_opi_psram_data *data = dev->data;
	MPI_TypeDef *mpi = cfg->mpi;

	ll_mpi_set_sck_delay(mpi, data->sck_delay);
	ll_mpi_set_dqs_delay(mpi, data->dqs_delay);
}

static void mpi_manual_cmd(MPI_TypeDef *mpi, bool is_write, uint8_t dmode, uint8_t dcyc,
			   uint8_t abmode, uint8_t absize, uint8_t adsize, uint8_t admode,
			   uint8_t imode)
{
	uint32_t ccr1 = 0U;

	ccr1 |= FIELD_PREP(MPI_CCR1_IMODE_Msk, imode);
	ccr1 |= FIELD_PREP(MPI_CCR1_ADMODE_Msk, admode);
	ccr1 |= FIELD_PREP(MPI_CCR1_ADSIZE_Msk, adsize);
	ccr1 |= FIELD_PREP(MPI_CCR1_ABMODE_Msk, abmode);
	ccr1 |= FIELD_PREP(MPI_CCR1_ABSIZE_Msk, absize);
	ccr1 |= FIELD_PREP(MPI_CCR1_DCYC_Msk, dcyc);
	ccr1 |= FIELD_PREP(MPI_CCR1_DMODE_Msk, dmode);
	if (is_write) {
		ccr1 |= MPI_CCR1_FMODE_Msk;
	}

	ll_mpi_write_command_config(mpi, LL_MPI_CS_1, ccr1);
}

static int mpi_psram_reset(const struct device *dev)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	MPI_TypeDef *mpi = cfg->mpi;
	int ret;

	/* Configure reset command: write mode, no data, 1-byte AB, 32-bit addr, OPI mode */
	mpi_manual_cmd(mpi, true, CCR_MODE_NONE, 0, CCR_MODE_OCT, 0, CCR_ADSIZE_32, CCR_MODE_OCT,
		       CCR_MODE_OCT);

	/* Send reset command */
	ll_mpi_set_address(mpi, LL_MPI_CS_1, 0U);
	ll_mpi_set_command_byte(mpi, LL_MPI_CS_1, OPSRAM_CMD_RESET);
	ret = mpi_wait_complete(mpi);
	if (ret < 0) {
		return ret;
	}

	/* Wait for PSRAM to reset */
	mpi_delay_us(3);
	return 0;
}

static int mpi_mr_write(const struct device *dev, uint8_t addr, uint8_t value)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	MPI_TypeDef *mpi = cfg->mpi;

	/* Configure MR write command: write mode, OPI data, no dummy, 32-bit addr, OPI mode */
	mpi_manual_cmd(mpi, true, CCR_MODE_OCT, 0, CCR_MODE_NONE, 0, CCR_ADSIZE_32, CCR_MODE_OCT,
		       CCR_MODE_OCT);

	/* Set data length to 2 bytes */
	ll_mpi_set_data_length(mpi, LL_MPI_CS_1, 2U);

	/* Write data to FIFO */
	ll_mpi_write_data(mpi, (uint32_t)value);

	/* Send command */
	ll_mpi_set_address(mpi, LL_MPI_CS_1, addr);
	ll_mpi_set_command_byte(mpi, LL_MPI_CS_1, OPSRAM_CMD_MRWRITE);
	return mpi_wait_complete(mpi);
}

static __maybe_unused uint8_t mpi_mr_read(const struct device *dev, uint8_t addr)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	struct memc_sf32lb_mpi_opi_psram_data *data = dev->data;
	MPI_TypeDef *mpi = cfg->mpi;
	uint8_t rdcyc = data->rd_latency;

	/* Configure MR read command: read mode, OPI data, dummy cycles, 32-bit addr, OPI mode */
	mpi_manual_cmd(mpi, false, CCR_MODE_OCT, rdcyc - 1, CCR_MODE_NONE, 0, CCR_ADSIZE_32,
		       CCR_MODE_OCT, CCR_MODE_OCT);

	/* Set data length to 2 bytes */
	ll_mpi_set_data_length(mpi, LL_MPI_CS_1, 2U);

	/* Send command */
	ll_mpi_set_address(mpi, LL_MPI_CS_1, addr);
	ll_mpi_set_command_byte(mpi, LL_MPI_CS_1, OPSRAM_CMD_MRREAD);
	if (mpi_wait_complete(mpi) < 0) {
		return 0;
	}

	return (uint8_t)(ll_mpi_read_data(mpi) & 0xFFU);
}

static void mpi_set_fixlat(const struct device *dev, bool fix, uint8_t r_lat, uint8_t w_lat)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	MPI_TypeDef *mpi = cfg->mpi;
	uint8_t mr0, mr4;
	uint8_t rlat_arr[8] = {0, 0, 0, 0, 1, 2, 3, 4};
	uint8_t wlat_arr[8] = {0, 0, 0, 0, 4, 2, 6, 1};

	/* Set fixed latency in DCR */
	ll_mpi_set_fixed_latency(mpi, fix ? 1U : 0U);

	/* Configure MR0 and MR4 */
	if (fix) {
		mr0 = (1U << 5) | (rlat_arr[r_lat / 2] << 2) | 1U;
		mr4 = (wlat_arr[w_lat] << 5);
	} else {
		mr0 = (rlat_arr[r_lat] << 2) | 1U;
		mr4 = (wlat_arr[w_lat] << 5);
	}

	mpi_mr_write(dev, 0, mr0);
	mpi_mr_write(dev, 4, mr4);
}

static void mpi_configure_ahb_cmd(const struct device *dev)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	struct memc_sf32lb_mpi_opi_psram_data *data = dev->data;
	MPI_TypeDef *mpi = cfg->mpi;
	uint32_t hrccr, hwccr;

	/* Configure AHB read command */
	hrccr = FIELD_PREP(MPI_HRCCR_IMODE_Msk, CCR_MODE_OCT) |
		FIELD_PREP(MPI_HRCCR_ADMODE_Msk, CCR_MODE_OCT) |
		FIELD_PREP(MPI_HRCCR_ADSIZE_Msk, CCR_ADSIZE_32) |
		FIELD_PREP(MPI_HRCCR_ABMODE_Msk, CCR_MODE_NONE) |
		FIELD_PREP(MPI_HRCCR_DCYC_Msk, data->rd_latency - 1) |
		FIELD_PREP(MPI_HRCCR_DMODE_Msk, CCR_MODE_OCT);
	ll_mpi_set_ahb_read_config(mpi, hrccr);

	/* Configure AHB write command */
	hwccr = FIELD_PREP(MPI_HWCCR_IMODE_Msk, CCR_MODE_OCT) |
		FIELD_PREP(MPI_HWCCR_ADMODE_Msk, CCR_MODE_OCT) |
		FIELD_PREP(MPI_HWCCR_ADSIZE_Msk, CCR_ADSIZE_32) |
		FIELD_PREP(MPI_HWCCR_ABMODE_Msk, CCR_MODE_NONE) |
		FIELD_PREP(MPI_HWCCR_DCYC_Msk, data->wr_latency - 1) |
		FIELD_PREP(MPI_HWCCR_DMODE_Msk, CCR_MODE_OCT);
	ll_mpi_set_ahb_write_config(mpi, hwccr);

	/* Set read/write commands */
	ll_mpi_set_ahb_read_command(mpi, OPSRAM_CMD_READ);
	ll_mpi_set_ahb_write_command(mpi, OPSRAM_CMD_WRITE);
}

static void mpi_set_cs_timing(const struct device *dev, uint32_t freq)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	MPI_TypeDef *mpi = cfg->mpi;
	uint16_t cs_min, cs_max, cshmin, trcmin;

	/* OPI frequency is half of MPI clock */
	freq /= 2;

	cs_min = 6;

	if (freq <= 24000000) {
		cs_max = 180;
		cshmin = 0;
		trcmin = 3;
	} else if (freq <= 120000000) {
		cs_max = 950;
		cshmin = 3;
		trcmin = 14;
	} else if (freq <= 144000000) {
		cs_max = 1140;
		cshmin = 5;
		trcmin = 17;
	} else {
		cs_max = 1330;
		cshmin = 8;
		trcmin = 20;
	}

	ll_mpi_set_cs_timing(mpi, cs_max, cs_min, cshmin, trcmin);
}

static void mpi_set_latency_by_freq(const struct device *dev, uint32_t freq)
{
	struct memc_sf32lb_mpi_opi_psram_data *data = dev->data;

	/* OPI frequency is half of MPI clock */
	freq /= 2;

	if (freq <= 24000000) {
		data->rd_latency = 3;
		data->wr_latency = 3;
	} else if (freq <= 120000000) {
		data->rd_latency = 5;
		data->wr_latency = 5;
	} else if (freq <= 144000000) {
		data->rd_latency = 6;
		data->wr_latency = 6;
	} else {
		data->rd_latency = 7;
		data->wr_latency = 7;
	}
}

static int memc_sf32lb_mpi_opi_psram_init(const struct device *dev)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	struct memc_sf32lb_mpi_opi_psram_data *data = dev->data;
	MPI_TypeDef *mpi = cfg->mpi;
	uint32_t freq;
	int ret;

	/* Enable power supply if specified */
	if (cfg->power_supply != NULL) {
		if (!device_is_ready(cfg->power_supply)) {
			LOG_ERR("Power supply device not ready");
			return -ENODEV;
		}
		ret = regulator_enable(cfg->power_supply);
		if (ret < 0) {
			LOG_ERR("Failed to enable power supply: %d", ret);
			return ret;
		}
		/* Wait for LDO to stabilize */
		k_busy_wait(5000);
		LOG_DBG("Power supply enabled");
	}

	/* Configure pinmux */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Failed to apply pinctrl: %d", ret);
		return ret;
	}

	/* Check clock device ready */
	if (!sf32lb_clock_is_ready_dt(&cfg->clock)) {
		LOG_ERR("Clock device is not ready");
		return -ENODEV;
	}

	/* Enable clock */
	ret = sf32lb_clock_control_on_dt(&cfg->clock);
	if (ret < 0) {
		LOG_ERR("Failed to enable clock: %d", ret);
		return ret;
	}

	/* Wait for DLL2 to lock before calibration */
	k_busy_wait(200);

	ret = sf32lb_clock_control_get_rate_dt(&cfg->clock, &freq);
	if (ret < 0) {
		LOG_ERR("Failed to get clock rate: %d", ret);
		/* Default to 288MHz if getting rate fails */
		freq = 288000000;
	}

	LOG_DBG("MPI clock frequency: %u Hz", freq);

	mpi_qspi_init(dev);

	/* Calibrate delay */
	ret = mpi_calibrate_delay(dev);
	if (ret < 0) {
		LOG_ERR("Calibration failed: %d", ret);
		return ret;
	}

	/* Set prescaler to 1 (no division) */
	ll_mpi_set_clock_div(mpi, 1U);

	/* Set CS timing based on frequency */
	mpi_set_cs_timing(dev, freq);

	/* Set latency based on frequency */
	mpi_set_latency_by_freq(dev, freq);

	/* Configure DCR: row boundary=7 (1KB), enable DQS */
	ll_mpi_set_row_boundary_size(mpi, 7U);
	ll_mpi_enable_dqs(mpi);

	/* Set delay values */
	mpi_set_delays(dev);

	/* Enable QSPI and OPI mode */
	ll_mpi_enable(mpi);
	ll_mpi_set_protocol(mpi, LL_MPI_PROTO_OPI);

	/* Reset PSRAM */
	ret = mpi_psram_reset(dev);
	if (ret < 0) {
		LOG_ERR("PSRAM reset failed: %d", ret);
		return ret;
	}

	/* Write MR8 = 0x03 (burst length) */
	ret = mpi_mr_write(dev, 8, 0x03);
	if (ret < 0) {
		LOG_ERR("MR8 write failed: %d", ret);
		return ret;
	}

	/* Calculate latencies for fixed latency mode */
	uint8_t w_lat, r_lat;
	uint32_t psram_freq = freq / 2;

	if (psram_freq <= 66000000) {
		w_lat = 3;
	} else if (psram_freq <= 109000000) {
		w_lat = 4;
	} else if (psram_freq <= 133000000) {
		w_lat = 5;
	} else if (psram_freq <= 166000000) {
		w_lat = 6;
	} else {
		w_lat = 7;
	}
	r_lat = w_lat * 2;

	data->rd_latency = r_lat;
	data->wr_latency = w_lat;

	/* Configure AHB commands */
	mpi_configure_ahb_cmd(dev);

	/* Set fixed latency */
	mpi_set_fixlat(dev, true, r_lat, w_lat);

	/* Set watchdog timer */
	ll_mpi_set_watchdog(mpi, 0x1FFFFU);

	LOG_INF("PSRAM initialized: base=0x%08lx, size=%u bytes", (unsigned long)cfg->psram_base,
		cfg->size);

	return 0;
}

#define MEMC_SF32LB_MPI_OPI_PSRAM_INIT(n)                                                          \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static struct memc_sf32lb_mpi_opi_psram_data memc_sf32lb_mpi_opi_psram_data_##n;           \
                                                                                                   \
	static const struct memc_sf32lb_mpi_opi_psram_config                                       \
		memc_sf32lb_mpi_opi_psram_config_##n = {                                           \
			.mpi = (MPI_TypeDef *)DT_INST_REG_ADDR_BY_NAME(n, ctrl),                            \
			.psram_base = DT_INST_REG_ADDR_BY_NAME(n, psram),                         \
			.size = DT_INST_REG_SIZE_BY_NAME(n, psram),                               \
			.clock = SF32LB_CLOCK_DT_INST_SPEC_GET(n),                                \
			.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                \
			.power_supply = COND_CODE_1(                                               \
				DT_INST_NODE_HAS_PROP(n, power_supply),                            \
				(DEVICE_DT_GET(DT_INST_PHANDLE(n, power_supply))),                 \
				(NULL)),                                                            \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, memc_sf32lb_mpi_opi_psram_init, NULL,                            \
			      &memc_sf32lb_mpi_opi_psram_data_##n,                                 \
			      &memc_sf32lb_mpi_opi_psram_config_##n, POST_KERNEL,                  \
			      CONFIG_MEMC_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MEMC_SF32LB_MPI_OPI_PSRAM_INIT)
