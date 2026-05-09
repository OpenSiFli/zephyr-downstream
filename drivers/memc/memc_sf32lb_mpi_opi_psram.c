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

#include <register.h>

LOG_MODULE_REGISTER(memc_sf32lb_mpi_opi_psram, CONFIG_MEMC_LOG_LEVEL);

/* MPI register offsets */
#define MPI_CR      offsetof(MPI_TypeDef, CR)
#define MPI_DR      offsetof(MPI_TypeDef, DR)
#define MPI_DCR     offsetof(MPI_TypeDef, DCR)
#define MPI_PSCLR   offsetof(MPI_TypeDef, PSCLR)
#define MPI_SR      offsetof(MPI_TypeDef, SR)
#define MPI_SCR     offsetof(MPI_TypeDef, SCR)
#define MPI_CMDR1   offsetof(MPI_TypeDef, CMDR1)
#define MPI_AR1     offsetof(MPI_TypeDef, AR1)
#define MPI_ABR1    offsetof(MPI_TypeDef, ABR1)
#define MPI_DLR1    offsetof(MPI_TypeDef, DLR1)
#define MPI_CCR1    offsetof(MPI_TypeDef, CCR1)
#define MPI_HCMDR   offsetof(MPI_TypeDef, HCMDR)
#define MPI_HRABR   offsetof(MPI_TypeDef, HRABR)
#define MPI_HRCCR   offsetof(MPI_TypeDef, HRCCR)
#define MPI_HWABR   offsetof(MPI_TypeDef, HWABR)
#define MPI_HWCCR   offsetof(MPI_TypeDef, HWCCR)
#define MPI_FIFOCR  offsetof(MPI_TypeDef, FIFOCR)
#define MPI_MISCR   offsetof(MPI_TypeDef, MISCR)
#define MPI_TIMR    offsetof(MPI_TypeDef, TIMR)
#define MPI_WDTR    offsetof(MPI_TypeDef, WDTR)
#define MPI_CALCR   offsetof(MPI_TypeDef, CALCR)
#define MPI_APM32CR offsetof(MPI_TypeDef, APM32CR)
#define MPI_CIR     offsetof(MPI_TypeDef, CIR)

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
	uintptr_t mpi_base;
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

static int mpi_wait_complete(uintptr_t mpi)
{
	int retries = 10000;

	while (!(sys_read32(mpi + MPI_SR) & MPI_SR_TCF)) {
		if (--retries <= 0) {
			LOG_ERR("MPI transfer timeout");
			return -ETIMEDOUT;
		}
	}
	sys_write32(MPI_SCR_TCFC, mpi + MPI_SCR);
	return 0;
}

static void mpi_qspi_init(const struct device *dev)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	uintptr_t mpi = cfg->mpi_base;

	sys_write32(0xFFU, mpi + MPI_TIMR);
	sys_write32(0x50005000, mpi + MPI_CIR);
	sys_write32(0xFFU, mpi + MPI_ABR1);
	sys_write32(0xFFU, mpi + MPI_HRABR);
}

static int mpi_calibrate_delay(const struct device *dev)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	struct memc_sf32lb_mpi_opi_psram_data *data = dev->data;
	uintptr_t mpi = cfg->mpi_base;
	uint32_t delay;
	uint32_t miscr;

	/* Set prescaler to 2 for calibration */
	sys_write32(2U, mpi + MPI_PSCLR);

	/* Clear SCK inversion */
	miscr = sys_read32(mpi + MPI_MISCR);
	miscr &= ~MPI_MISCR_SCKINV_Msk;
	sys_write32(miscr, mpi + MPI_MISCR);

	/* Enable calibration */
	sys_set_bit(mpi + MPI_CALCR, MPI_CALCR_EN_Pos);

	/* Wait for calibration to complete (with timeout) */
	mpi_delay_us(20);
	{
		int retries = 1000;

		while (!(sys_read32(mpi + MPI_CALCR) & MPI_CALCR_DONE)) {
			if (--retries <= 0) {
				sys_clear_bit(mpi + MPI_CALCR, MPI_CALCR_EN_Pos);
				LOG_ERR("MPI calibration timeout (DLL2 not locked?)");
				return -ETIMEDOUT;
			}
			mpi_delay_us(1);
		}
	}

	/* Read delay value */
	delay = sys_read32(mpi + MPI_CALCR) & MPI_CALCR_DELAY_Msk;

	/* Disable calibration */
	sys_clear_bit(mpi + MPI_CALCR, MPI_CALCR_EN_Pos);

	if (delay < 4) {
		LOG_ERR("MPI calibration result too small: %u", delay);
		return -EINVAL;
	}

	/* Calculate SCK and DQS delays (SF32LB52X specific) */
	data->sck_delay = (uint8_t)(delay - 1);
	data->dqs_delay = (uint8_t)(delay - 4);

	/* Restore prescaler to 1 */
	sys_write32(1U, mpi + MPI_PSCLR);

	LOG_DBG("Calibration: delay=%u, sck=%u, dqs=%u", delay, data->sck_delay, data->dqs_delay);

	return 0;
}

static void mpi_set_delays(const struct device *dev)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	struct memc_sf32lb_mpi_opi_psram_data *data = dev->data;
	uintptr_t mpi = cfg->mpi_base;
	uint32_t miscr;

	miscr = sys_read32(mpi + MPI_MISCR);
	miscr &= ~(MPI_MISCR_SCKDLY_Msk | MPI_MISCR_DQSDLY_Msk);
	miscr |= FIELD_PREP(MPI_MISCR_SCKDLY_Msk, data->sck_delay);
	miscr |= FIELD_PREP(MPI_MISCR_DQSDLY_Msk, data->dqs_delay);
	sys_write32(miscr, mpi + MPI_MISCR);
}

static void mpi_manual_cmd(uintptr_t mpi, bool is_write, uint8_t dmode, uint8_t dcyc,
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

	sys_write32(ccr1, mpi + MPI_CCR1);
}

static int mpi_psram_reset(const struct device *dev)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	uintptr_t mpi = cfg->mpi_base;
	int ret;

	/* Configure reset command: write mode, no data, 1-byte AB, 32-bit addr, OPI mode */
	mpi_manual_cmd(mpi, true, CCR_MODE_NONE, 0, CCR_MODE_OCT, 0, CCR_ADSIZE_32, CCR_MODE_OCT,
		       CCR_MODE_OCT);

	/* Send reset command */
	sys_write32(0U, mpi + MPI_AR1);
	sys_write32(OPSRAM_CMD_RESET, mpi + MPI_CMDR1);
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
	uintptr_t mpi = cfg->mpi_base;

	/* Configure MR write command: write mode, OPI data, no dummy, 32-bit addr, OPI mode */
	mpi_manual_cmd(mpi, true, CCR_MODE_OCT, 0, CCR_MODE_NONE, 0, CCR_ADSIZE_32, CCR_MODE_OCT,
		       CCR_MODE_OCT);

	/* Set data length to 2 bytes */
	sys_write32(1U, mpi + MPI_DLR1);

	/* Write data to FIFO */
	sys_write32((uint32_t)value, mpi + MPI_DR);

	/* Send command */
	sys_write32(addr, mpi + MPI_AR1);
	sys_write32(OPSRAM_CMD_MRWRITE, mpi + MPI_CMDR1);
	return mpi_wait_complete(mpi);
}

static __maybe_unused uint8_t mpi_mr_read(const struct device *dev, uint8_t addr)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	struct memc_sf32lb_mpi_opi_psram_data *data = dev->data;
	uintptr_t mpi = cfg->mpi_base;
	uint8_t rdcyc = data->rd_latency;

	/* Configure MR read command: read mode, OPI data, dummy cycles, 32-bit addr, OPI mode */
	mpi_manual_cmd(mpi, false, CCR_MODE_OCT, rdcyc - 1, CCR_MODE_NONE, 0, CCR_ADSIZE_32,
		       CCR_MODE_OCT, CCR_MODE_OCT);

	/* Set data length to 2 bytes */
	sys_write32(1U, mpi + MPI_DLR1);

	/* Send command */
	sys_write32(addr, mpi + MPI_AR1);
	sys_write32(OPSRAM_CMD_MRREAD, mpi + MPI_CMDR1);
	if (mpi_wait_complete(mpi) < 0) {
		return 0;
	}

	return (uint8_t)(sys_read32(mpi + MPI_DR) & 0xFFU);
}

static void mpi_set_fixlat(const struct device *dev, bool fix, uint8_t r_lat, uint8_t w_lat)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	uintptr_t mpi = cfg->mpi_base;
	uint32_t dcr;
	uint8_t mr0, mr4;
	uint8_t rlat_arr[8] = {0, 0, 0, 0, 1, 2, 3, 4};
	uint8_t wlat_arr[8] = {0, 0, 0, 0, 4, 2, 6, 1};

	/* Set fixed latency in DCR */
	dcr = sys_read32(mpi + MPI_DCR);
	if (fix) {
		dcr |= MPI_DCR_FIXLAT_Msk;
	} else {
		dcr &= ~MPI_DCR_FIXLAT_Msk;
	}
	sys_write32(dcr, mpi + MPI_DCR);

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
	uintptr_t mpi = cfg->mpi_base;
	uint32_t hrccr, hwccr, hcmdr;

	/* Configure AHB read command */
	hrccr = FIELD_PREP(MPI_HRCCR_IMODE_Msk, CCR_MODE_OCT) |
		FIELD_PREP(MPI_HRCCR_ADMODE_Msk, CCR_MODE_OCT) |
		FIELD_PREP(MPI_HRCCR_ADSIZE_Msk, CCR_ADSIZE_32) |
		FIELD_PREP(MPI_HRCCR_ABMODE_Msk, CCR_MODE_NONE) |
		FIELD_PREP(MPI_HRCCR_DCYC_Msk, data->rd_latency - 1) |
		FIELD_PREP(MPI_HRCCR_DMODE_Msk, CCR_MODE_OCT);
	sys_write32(hrccr, mpi + MPI_HRCCR);

	/* Configure AHB write command */
	hwccr = FIELD_PREP(MPI_HWCCR_IMODE_Msk, CCR_MODE_OCT) |
		FIELD_PREP(MPI_HWCCR_ADMODE_Msk, CCR_MODE_OCT) |
		FIELD_PREP(MPI_HWCCR_ADSIZE_Msk, CCR_ADSIZE_32) |
		FIELD_PREP(MPI_HWCCR_ABMODE_Msk, CCR_MODE_NONE) |
		FIELD_PREP(MPI_HWCCR_DCYC_Msk, data->wr_latency - 1) |
		FIELD_PREP(MPI_HWCCR_DMODE_Msk, CCR_MODE_OCT);
	sys_write32(hwccr, mpi + MPI_HWCCR);

	/* Set read/write commands */
	hcmdr = OPSRAM_CMD_READ | (OPSRAM_CMD_WRITE << 8);
	sys_write32(hcmdr, mpi + MPI_HCMDR);
}

static void mpi_set_cs_timing(const struct device *dev, uint32_t freq)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	uintptr_t mpi = cfg->mpi_base;
	uint32_t dcr;
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

	dcr = sys_read32(mpi + MPI_DCR);
	dcr &= ~(MPI_DCR_CSLMAX_Msk | MPI_DCR_CSLMIN_Msk | MPI_DCR_CSHMIN_Msk |
		 MPI_DCR_TRCMIN_Msk);
	dcr |= FIELD_PREP(MPI_DCR_CSLMAX_Msk, cs_max);
	dcr |= FIELD_PREP(MPI_DCR_CSLMIN_Msk, cs_min);
	dcr |= FIELD_PREP(MPI_DCR_CSHMIN_Msk, cshmin);
	dcr |= FIELD_PREP(MPI_DCR_TRCMIN_Msk, trcmin);
	sys_write32(dcr, mpi + MPI_DCR);
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
	uintptr_t mpi = cfg->mpi_base;
	uint32_t cr, dcr;
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
	sys_write32(1U, mpi + MPI_PSCLR);

	/* Set CS timing based on frequency */
	mpi_set_cs_timing(dev, freq);

	/* Set latency based on frequency */
	mpi_set_latency_by_freq(dev, freq);

	/* Configure DCR: row boundary=7 (1KB), enable DQS */
	dcr = sys_read32(mpi + MPI_DCR);
	dcr &= ~MPI_DCR_RBSIZE_Msk;
	dcr |= FIELD_PREP(MPI_DCR_RBSIZE_Msk, 7);
	dcr |= MPI_DCR_DQSE_Msk;
	sys_write32(dcr, mpi + MPI_DCR);

	/* Set delay values */
	mpi_set_delays(dev);

	/* Enable QSPI and OPI mode */
	cr = sys_read32(mpi + MPI_CR);
	cr |= MPI_CR_EN_Msk | MPI_CR_OPIE_Msk;
	sys_write32(cr, mpi + MPI_CR);

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
	sys_write32(0x1FFFF, mpi + MPI_WDTR);

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
			.mpi_base = DT_INST_REG_ADDR_BY_NAME(n, ctrl),                            \
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
