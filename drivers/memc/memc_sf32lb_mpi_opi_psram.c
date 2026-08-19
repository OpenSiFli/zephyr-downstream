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

/* Alternate byte size values */
#define CCR_ABSIZE_8    0U
#define CCR_ABSIZE_16   1U
#define CCR_ABSIZE_24   2U
#define CCR_ABSIZE_32   3U

/*
 * PSRAM interface protocols supported by the MPI controller.
 * The enum order MUST match the "sifli,protocol" DT string enum so that
 * DT_INST_ENUM_IDX() can be used to select the protocol descriptor.
 */
enum sf32lb_psram_protocol {
	SF32LB_PSRAM_PROTO_XCCELA_OPI = 0, /* Xccela octal (SiP OPI PSRAM)      */
	SF32LB_PSRAM_PROTO_XCCELA_LEGACY,  /* Xccela legacy (AP 32Mb, external) */
	SF32LB_PSRAM_PROTO_HYPERRAM,       /* HyperBus (external)               */
};

/* Dummy-cycle formula used to derive AHB/MR access dummy cycles */
enum sf32lb_psram_dummy {
	SF32LB_PSRAM_DUMMY_OPI,    /* dcyc = lat - 1 (Xccela OPI)           */
	SF32LB_PSRAM_DUMMY_LEGACY, /* dcyc = 2*lat (rd) / lat (wr) (legacy) */
};

struct sf32lb_psram_cmd {
	uint8_t read;
	uint8_t write;
	uint8_t mr_read;
	uint8_t mr_write;
	uint8_t reset;
};

struct memc_sf32lb_mpi_opi_psram_data;

/*
 * Per-protocol descriptor. Everything that differs between the PSRAM
 * interface protocols (Xccela OPI / Xccela legacy QPI / HyperBus) is
 * described here so that one driver can cover all of them.
 */
struct sf32lb_psram_proto {
	uint32_t ll_proto;    /* LL_MPI_PROTO_*                    */
	uint8_t xlegacy;      /* DCR.XLEGACY (Xccela legacy)       */
	uint8_t dqs;          /* DCR.DQSE                          */
	uint8_t imode;        /* CCR instruction phase mode        */
	uint8_t admode;       /* CCR address phase mode            */
	uint8_t dmode;        /* CCR data phase mode               */
	uint8_t adsize;       /* CCR address size                  */
	uint8_t reset_count;  /* number of reset command cycles    */
	uint8_t reset_absize; /* reset alternate-byte size         */
	uint8_t mr_wr_len;    /* MR write data length in bytes     */
	bool     burst_mr8;   /* program MR8 burst length (Xccela) */
	uint8_t dummy_mode;   /* enum sf32lb_psram_dummy           */
	uint8_t ahb_abmode;   /* AHB read/write alternate-byte mode (CCR_ABMODE) */
	struct sf32lb_psram_cmd cmd;
	/* Compute dummy-cycle read/write latency from the MPI clock rate */
	void (*set_dummy_latency)(struct memc_sf32lb_mpi_opi_psram_data *data, uint32_t freq);
	/* Program device MR registers + DCR.FIXLAT for fixed-latency mode */
	int (*set_fixlat)(const struct device *dev, uint32_t psram_freq);
};

struct memc_sf32lb_mpi_opi_psram_config {
	MPI_TypeDef *mpi;
	uintptr_t psram_base;
	uint32_t size;
	struct sf32lb_clock_dt_spec clock;
	const struct pinctrl_dev_config *pcfg;
	const struct device *power_supply;
	const struct sf32lb_psram_proto *proto;
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

/*
 * Compute dummy cycles for read/write access from the protocol dummy mode.
 * - OPI:    dcyc = latency - 1
 * - legacy: dcyc = 2*latency (read), latency (write)
 */
static inline uint8_t mpi_rd_dummy(uint8_t mode, uint8_t lat)
{
	return (mode == SF32LB_PSRAM_DUMMY_LEGACY) ? (uint8_t)(lat * 2U) : (uint8_t)(lat - 1U);
}

static inline uint8_t mpi_wr_dummy(uint8_t mode, uint8_t lat)
{
	return (mode == SF32LB_PSRAM_DUMMY_LEGACY) ? lat : (uint8_t)(lat - 1U);
}

static int mpi_psram_reset(const struct device *dev)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	const struct sf32lb_psram_proto *proto = cfg->proto;
	MPI_TypeDef *mpi = cfg->mpi;
	uint8_t i;
	int ret;

	/*
	 * Configure reset command: write mode, no data, alternate byte in octal,
	 * protocol address size and instruction/address phases. Legacy (AP 32Mb)
	 * PSRAM needs two reset cycles and a 4-byte alternate byte.
	 */
	for (i = 0; i < proto->reset_count; i++) {
		mpi_manual_cmd(mpi, true, CCR_MODE_NONE, 0, CCR_MODE_OCT, proto->reset_absize,
			       proto->adsize, proto->admode, proto->imode);

		/* Send reset command */
		ll_mpi_set_address(mpi, LL_MPI_CS_1, 0U);
		ll_mpi_set_command_byte(mpi, LL_MPI_CS_1, proto->cmd.reset);
		ret = mpi_wait_complete(mpi);
		if (ret < 0) {
			return ret;
		}
	}

	/* Wait for PSRAM to reset */
	mpi_delay_us(3);
	return 0;
}

static int mpi_mr_write(const struct device *dev, uint8_t addr, uint8_t value)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	const struct sf32lb_psram_proto *proto = cfg->proto;
	MPI_TypeDef *mpi = cfg->mpi;

	/* Configure MR write command: write mode, protocol data phase, no dummy */
	mpi_manual_cmd(mpi, true, proto->dmode, 0, CCR_MODE_NONE, 0, proto->adsize,
		       proto->admode, proto->imode);

	/* Set data length (2 bytes OPI / 4 bytes legacy) */
	ll_mpi_set_data_length(mpi, LL_MPI_CS_1, proto->mr_wr_len);

	/* Write data to FIFO */
	ll_mpi_write_data(mpi, (uint32_t)value);

	/* Send command */
	ll_mpi_set_address(mpi, LL_MPI_CS_1, addr);
	ll_mpi_set_command_byte(mpi, LL_MPI_CS_1, proto->cmd.mr_write);
	return mpi_wait_complete(mpi);
}

static __maybe_unused uint8_t mpi_mr_read(const struct device *dev, uint8_t addr)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	const struct sf32lb_psram_proto *proto = cfg->proto;
	struct memc_sf32lb_mpi_opi_psram_data *data = dev->data;
	MPI_TypeDef *mpi = cfg->mpi;
	uint8_t rdcyc = data->rd_latency;

	/* Configure MR read command: read mode, protocol data phase, dummy cycles */
	mpi_manual_cmd(mpi, false, proto->dmode, mpi_rd_dummy(proto->dummy_mode, rdcyc),
		       CCR_MODE_NONE, 0, proto->adsize, proto->admode, proto->imode);

	/* Set data length to 2 bytes */
	ll_mpi_set_data_length(mpi, LL_MPI_CS_1, 2U);

	/* Send command */
	ll_mpi_set_address(mpi, LL_MPI_CS_1, addr);
	ll_mpi_set_command_byte(mpi, LL_MPI_CS_1, proto->cmd.mr_read);
	if (mpi_wait_complete(mpi) < 0) {
		return 0;
	}

	return (uint8_t)(ll_mpi_read_data(mpi) & 0xFFU);
}

static int mpi_csr_write(const struct device *dev, uint8_t idx, uint16_t value)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	const struct sf32lb_psram_proto *proto = cfg->proto;
	MPI_TypeDef *mpi = cfg->mpi;

	/*
	 * HyperBus CSR write: command 0x60, address 0x10000, alternate byte
	 * carries the CSR index (one byte, octal phase).
	 */
	mpi_manual_cmd(mpi, true, proto->dmode, 0, CCR_MODE_OCT, CCR_ABSIZE_8, proto->adsize,
		       proto->admode, proto->imode);

	ll_mpi_set_alt_bytes(mpi, LL_MPI_CS_1, idx);
	ll_mpi_set_data_length(mpi, LL_MPI_CS_1, 2U);
	ll_mpi_write_data(mpi, (uint32_t)value);
	ll_mpi_set_address(mpi, LL_MPI_CS_1, 0x10000U);
	ll_mpi_set_command_byte(mpi, LL_MPI_CS_1, 0x60U);
	return mpi_wait_complete(mpi);
}

static void mpi_configure_ahb_cmd(const struct device *dev)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	const struct sf32lb_psram_proto *proto = cfg->proto;
	struct memc_sf32lb_mpi_opi_psram_data *data = dev->data;
	MPI_TypeDef *mpi = cfg->mpi;
	uint32_t hrccr, hwccr;

	/* Configure AHB read command */
	hrccr = FIELD_PREP(MPI_HRCCR_IMODE_Msk, proto->imode) |
		FIELD_PREP(MPI_HRCCR_ADMODE_Msk, proto->admode) |
		FIELD_PREP(MPI_HRCCR_ADSIZE_Msk, proto->adsize) |
		FIELD_PREP(MPI_HRCCR_ABMODE_Msk, proto->ahb_abmode) |
		FIELD_PREP(MPI_HRCCR_DCYC_Msk, mpi_rd_dummy(proto->dummy_mode, data->rd_latency)) |
		FIELD_PREP(MPI_HRCCR_DMODE_Msk, proto->dmode);
	ll_mpi_set_ahb_read_config(mpi, hrccr);

	/* Configure AHB write command */
	hwccr = FIELD_PREP(MPI_HWCCR_IMODE_Msk, proto->imode) |
		FIELD_PREP(MPI_HWCCR_ADMODE_Msk, proto->admode) |
		FIELD_PREP(MPI_HWCCR_ADSIZE_Msk, proto->adsize) |
		FIELD_PREP(MPI_HWCCR_ABMODE_Msk, proto->ahb_abmode) |
		FIELD_PREP(MPI_HWCCR_DCYC_Msk, mpi_wr_dummy(proto->dummy_mode, data->wr_latency)) |
		FIELD_PREP(MPI_HWCCR_DMODE_Msk, proto->dmode);
	ll_mpi_set_ahb_write_config(mpi, hwccr);

	/* Set read/write commands */
	ll_mpi_set_ahb_read_command(mpi, proto->cmd.read);
	ll_mpi_set_ahb_write_command(mpi, proto->cmd.write);
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

static void mpi_set_latency_by_freq(struct memc_sf32lb_mpi_opi_psram_data *data, uint32_t freq)
{
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

static uint8_t mpi_get_fixed_wr_latency(uint32_t psram_freq)
{
	if (psram_freq <= 66000000) {
		return 3;
	} else if (psram_freq <= 109000000) {
		return 4;
	} else if (psram_freq <= 133000000) {
		return 5;
	} else if (psram_freq <= 166000000) {
		return 6;
	}

	return 7;
}

static int mpi_set_fixlat_opi(const struct device *dev, uint32_t psram_freq)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	struct memc_sf32lb_mpi_opi_psram_data *data = dev->data;
	uint8_t mr0, mr4;
	uint8_t rlat_arr[8] = {0, 0, 0, 0, 1, 2, 3, 4};
	uint8_t wlat_arr[8] = {0, 0, 0, 0, 4, 2, 6, 1};
	uint8_t w_lat = mpi_get_fixed_wr_latency(psram_freq);
	uint8_t r_lat = w_lat * 2U;
	int ret;

	data->rd_latency = r_lat;
	data->wr_latency = w_lat;

	/* Set fixed latency in DCR */
	ll_mpi_set_fixed_latency(cfg->mpi, 1U);

	/* Configure MR0 and MR4 for Xccela OPI */
	mr0 = (1U << 5) | (rlat_arr[r_lat / 2] << 2) | 1U;
	mr4 = (wlat_arr[w_lat] << 5);

	ret = mpi_mr_write(dev, 0, mr0);
	if (ret < 0) {
		return ret;
	}

	return mpi_mr_write(dev, 4, mr4);
}

static int mpi_set_fixlat_legacy(const struct device *dev, uint32_t psram_freq)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	struct memc_sf32lb_mpi_opi_psram_data *data = dev->data;
	uint8_t rd_lat, wr_lat, mr0, mr4;
	int ret;

	/* Legacy (AP 32Mb) fixed-latency table (rdcyc / wdcyc) */
	if (psram_freq <= 120000000) {
		rd_lat = 4;
		wr_lat = 0;
	} else if (psram_freq <= 144000000) {
		rd_lat = 5;
		wr_lat = 0;
	} else {
		rd_lat = 6;
		wr_lat = 2;
	}

	data->rd_latency = rd_lat;
	data->wr_latency = wr_lat;

	/* Set fixed latency in DCR */
	ll_mpi_set_fixed_latency(cfg->mpi, 1U);

	/* MR0: fixed-latency + read latency + drive strength */
	mr0 = (1U << 5) | (rd_lat << 2) | 3U;
	/* MR4: write latency + refresh */
	mr4 = (wr_lat << 7) | (1U << 3);

	ret = mpi_mr_write(dev, 0, mr0);
	if (ret < 0) {
		return ret;
	}

	return mpi_mr_write(dev, 4, mr4);
}

static int mpi_set_fixlat_hyper(const struct device *dev, uint32_t psram_freq)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	uint16_t cr0;

	/* HyperRAM CSR0 latency configuration (2 bytes, device byte-swaps) */
	if (psram_freq <= 85000000) {
		cr0 = (14U << 12) | 0x078fU;
	} else if (psram_freq <= 104000000) {
		cr0 = (15U << 12) | 0x078fU;
	} else if (psram_freq <= 120000000) {
		cr0 = (0U << 12) | 0x078fU;
	} else if (psram_freq <= 144000000) {
		cr0 = (1U << 12) | 0x078fU;
	} else {
		cr0 = (2U << 12) | 0x078fU;
	}

	/*
	 * rd/wr latency (dummy cycles) is already set by set_dummy_latency()
	 * with the shared Xccela OPI table and is used by mpi_configure_ahb_cmd().
	 */
	ll_mpi_set_fixed_latency(cfg->mpi, 1U);

	return mpi_csr_write(dev, 0, cr0);
}

static const struct sf32lb_psram_proto sf32lb_psram_protos[] = {
	[SF32LB_PSRAM_PROTO_XCCELA_OPI] = {
		.ll_proto = LL_MPI_PROTO_OPI,
		.xlegacy = 0U,
		.dqs = 1U,
		.imode = CCR_MODE_OCT,
		.admode = CCR_MODE_OCT,
		.dmode = CCR_MODE_OCT,
		.adsize = CCR_ADSIZE_32,
		.reset_count = 1U,
		.reset_absize = CCR_ABSIZE_8,
		.mr_wr_len = 2U,
		.burst_mr8 = true,
		.dummy_mode = SF32LB_PSRAM_DUMMY_OPI,
		.ahb_abmode = CCR_MODE_NONE,
		.cmd = {
			.read = OPSRAM_CMD_READ,
			.write = OPSRAM_CMD_WRITE,
			.mr_read = OPSRAM_CMD_MRREAD,
			.mr_write = OPSRAM_CMD_MRWRITE,
			.reset = OPSRAM_CMD_RESET,
		},
		.set_dummy_latency = mpi_set_latency_by_freq,
		.set_fixlat = mpi_set_fixlat_opi,
	},
	[SF32LB_PSRAM_PROTO_XCCELA_LEGACY] = {
		.ll_proto = LL_MPI_PROTO_OPI,
		.xlegacy = 1U,
		.dqs = 1U,
		.imode = CCR_MODE_OCT,
		.admode = CCR_MODE_OCT,
		.dmode = CCR_MODE_OCT,
		.adsize = CCR_ADSIZE_24,
		.reset_count = 2U,
		.reset_absize = CCR_ABSIZE_32,
		.mr_wr_len = 4U,
		.burst_mr8 = false,
		.dummy_mode = SF32LB_PSRAM_DUMMY_LEGACY,
		.ahb_abmode = CCR_MODE_NONE,
		.cmd = {
			.read = OPSRAM_CMD_READ,
			.write = OPSRAM_CMD_WRITE,
			.mr_read = OPSRAM_CMD_MRREAD,
			.mr_write = OPSRAM_CMD_MRWRITE,
			.reset = OPSRAM_CMD_RESET,
		},
		.set_dummy_latency = mpi_set_latency_by_freq,
		.set_fixlat = mpi_set_fixlat_legacy,
	},
	[SF32LB_PSRAM_PROTO_HYPERRAM] = {
		.ll_proto = LL_MPI_PROTO_HYPER,
		.xlegacy = 0U,
		.dqs = 1U,
		.imode = CCR_MODE_OCT,
		.admode = CCR_MODE_OCT,
		.dmode = CCR_MODE_OCT,
		.adsize = CCR_ADSIZE_32,
		.reset_count = 0U,
		.reset_absize = CCR_ABSIZE_16,
		.mr_wr_len = 2U,
		.burst_mr8 = false,
		.dummy_mode = SF32LB_PSRAM_DUMMY_OPI,
		.ahb_abmode = CCR_MODE_OCT,
		.cmd = {
			.read = 0x0BU,   /* FAST_READ: HyperRAM AHB read command      */
			.write = 0x02U,  /* WRITE: HyperRAM AHB write command         */
			.mr_read = 0xE0U,
			.mr_write = 0x60U,
			.reset = OPSRAM_CMD_RESET,
		},
		.set_dummy_latency = mpi_set_latency_by_freq,
		.set_fixlat = mpi_set_fixlat_hyper,
	},
};

static int memc_sf32lb_mpi_opi_psram_init(const struct device *dev)
{
	const struct memc_sf32lb_mpi_opi_psram_config *cfg = dev->config;
	const struct sf32lb_psram_proto *proto = cfg->proto;
	struct memc_sf32lb_mpi_opi_psram_data *data = dev->data;
	MPI_TypeDef *mpi = cfg->mpi;
	uint32_t freq;
	int ret;

#if defined(CONFIG_REGULATOR)
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
#endif /* CONFIG_REGULATOR */

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
	proto->set_dummy_latency(data, freq);

	/* Configure DCR: row boundary=7 (1KB), DQS per protocol */
	ll_mpi_set_row_boundary_size(mpi, 7U);
	ll_mpi_set_dqs(mpi, proto->dqs);

	/* Set delay values */
	mpi_set_delays(dev);

	/* Enable MPI and select protocol */
	ll_mpi_enable(mpi);
	ll_mpi_set_protocol(mpi, proto->ll_proto);
	ll_mpi_set_xlegacy(mpi, proto->xlegacy);

	/* Reset PSRAM */
	ret = mpi_psram_reset(dev);
	if (ret < 0) {
		LOG_ERR("PSRAM reset failed: %d", ret);
		return ret;
	}

	/* Write MR8 = 0x03 (burst length, Xccela OPI only) */
	if (proto->burst_mr8) {
		ret = mpi_mr_write(dev, 8, 0x03);
		if (ret < 0) {
			LOG_ERR("MR8 write failed: %d", ret);
			return ret;
		}
	}

	/* Program fixed-latency mode (device MR + DCR.FIXLAT) */
	ret = proto->set_fixlat(dev, freq / 2);
	if (ret < 0) {
		LOG_ERR("Fixed latency config failed: %d", ret);
		return ret;
	}

	/* Configure AHB commands */
	mpi_configure_ahb_cmd(dev);

	/* Set watchdog timer */
	ll_mpi_set_watchdog(mpi, 0x1FFFFU);

	LOG_INF("PSRAM initialized: base=0x%08lx, size=%u bytes", (unsigned long)cfg->psram_base,
		cfg->size);

	return 0;
}

/* power_supply is optional, so only expand its phandle when the property exists. */
#define MEMC_SF32LB_POWER_SUPPLY(n)                                                        \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, power_supply),                                    \
		(DEVICE_DT_GET(DT_INST_PHANDLE(n, power_supply))),                               \
		(NULL))

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
			.power_supply = MEMC_SF32LB_POWER_SUPPLY(n),                             \
			.proto = &sf32lb_psram_protos[DT_INST_ENUM_IDX(n, sifli_protocol)],              \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, memc_sf32lb_mpi_opi_psram_init, NULL,                            \
			      &memc_sf32lb_mpi_opi_psram_data_##n,                                 \
			      &memc_sf32lb_mpi_opi_psram_config_##n, POST_KERNEL,                  \
			      CONFIG_MEMC_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MEMC_SF32LB_MPI_OPI_PSRAM_INIT)
