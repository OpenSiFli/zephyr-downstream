/*
 * Copyright (c) 2025 Core Devices LLC
 * Copyright (c) 2025 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sifli_sf32lb_rcc_clk

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control/sf32lb.h>
#include <zephyr/dt-bindings/clock/sf32lb-clocks-common.h>
#include <zephyr/dt-bindings/clock/sf32lb52x-clocks.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

#include <ll_hpsys_cfg.h>
#include <ll_pmuc.h>
#include <ll_rcc.h>

#define SF32LB_CLOCK_FREQ_BY_NAME(inst, name)                                                      \
	COND_CODE_1(DT_NODE_HAS_COMPAT(DT_INST_CLOCKS_CTLR_BY_NAME(inst, name),                   \
					       sifli_sf32lb_pmuc_clk),                          \
		    (DT_PROP_BY_IDX(DT_INST_CLOCKS_CTLR_BY_NAME(inst, name),                   \
				    clock_frequencies,                                         \
				    DT_INST_CLOCKS_CELL_BY_NAME(inst, name, clk_id))),         \
		    (DT_PROP(DT_INST_CLOCKS_CTLR_BY_NAME(inst, name), clock_frequency)))

#define SF32LB_DLL_FREQ(inst, node)                                                                \
	COND_CODE_1(DT_NODE_HAS_STATUS(DT_INST_CHILD(inst, node), okay),                          \
		    (DT_PROP(DT_INST_CHILD(inst, node), clock_frequency)), (0U))

#define SF32LB_DLL1_FREQ(inst) SF32LB_DLL_FREQ(inst, dll1)
#define SF32LB_DLL2_FREQ(inst) SF32LB_DLL_FREQ(inst, dll2)

/* Enum values match the register field encoding used by RCC. Value 2 is reserved. */
enum sf32lb_sys_clk_idx {
	SF32LB_SYS_CLK_IDX_HRC48 = 0,
	SF32LB_SYS_CLK_IDX_HXT48 = 1,
	SF32LB_SYS_CLK_IDX_DLL1 = 3,
};

enum sf32lb_sys_clk_src_idx {
	SF32LB_SYS_CLK_SRC_HRC48 = 0,
	SF32LB_SYS_CLK_SRC_HXT48 = 1,
	SF32LB_SYS_CLK_SRC_DLL1 = 2,
};

enum sf32lb_peri_clk_idx {
	SF32LB_PERI_CLK_IDX_HRC48 = 0,
	SF32LB_PERI_CLK_IDX_HXT48 = 1,
};

enum sf32lb_mpi_clk_idx {
	SF32LB_MPI_CLK_IDX_PERI = 0,
	SF32LB_MPI_CLK_IDX_DLL1 = 1,
	SF32LB_MPI_CLK_IDX_DLL2 = 2,
};

enum sf32lb_usb_clk_idx {
	SF32LB_USB_CLK_IDX_SYSCLK = 0,
	SF32LB_USB_CLK_IDX_DLL2 = 1,
};

#define SF32LB_SYS_CLK_SRC_RAW(inst)                                                               \
	DT_ENUM_IDX_OR(DT_DRV_INST(inst), sifli_sys_clk_src, SF32LB_SYS_CLK_SRC_HRC48)
#define SF32LB_SYS_CLK_SRC_IDX(inst)                                                               \
	((SF32LB_SYS_CLK_SRC_RAW(inst) == SF32LB_SYS_CLK_SRC_HXT48)  ? SF32LB_SYS_CLK_IDX_HXT48    \
	 : (SF32LB_SYS_CLK_SRC_RAW(inst) == SF32LB_SYS_CLK_SRC_DLL1) ? SF32LB_SYS_CLK_IDX_DLL1     \
								     : SF32LB_SYS_CLK_IDX_HRC48)
#define SF32LB_PERI_CLK_SRC_IDX(inst)                                                              \
	DT_ENUM_IDX_OR(DT_DRV_INST(inst), sifli_peri_clk_src, SF32LB_PERI_CLK_IDX_HXT48)
#define SF32LB_MPI1_CLK_SRC_IDX(inst)                                                              \
	DT_ENUM_IDX_OR(DT_DRV_INST(inst), sifli_mpi1_clk_src, SF32LB_MPI_CLK_IDX_PERI)
#define SF32LB_MPI2_CLK_SRC_IDX(inst)                                                              \
	DT_ENUM_IDX_OR(DT_DRV_INST(inst), sifli_mpi2_clk_src, SF32LB_MPI_CLK_IDX_PERI)
#define SF32LB_USB_CLK_SRC_IDX(inst)                                                               \
	DT_ENUM_IDX_OR(DT_DRV_INST(inst), sifli_usb_clk_src, SF32LB_USB_CLK_IDX_SYSCLK)
#define SF32LB_SYS_CLK_SRC_VALUE(inst)  SF32LB_SYS_CLK_SRC_IDX(inst)
#define SF32LB_PERI_CLK_SRC_VALUE(inst) SF32LB_PERI_CLK_SRC_IDX(inst)
#define SF32LB_MPI1_CLK_SRC_VALUE(inst) SF32LB_MPI1_CLK_SRC_IDX(inst)
#define SF32LB_MPI2_CLK_SRC_VALUE(inst) SF32LB_MPI2_CLK_SRC_IDX(inst)
#define SF32LB_USB_CLK_SRC_VALUE(inst)  SF32LB_USB_CLK_SRC_IDX(inst)

#define SF32LB_SYS_CLK_FREQ(inst)                                                                  \
	((SF32LB_SYS_CLK_SRC_IDX(inst) == SF32LB_SYS_CLK_IDX_HRC48)                                \
		 ? SF32LB_CLOCK_FREQ_BY_NAME(inst, hrc48)                                          \
	 : (SF32LB_SYS_CLK_SRC_IDX(inst) == SF32LB_SYS_CLK_IDX_HXT48)                              \
		 ? SF32LB_CLOCK_FREQ_BY_NAME(inst, hxt48)                                          \
		 : SF32LB_DLL1_FREQ(inst))

#define SF32LB_PERI_CLK_FREQ(inst)                                                                 \
	((SF32LB_PERI_CLK_SRC_IDX(inst) == SF32LB_PERI_CLK_IDX_HXT48)                              \
		 ? SF32LB_CLOCK_FREQ_BY_NAME(inst, hxt48)                                          \
		 : SF32LB_CLOCK_FREQ_BY_NAME(inst, hrc48))

#define SF32LB_USB_DIV_VALUE(inst) DT_INST_PROP_OR(DT_DRV_INST(inst), sifli_usb_div, 4)

#define SF32LB_RCC_NEEDS_HXT48(inst)                                                               \
	(DT_NODE_HAS_STATUS(DT_INST_CHILD(inst, dll1), okay) ||                                    \
	 DT_NODE_HAS_STATUS(DT_INST_CHILD(inst, dll2), okay) ||                                    \
	 (SF32LB_SYS_CLK_SRC_IDX(inst) == SF32LB_SYS_CLK_IDX_HXT48) ||                             \
	 (SF32LB_PERI_CLK_SRC_IDX(inst) == SF32LB_PERI_CLK_IDX_HXT48))

#define SF32LB_SYS_CLK_REQUIRES_DLL1(inst) (SF32LB_SYS_CLK_SRC_IDX(inst) == SF32LB_SYS_CLK_IDX_DLL1)
#define SF32LB_MPI1_CLK_REQUIRES_DLL2(inst)                                                        \
	(SF32LB_MPI1_CLK_SRC_VALUE(inst) == SF32LB_MPI_CLK_IDX_DLL2)
#define SF32LB_MPI1_CLK_REQUIRES_DLL1(inst)                                                        \
	(SF32LB_MPI1_CLK_SRC_VALUE(inst) == SF32LB_MPI_CLK_IDX_DLL1)
#define SF32LB_MPI2_CLK_REQUIRES_DLL2(inst)                                                        \
	(SF32LB_MPI2_CLK_SRC_VALUE(inst) == SF32LB_MPI_CLK_IDX_DLL2)
#define SF32LB_MPI2_CLK_REQUIRES_DLL1(inst)                                                        \
	(SF32LB_MPI2_CLK_SRC_VALUE(inst) == SF32LB_MPI_CLK_IDX_DLL1)
#define SF32LB_USB_CLK_REQUIRES_DLL2(inst)                                                         \
	(SF32LB_USB_CLK_SRC_VALUE(inst) == SF32LB_USB_CLK_IDX_DLL2)

struct clock_control_sf32lb_rcc_config {
	uintptr_t base;
	uintptr_t cfg;
	uintptr_t pmuc;
	uint8_t hdiv;
	uint8_t pdiv1;
	uint8_t pdiv2;
	uint8_t sys_clk_src;
	uint8_t peri_clk_src;
	uint8_t mpi1_clk_src;
	uint8_t mpi2_clk_src;
	uint8_t usb_clk_src;
	uint8_t usb_div;
	uint32_t peri_clk_freq;
	uint32_t hrc48_freq;
	uint32_t hxt48_freq;
	uint32_t dll1_freq;
	uint32_t dll2_freq;
	const struct device *hxt48;
};

static inline void configure_dll(HPSYS_RCC_TypeDef *rcc, uint32_t freq, uint32_t dll_idx)
{
	uint32_t stg = (freq / 24000000UL) - 1U;

	ll_rcc_dll_disable(rcc, dll_idx);
	ll_rcc_dll_set_out_div2(rcc, dll_idx, 0U);
	ll_rcc_dll_set_in_div2(rcc, dll_idx, 1U);
	ll_rcc_dll_set_stg(rcc, dll_idx, stg);
	ll_rcc_dll_enable(rcc, dll_idx);
	while (!ll_rcc_dll_is_ready(rcc, dll_idx)) {
	}
}

static bool sf32lb_rcc_needs_hxt48(const struct clock_control_sf32lb_rcc_config *config)
{
	return (config->sys_clk_src == SF32LB_SYS_CLK_IDX_HXT48) ||
	       (config->peri_clk_src == SF32LB_PERI_CLK_IDX_HXT48) || (config->dll1_freq != 0U) ||
	       (config->dll2_freq != 0U);
}

/*
 * Rate queries are computed from the DT configuration so that
 * clock_control_get_rate() returns the configured target frequency even
 * before this driver's init has programmed the hardware (see manual 2.9,
 * CSR.SEL_* / CFGR.HDIV / CFGR.PDIV1 encode the same values).
 */
static uint32_t sf32lb_get_sys_clk(const struct clock_control_sf32lb_rcc_config *config)
{
	switch (config->sys_clk_src) {
	case SF32LB_SYS_CLK_IDX_HXT48:
		return config->hxt48_freq;
	case SF32LB_SYS_CLK_IDX_DLL1:
		return config->dll1_freq;
	case SF32LB_SYS_CLK_IDX_HRC48:
	default:
		return config->hrc48_freq;
	}
}

static uint32_t sf32lb_get_hclk(const struct clock_control_sf32lb_rcc_config *config)
{
	uint32_t hdiv = config->hdiv;

	if (hdiv == 0U) {
		return sf32lb_get_sys_clk(config);
	}

	return sf32lb_get_sys_clk(config) / hdiv;
}

static uint32_t sf32lb_get_pclk1(const struct clock_control_sf32lb_rcc_config *config)
{
	uint32_t divisor = BIT(config->pdiv1);

	return sf32lb_get_hclk(config) / MAX(divisor, 1U);
}

static uint32_t sf32lb_get_clk_peri(const struct clock_control_sf32lb_rcc_config *config)
{
	return config->peri_clk_freq;
}

static uint32_t sf32lb_get_mpi_clk(const struct clock_control_sf32lb_rcc_config *config,
				   uint8_t src)
{
	switch (src) {
	case SF32LB_MPI_CLK_IDX_DLL1:
		return config->dll1_freq;
	case SF32LB_MPI_CLK_IDX_DLL2:
		return config->dll2_freq;
	default:
		return config->peri_clk_freq;
	}
}

static uint32_t sf32lb_get_usb_clk(const struct clock_control_sf32lb_rcc_config *config)
{
	uint32_t src = (config->usb_clk_src == SF32LB_USB_CLK_IDX_DLL2)
			       ? config->dll2_freq
			       : sf32lb_get_sys_clk(config);

	if (config->usb_div == 0U) {
		return src;
	}

	return src / config->usb_div;
}

static int clock_control_sf32lb_rcc_on(const struct device *dev, clock_control_subsys_t sys)
{
	const struct clock_control_sf32lb_rcc_config *config = dev->config;
	HPSYS_RCC_TypeDef *rcc = (HPSYS_RCC_TypeDef *)config->base;
	uint16_t id = *(uint16_t *)sys;
	uint32_t offset = FIELD_GET(SF32LB_CLOCK_OFFSET_MSK, id);
	uint32_t mask = BIT(FIELD_GET(SF32LB_CLOCK_BIT_MSK, id));
	uint32_t group = (offset - SF32LB52X_RCC_ENR1) / 4U;

	ll_rcc_enable_clock(rcc, group, mask);

	return 0;
}

static int clock_control_sf32lb_rcc_off(const struct device *dev, clock_control_subsys_t sys)
{
	const struct clock_control_sf32lb_rcc_config *config = dev->config;
	HPSYS_RCC_TypeDef *rcc = (HPSYS_RCC_TypeDef *)config->base;
	uint16_t id = *(uint16_t *)sys;
	uint32_t offset = FIELD_GET(SF32LB_CLOCK_OFFSET_MSK, id);
	uint32_t mask = BIT(FIELD_GET(SF32LB_CLOCK_BIT_MSK, id));
	uint32_t group = (offset - SF32LB52X_RCC_ENR1) / 4U;

	ll_rcc_disable_clock(rcc, group, mask);

	return 0;
}

int clock_control_sf32lb_rcc_get_rate(const struct device *dev, clock_control_subsys_t sys,
				      uint32_t *rate)
{
	const struct clock_control_sf32lb_rcc_config *config = dev->config;
	uint16_t id = *(uint16_t *)sys;

	switch (id) {
	case SF32LB52X_CLOCK_DMAC1:
	case SF32LB52X_CLOCK_EXTDMA:
	case SF32LB52X_CLOCK_EPIC:
	case SF32LB52X_CLOCK_EZIP1:
	case SF32LB52X_CLOCK_LCDC1:
	case SF32LB52X_CLOCK_AES:
	case SF32LB52X_CLOCK_SDMMC1:
	case SF32LB52X_CLOCK_CRC1:
	case SF32LB52X_CLOCK_SECU1:
		*rate = sf32lb_get_hclk(config);
		return 0;
	case SF32LB52X_CLOCK_MPI1:
		*rate = sf32lb_get_mpi_clk(config, config->mpi1_clk_src);
		return 0;
	case SF32LB52X_CLOCK_MPI2:
		*rate = sf32lb_get_mpi_clk(config, config->mpi2_clk_src);
		return 0;
	case SF32LB52X_CLOCK_USBC:
		*rate = sf32lb_get_usb_clk(config);
		return 0;
	case SF32LB52X_CLOCK_MAILBOX1:
	case SF32LB52X_CLOCK_PINMUX1:
	case SF32LB52X_CLOCK_SYSCFG1:
	case SF32LB52X_CLOCK_GPIO1:
	case SF32LB52X_CLOCK_PTC1:
	case SF32LB52X_CLOCK_TRNG:
	case SF32LB52X_CLOCK_EFUSEC:
	case SF32LB52X_CLOCK_GPADC:
	case SF32LB52X_CLOCK_TSEN:
	case SF32LB52X_CLOCK_GPTIM1:
	case SF32LB52X_CLOCK_ATIM1:
		*rate = sf32lb_get_pclk1(config);
		return 0;
	case SF32LB52X_CLOCK_GPTIM2:
		*rate = 24000000U;
		return 0;
	case SF32LB52X_CLOCK_BTIM1:
	case SF32LB52X_CLOCK_BTIM2:
		*rate = sf32lb_get_pclk1(config) / 2U;
		return 0;
	case SF32LB52X_CLOCK_I2C1:
	case SF32LB52X_CLOCK_I2C2:
	case SF32LB52X_CLOCK_I2C3:
	case SF32LB52X_CLOCK_I2C4:
	case SF32LB52X_CLOCK_SPI1:
	case SF32LB52X_CLOCK_SPI2:
	case SF32LB52X_CLOCK_USART2:
	case SF32LB52X_CLOCK_USART3:
		*rate = sf32lb_get_clk_peri(config);
		return 0;
	case SF32LB52X_CLOCK_I2S1:
	case SF32LB52X_CLOCK_AUDPRC:
	case SF32LB52X_CLOCK_AUDCODEC:
	case SF32LB52X_CLOCK_PDM1:
		*rate = config->hxt48_freq;
		return 0;
	default:
		break;
	}

	return -ENOTSUP;
}

static enum clock_control_status clock_control_sf32lb_rcc_get_status(const struct device *dev,
								     clock_control_subsys_t sys)
{
	const struct clock_control_sf32lb_rcc_config *config = dev->config;
	HPSYS_RCC_TypeDef *rcc = (HPSYS_RCC_TypeDef *)config->base;
	uint16_t id = *(uint16_t *)sys;
	uint32_t offset = FIELD_GET(SF32LB_CLOCK_OFFSET_MSK, id);
	uint32_t mask = BIT(FIELD_GET(SF32LB_CLOCK_BIT_MSK, id));
	uint32_t group = (offset - SF32LB52X_RCC_ENR1) / 4U;

	if (ll_rcc_is_clock_enabled(rcc, group, mask) != 0U) {
		return CLOCK_CONTROL_STATUS_ON;
	}

	return CLOCK_CONTROL_STATUS_OFF;
}

static DEVICE_API(clock_control, clock_control_sf32lb_rcc_api) = {
	.on = clock_control_sf32lb_rcc_on,
	.off = clock_control_sf32lb_rcc_off,
	.get_rate = clock_control_sf32lb_rcc_get_rate,
	.get_status = clock_control_sf32lb_rcc_get_status,
};

static int clock_control_sf32lb_rcc_init(const struct device *dev)
{
	const struct clock_control_sf32lb_rcc_config *config = dev->config;
	HPSYS_RCC_TypeDef *rcc = (HPSYS_RCC_TypeDef *)config->base;
	bool need_hxt48 = sf32lb_rcc_needs_hxt48(config);
	int ret;

	if (need_hxt48) {
		if ((config->hxt48 == NULL) || !device_is_ready(config->hxt48)) {
			return -ENODEV;
		}

		ret = clock_control_on(config->hxt48, NULL);
		if (ret < 0) {
			return ret;
		}
	}

	if (config->dll1_freq != 0U || config->dll2_freq != 0U) {
		/* Enable HXT48 DLL buffer and CAU2 HPBG rail before DLL config */
		ll_pmuc_hxt48_enable_dll_buf((PMUC_TypeDef *)config->pmuc);
		ll_cfg_hpbg_enable((HPSYS_CFG_TypeDef *)config->cfg);

		/* Switch sys/peri clock to HXT48 before DLL configuration */
		ll_rcc_set_sys_clock_source(rcc, LL_RCC_SYS_CLKSRC_HXT48);
		ll_rcc_set_peri_clock_source(rcc, LL_RCC_PERI_CLKSRC_HXT48);

		if (config->dll1_freq != 0U) {
			configure_dll(rcc, config->dll1_freq, LL_RCC_DLL_INDEX_1);
		}

		if (config->dll2_freq != 0U) {
			configure_dll(rcc, config->dll2_freq, LL_RCC_DLL_INDEX_2);
		}
	}

	/* configure HDIV/PDIV1/PDIV2 dividers */
	ll_rcc_set_ahb_div(rcc, config->hdiv);
	ll_rcc_set_apb1_div(rcc, config->pdiv1);
	ll_rcc_set_apb2_div(rcc, config->pdiv2);

	/* configure clock sources */
	ll_rcc_set_sys_clock_source(rcc, config->sys_clk_src);
	ll_rcc_set_peri_clock_source(rcc, config->peri_clk_src);
	ll_rcc_set_mpi1_clock_source(rcc, config->mpi1_clk_src);
	ll_rcc_set_mpi2_clock_source(rcc, config->mpi2_clk_src);
	ll_rcc_set_usb_clock_source(rcc, config->usb_clk_src);

	/* configure USB divider */
	ll_rcc_set_usb_div(rcc, config->usb_div);

	return 0;
}

/* DLL1/2 requires HXT48 */
IF_ENABLED(UTIL_OR(
		DT_NODE_HAS_STATUS(DT_INST_CHILD(0, dll1), okay),
		DT_NODE_HAS_STATUS(DT_INST_CHILD(0, dll2), okay)),
	   (BUILD_ASSERT(
		(DT_NODE_HAS_STATUS(DT_INST_CLOCKS_CTLR_BY_NAME(0, hxt48), okay)),
		"DLL1/2 require HXT48 to be enabled");))

/* DLL1/2 frequency must be a multiple of step size (24 MHz) */
IF_ENABLED(DT_NODE_HAS_STATUS(DT_INST_CHILD(0, dll1), okay), (
	   BUILD_ASSERT(
		((DT_PROP(DT_INST_CHILD(0, dll1), clock_frequency) != 0) &&
		 ((DT_PROP(DT_INST_CHILD(0, dll1), clock_frequency) % 24000000UL) == 0)),
		"DLL1 frequency must be a non-zero multiple of 24000000"
	   );))

IF_ENABLED(DT_NODE_HAS_STATUS(DT_INST_CHILD(0, dll2), okay), (
	   BUILD_ASSERT(
		((DT_PROP(DT_INST_CHILD(0, dll2), clock_frequency) != 0) &&
		 ((DT_PROP(DT_INST_CHILD(0, dll2), clock_frequency) % 24000000UL) == 0)),
		"DLL2 frequency must be a non-zero multiple of 24000000"
	   );))

BUILD_ASSERT(!SF32LB_RCC_NEEDS_HXT48(0) ||
		     DT_NODE_HAS_STATUS(DT_INST_CLOCKS_CTLR_BY_NAME(0, hxt48), okay),
	     "HXT48 clock must be enabled when selected or when DLLs are used");

BUILD_ASSERT(!SF32LB_SYS_CLK_REQUIRES_DLL1(0) || DT_NODE_HAS_STATUS(DT_INST_CHILD(0, dll1), okay),
	     "DLL1 system clock selection requires the dll1 node to be enabled");

BUILD_ASSERT(!SF32LB_MPI1_CLK_REQUIRES_DLL2(0) || DT_NODE_HAS_STATUS(DT_INST_CHILD(0, dll2), okay),
	     "MPI1 clock selection requires the dll2 node to be enabled when set to DLL2");

BUILD_ASSERT(!SF32LB_MPI1_CLK_REQUIRES_DLL1(0) || DT_NODE_HAS_STATUS(DT_INST_CHILD(0, dll1), okay),
	     "MPI1 clock selection requires the dll1 node to be enabled when set to DLL1");

BUILD_ASSERT(!SF32LB_MPI2_CLK_REQUIRES_DLL2(0) || DT_NODE_HAS_STATUS(DT_INST_CHILD(0, dll2), okay),
	     "MPI2 clock selection requires the dll2 node to be enabled when set to DLL2");

BUILD_ASSERT(!SF32LB_MPI2_CLK_REQUIRES_DLL1(0) || DT_NODE_HAS_STATUS(DT_INST_CHILD(0, dll1), okay),
	     "MPI2 clock selection requires the dll1 node to be enabled when set to DLL1");

BUILD_ASSERT(IN_RANGE(SF32LB_USB_DIV_VALUE(0), 1, 7),
	     "USB clock divider must be in the range [1, 7]");

BUILD_ASSERT(!SF32LB_USB_CLK_REQUIRES_DLL2(0) || DT_NODE_HAS_STATUS(DT_INST_CHILD(0, dll2), okay),
	     "USB clock selection requires the dll2 node to be enabled when set to DLL2");

static const struct clock_control_sf32lb_rcc_config config = {
	.base = DT_REG_ADDR(DT_INST_PARENT(0)),
	.cfg = DT_REG_ADDR(DT_INST_PHANDLE(0, sifli_cfg)),
	.pmuc = DT_REG_ADDR(DT_INST_PHANDLE(0, sifli_pmuc)),
	.hxt48 = DEVICE_DT_GET_OR_NULL(DT_INST_CLOCKS_CTLR_BY_NAME(0, hxt48)),
	.hdiv = DT_INST_PROP(0, sifli_hdiv),
	.pdiv1 = DT_INST_PROP(0, sifli_pdiv1),
	.pdiv2 = DT_INST_PROP(0, sifli_pdiv2),
	.sys_clk_src = SF32LB_SYS_CLK_SRC_VALUE(0),
	.peri_clk_src = SF32LB_PERI_CLK_SRC_VALUE(0),
	.mpi1_clk_src = SF32LB_MPI1_CLK_SRC_VALUE(0),
	.mpi2_clk_src = SF32LB_MPI2_CLK_SRC_VALUE(0),
	.usb_clk_src = SF32LB_USB_CLK_SRC_VALUE(0),
	.usb_div = SF32LB_USB_DIV_VALUE(0),
	.peri_clk_freq = SF32LB_PERI_CLK_FREQ(0),
	.hrc48_freq = SF32LB_CLOCK_FREQ_BY_NAME(0, hrc48),
	.hxt48_freq = SF32LB_CLOCK_FREQ_BY_NAME(0, hxt48),
	.dll1_freq = COND_CODE_1(DT_NODE_HAS_STATUS(DT_INST_CHILD(0, dll1), okay),
				 (DT_PROP(DT_INST_CHILD(0, dll1), clock_frequency)), (0U)),
		 .dll2_freq = COND_CODE_1(DT_NODE_HAS_STATUS(DT_INST_CHILD(0, dll2), okay),
				  (DT_PROP(DT_INST_CHILD(0, dll2), clock_frequency)), (0U)),
};

DEVICE_DT_INST_DEFINE(0, clock_control_sf32lb_rcc_init, NULL, NULL, &config, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_sf32lb_rcc_api);
