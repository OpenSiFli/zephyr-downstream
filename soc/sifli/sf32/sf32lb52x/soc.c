/*
 * Copyright (c) 2025 Core Devices LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/cache.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

#include <ll_hpsys_aon.h>
#include <ll_hpsys_cfg.h>
#include <ll_mpi.h>
#include <ll_pmuc.h>
#include <ll_rcc.h>

#define BOOTROM_BKP_REG             DT_REG_ADDR(DT_INST(0, sifli_sf32lb_rtc_backup))
#define BOOTROM_FLASH_OFF_DELAY_MSK GENMASK(11U, 4U)
#define BOOTROM_FLASH_ON_DELAY_MSK  GENMASK(23U, 12U)

/*
 * Apply the DT-prescribed MPI (Memory Peripheral Interface) settings before
 * any clock source change. When the early clock configuration switches the
 * MPI clock (CSR.SEL_MPIx), the prescaler and RX clock tuning must already be
 * in place so that XIP and memory accesses use the final divider/sampling.
 * The flash driver re-applies the same values at PRE_KERNEL_1, so this is
 * idempotent.
 */
static void sf32lb52x_early_mpi_init(void)
{
#if DT_NODE_HAS_STATUS(DT_NODELABEL(mpi2), okay) &&                                                  \
	DT_NODE_HAS_COMPAT(DT_NODELABEL(mpi2), sifli_sf32lb_mpi_qspi_nor)
	MPI_TypeDef *mpi = (MPI_TypeDef *)DT_REG_ADDR_BY_NAME(DT_NODELABEL(mpi2), ctrl);

	/* MCLK = FCLK / PSCLR (0 selects /1), see sifli,sf32lb-mpi-qspi-nor binding */
	ll_mpi_set_clock_div(mpi, (uint8_t)DT_PROP_OR(DT_NODELABEL(mpi2), sifli_psclr, 4U));
	ll_mpi_set_rx_clk_invert(mpi, DT_PROP_OR(DT_NODELABEL(mpi2), sifli_invert_rx_clk, 0U));
#endif
}

#if DT_HAS_COMPAT_STATUS_OKAY(sifli_sf32lb_rcc_clk)

#define SF32LB_RCC_CLK_NODE   DT_NODELABEL(rcc_clk)
#define SF32LB_RCC_BASE       DT_REG_ADDR(DT_PARENT(SF32LB_RCC_CLK_NODE))
#define SF32LB_CFG_BASE       DT_REG_ADDR(DT_PHANDLE(SF32LB_RCC_CLK_NODE, sifli_cfg))
#define SF32LB_PMUC_BASE      DT_REG_ADDR(DT_PHANDLE(SF32LB_RCC_CLK_NODE, sifli_pmuc))
#define SF32LB_AON_BASE       DT_REG_ADDR(DT_NODELABEL(aon))

/*
 * DT enum order follows the binding (sifli,sf32lb-rcc-clk.yaml); the
 * CSR.SEL_SYS register encoding differs for DLL1 (3), hence the mapping.
 */
#define SF32LB_SYS_CLKSRC_LL(node)                                                              \
	((DT_ENUM_IDX_OR(node, sifli_sys_clk_src, 0U) == 1U) ? LL_RCC_SYS_CLKSRC_HXT48 :          \
	 (DT_ENUM_IDX_OR(node, sifli_sys_clk_src, 0U) == 2U) ? LL_RCC_SYS_CLKSRC_DLL1  :          \
							       LL_RCC_SYS_CLKSRC_HRC48)
#define SF32LB_PERI_CLKSRC_LL(node)                                                             \
	(DT_ENUM_IDX_OR(node, sifli_peri_clk_src, 1U) == 0U ? LL_RCC_PERI_CLKSRC_HRC48 :          \
							      LL_RCC_PERI_CLKSRC_HXT48)
/* MPI/USB DT enum order matches the CSR.SEL_* register encoding. */
#define SF32LB_MPI1_CLKSRC_LL(node) DT_ENUM_IDX_OR(node, sifli_mpi1_clk_src, 0U)
#define SF32LB_MPI2_CLKSRC_LL(node) DT_ENUM_IDX_OR(node, sifli_mpi2_clk_src, 0U)
#define SF32LB_USB_CLKSRC_LL(node)  DT_ENUM_IDX_OR(node, sifli_usb_clk_src, 0U)
#define SF32LB_USB_DIV_LL(node)     DT_PROP_OR(node, sifli_usb_div, 4U)

#define SF32LB_DLL_FREQ(node)                                                                   \
	COND_CODE_1(DT_NODE_HAS_STATUS(node, okay), (DT_PROP(node, clock_frequency)), (0U))

static void sf32lb52x_configure_dll(HPSYS_RCC_TypeDef *rcc, uint32_t freq, uint32_t dll_idx)
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

/*
 * Bring the clock tree up to the DT parameters before the kernel and its
 * device init run, so XIP runs at the configured frequency from the start.
 * Runs from XIP; the MPI prescaler/sampling must already be set (see
 * sf32lb52x_early_mpi_init) so flash accesses survive the clock switch.
 * The RCC clock-control driver re-applies the same values at PRE_KERNEL_1,
 * which is safe (it switches to HXT48 before touching the DLLs).
 */
static void sf32lb52x_early_clock_init(void)
{
	HPSYS_RCC_TypeDef *rcc = (HPSYS_RCC_TypeDef *)SF32LB_RCC_BASE;
	HPSYS_AON_TypeDef *aon = (HPSYS_AON_TypeDef *)SF32LB_AON_BASE;
	HPSYS_CFG_TypeDef *cfg = (HPSYS_CFG_TypeDef *)SF32LB_CFG_BASE;
	PMUC_TypeDef *pmuc = (PMUC_TypeDef *)SF32LB_PMUC_BASE;
	uint32_t dll1_freq = SF32LB_DLL_FREQ(DT_NODELABEL(dll1));
	uint32_t dll2_freq = SF32LB_DLL_FREQ(DT_NODELABEL(dll2));

	/* HXT48: boot ROM enables it at reset; ensure and wait ready (pre-device) */
	ll_aon_hxt48_req_set(aon, LL_AON_PM_ACTIVE);
	while (!ll_aon_hxt48_is_ready(aon)) {
	}

	if (dll1_freq != 0U || dll2_freq != 0U) {
		/* DLL buffer and HPBG rail are prerequisites for the DLLs */
		ll_pmuc_hxt48_enable_dll_buf(pmuc);
		ll_cfg_hpbg_enable(cfg);

		/* Switch sys/peri to HXT48 before (re)configuring the DLLs */
		ll_rcc_set_sys_clock_source(rcc, LL_RCC_SYS_CLKSRC_HXT48);
		ll_rcc_set_peri_clock_source(rcc, LL_RCC_PERI_CLKSRC_HXT48);

		if (dll1_freq != 0U) {
			sf32lb52x_configure_dll(rcc, dll1_freq, LL_RCC_DLL_INDEX_1);
		}

		if (dll2_freq != 0U) {
			sf32lb52x_configure_dll(rcc, dll2_freq, LL_RCC_DLL_INDEX_2);
		}
	}

	/* Dividers */
	ll_rcc_set_ahb_div(rcc, DT_PROP(SF32LB_RCC_CLK_NODE, sifli_hdiv));
	ll_rcc_set_apb1_div(rcc, DT_PROP(SF32LB_RCC_CLK_NODE, sifli_pdiv1));
	ll_rcc_set_apb2_div(rcc, DT_PROP(SF32LB_RCC_CLK_NODE, sifli_pdiv2));

	/* Final clock sources and USB divider */
	ll_rcc_set_sys_clock_source(rcc, SF32LB_SYS_CLKSRC_LL(SF32LB_RCC_CLK_NODE));
	ll_rcc_set_peri_clock_source(rcc, SF32LB_PERI_CLKSRC_LL(SF32LB_RCC_CLK_NODE));
	ll_rcc_set_mpi1_clock_source(rcc, SF32LB_MPI1_CLKSRC_LL(SF32LB_RCC_CLK_NODE));
	ll_rcc_set_mpi2_clock_source(rcc, SF32LB_MPI2_CLKSRC_LL(SF32LB_RCC_CLK_NODE));
	ll_rcc_set_usb_clock_source(rcc, SF32LB_USB_CLKSRC_LL(SF32LB_RCC_CLK_NODE));
	ll_rcc_set_usb_div(rcc, SF32LB_USB_DIV_LL(SF32LB_RCC_CLK_NODE));
}

#endif /* DT_HAS_COMPAT_STATUS_OKAY(sifli_sf32lb_rcc_clk) */

void soc_early_init_hook(void)
{
	sys_cache_instr_enable();
	sys_cache_data_enable();

	/* configure MPI before any clock source change (XIP safety) */
	sf32lb52x_early_mpi_init();

#if DT_HAS_COMPAT_STATUS_OKAY(sifli_sf32lb_rcc_clk)
	/* bring the clock tree up to the DT parameters before kernel init */
	sf32lb52x_early_clock_init();
#endif

#if CONFIG_SF32LB52X_BOOTROM_FLASH_ON_DELAY_MS > 0 ||                                              \
	CONFIG_SF32LB52X_BOOTROM_FLASH_OFF_DELAY_MS > 0
	uint32_t val;

	val = sys_read32(BOOTROM_BKP_REG);
	val &= ~(BOOTROM_FLASH_OFF_DELAY_MSK | BOOTROM_FLASH_ON_DELAY_MSK);
	val |= FIELD_PREP(BOOTROM_FLASH_OFF_DELAY_MSK,
			  CONFIG_SF32LB52X_BOOTROM_FLASH_OFF_DELAY_MS) |
	       FIELD_PREP(BOOTROM_FLASH_ON_DELAY_MSK, CONFIG_SF32LB52X_BOOTROM_FLASH_ON_DELAY_MS);
	sys_write32(val, BOOTROM_BKP_REG);
#endif
}
