/*
 * Copyright (c) 2025 Core Devices LLC
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/cache.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

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

/*
 * Early XIP quad-mode setup. Boot ROM leaves the AHB read path (HRCCR /
 * HCMDR) in single-line mode, so all XIP until now has been 1-bit. Here we
 * enable the flash quad-enable bit (via single-line register commands),
 * optionally switch to 4-byte addressing and then reprogram HRCCR/HCMDR for
 * quad reads, so XIP runs 4-bit from this point on. Must happen before the
 * early clock switch so the faster clock is only ever used with quad reads.
 */

#define SF32LB_SPI_NOR_CMD_WREN   0x06U
#define SF32LB_SPI_NOR_CMD_WRSR   0x01U
#define SF32LB_SPI_NOR_CMD_RDSR   0x05U
#define SF32LB_SPI_NOR_CMD_WRSR2  0x31U
#define SF32LB_SPI_NOR_CMD_RDSR2  0x35U
#define SF32LB_SPI_NOR_CMD_4BA    0xB7U
#define SF32LB_SPI_NOR_CMD_4READ      0xEBU
#define SF32LB_SPI_NOR_CMD_4READ_4B   0xECU

/* QE (quad enable) requirement encodings, values match JESD216 SFDP DW15 */
#define SF32LB_QSPI_QER_VAL_NONE   0U
#define SF32LB_QSPI_QER_VAL_S2B1v1 1U
#define SF32LB_QSPI_QER_VAL_S1B6   2U
#define SF32LB_QSPI_QER_VAL_S2B1v4 4U
#define SF32LB_QSPI_QER_VAL_S2B1v5 5U
#define SF32LB_QSPI_QER_VAL_S2B1v6 6U

#define MPI_CCRX_IMODE_SINGLE  FIELD_PREP(MPI_CCR1_IMODE_Msk, 1U)
#define MPI_CCRX_ADMODE_NONE   FIELD_PREP(MPI_CCR1_ADMODE_Msk, 0U)
#define MPI_CCRX_ADMODE_SINGLE FIELD_PREP(MPI_CCR1_ADMODE_Msk, 1U)
#define MPI_CCRX_ADMODE_QUAD   FIELD_PREP(MPI_CCR1_ADMODE_Msk, 3U)
#define MPI_CCRX_ADSIZE_N(n)   FIELD_PREP(MPI_CCR1_ADSIZE_Msk, (n) - 1U)
#define MPI_CCRX_ABMODE_NONE   FIELD_PREP(MPI_CCR1_ABMODE_Msk, 0U)
#define MPI_CCRX_ABMODE_SINGLE FIELD_PREP(MPI_CCR1_ABMODE_Msk, 1U)
#define MPI_CCRX_ABMODE_QUAD   FIELD_PREP(MPI_CCR1_ABMODE_Msk, 3U)
#define MPI_CCRX_ABSIZE_N(n)   FIELD_PREP(MPI_CCR1_ABSIZE_Msk, (n) - 1U)
#define MPI_CCRX_DCYC_N(n)     FIELD_PREP(MPI_CCR1_DCYC_Msk, (n))
#define MPI_CCRX_DMODE_NONE    FIELD_PREP(MPI_CCR1_DMODE_Msk, 0U)
#define MPI_CCRX_DMODE_SINGLE  FIELD_PREP(MPI_CCR1_DMODE_Msk, 1U)
#define MPI_CCRX_DMODE_QUAD    FIELD_PREP(MPI_CCR1_DMODE_Msk, 3U)
#define MPI_CCRX_FMODE_READ    FIELD_PREP(MPI_CCR1_FMODE_Msk, 0U)
#define MPI_CCRX_FMODE_WRITE   FIELD_PREP(MPI_CCR1_FMODE_Msk, 1U)

#define MPI_CCRX_CMD_RDSR (MPI_CCRX_IMODE_SINGLE | MPI_CCRX_DMODE_SINGLE)
#define MPI_CCRX_CMD_WRSR (MPI_CCRX_IMODE_SINGLE | MPI_CCRX_DMODE_SINGLE | MPI_CCRX_FMODE_WRITE)
#define MPI_CCRX_CMD_4READ                                                                      \
	(MPI_CCRX_IMODE_SINGLE | MPI_CCRX_ADMODE_QUAD | MPI_CCRX_ADSIZE_N(3U) |                  \
	 MPI_CCRX_ABMODE_QUAD | MPI_CCRX_ABSIZE_N(1U) | MPI_CCRX_DCYC_N(4U) | MPI_CCRX_DMODE_QUAD)
#define MPI_CCRX_CMD_4READ_4B                                                                   \
	(MPI_CCRX_IMODE_SINGLE | MPI_CCRX_ADMODE_QUAD | MPI_CCRX_ADSIZE_N(4U) |                  \
	 MPI_CCRX_ABMODE_QUAD | MPI_CCRX_ABSIZE_N(1U) | MPI_CCRX_DCYC_N(4U) | MPI_CCRX_DMODE_QUAD)

#define SF32LB_QSPI_MAX_3B_SIZE 0x1000000UL
#define SF32LB_QSPI_IS_QUAD(n) (DT_PROP_OR(DT_NODELABEL(n), sifli_lines, 1U) == 4U)
#define SF32LB_QSPI_NEEDS_4B_ADDR(n)                                                           \
	((DT_PROP(DT_CHILD(DT_NODELABEL(n), flash_0), size) / 8U) > SF32LB_QSPI_MAX_3B_SIZE)
#define SF32LB_QSPI_QER(n)                                                                      \
	_CONCAT(SF32LB_QSPI_QER_VAL_,                                                             \
		DT_STRING_TOKEN_OR(DT_CHILD(DT_NODELABEL(n), flash_0), quad_enable_requirements, NONE))

static __ramfunc void sf32lb52x_qspi_memcpy(void *dst, const void *src, size_t len)
{
	const uint8_t *csrc = src;
	uint8_t *cdst = dst;

	while (len--) {
		*cdst++ = *csrc++;
	}
}

static __ramfunc void sf32lb52x_qspi_cinstr(MPI_TypeDef *mpi, uint8_t cmd)
{
	ll_mpi_write_command_config(mpi, LL_MPI_CS_1, MPI_CCRX_IMODE_SINGLE);
	ll_mpi_set_command_byte(mpi, LL_MPI_CS_1, cmd);

	while (!ll_mpi_get_transfer_complete_flag(mpi)) {
	}

	ll_mpi_clear_transfer_complete_flag(mpi);
}

static __ramfunc void sf32lb52x_qspi_rdsr(MPI_TypeDef *mpi, uint8_t cmd, uint8_t *sr)
{
	uint32_t dr;

	ll_mpi_write_command_config(mpi, LL_MPI_CS_1, MPI_CCRX_CMD_RDSR);
	ll_mpi_set_data_length(mpi, LL_MPI_CS_1, 1U);
	ll_mpi_set_address(mpi, LL_MPI_CS_1, 0U);
	ll_mpi_set_command_byte(mpi, LL_MPI_CS_1, cmd);

	while (!ll_mpi_get_transfer_complete_flag(mpi)) {
	}

	ll_mpi_clear_transfer_complete_flag(mpi);

	dr = ll_mpi_read_data(mpi);
	*sr = (uint8_t)dr;
}

static __ramfunc void sf32lb52x_qspi_wrsr(MPI_TypeDef *mpi, uint8_t cmd, const uint8_t *sr,
					  size_t len)
{
	uint32_t dr = 0U;

	sf32lb52x_qspi_memcpy(&dr, sr, len);

	/* push data to FIFO */
	ll_mpi_set_data_length(mpi, LL_MPI_CS_1, len);
	ll_mpi_write_data(mpi, dr);

	/* WREN */
	sf32lb52x_qspi_cinstr(mpi, SF32LB_SPI_NOR_CMD_WREN);

	/* WRSR, wait for WIP to clear via CMD2 status match */
	ll_mpi_write_command_config(mpi, LL_MPI_CS_2, MPI_CCRX_CMD_RDSR);
	ll_mpi_set_command_byte(mpi, LL_MPI_CS_2, SF32LB_SPI_NOR_CMD_RDSR);
	ll_mpi_set_data_length(mpi, LL_MPI_CS_2, 1U);
	ll_mpi_set_status_match_mask(mpi, BIT(0));
	ll_mpi_set_status_match_value(mpi, 0U);
	ll_mpi_enable_command2(mpi);
	ll_mpi_set_status_match_enable(mpi, LL_MPI_CS_2, 1U);

	ll_mpi_write_command_config(mpi, LL_MPI_CS_1, MPI_CCRX_CMD_WRSR);
	ll_mpi_set_command_byte(mpi, LL_MPI_CS_1, cmd);

	while (!ll_mpi_get_status_match_flag(mpi)) {
	}

	ll_mpi_clear_status_match_flag(mpi);
	ll_mpi_clear_transfer_complete_flag(mpi);
	ll_mpi_disable_command2(mpi);
	ll_mpi_set_status_match_enable(mpi, LL_MPI_CS_2, 0U);
}

/*
 * The whole switch (QE + HRCCR/HCMDR) runs from RAM: once HRCCR is
 * reprogrammed for quad reads, any flash fetch would already use the new
 * (possibly still unproven) configuration. Keeping this sequence in RAM
 * makes the transition itself deterministic.
 */
static __ramfunc void sf32lb52x_early_xip_quad_init(void)
{
#if DT_NODE_HAS_STATUS(DT_NODELABEL(mpi2), okay) &&                                                  \
	DT_NODE_HAS_COMPAT(DT_NODELABEL(mpi2), sifli_sf32lb_mpi_qspi_nor)
	MPI_TypeDef *mpi = (MPI_TypeDef *)DT_REG_ADDR_BY_NAME(DT_NODELABEL(mpi2), ctrl);
	uint8_t sr[2];
	uint32_t ccrx_read;
	uint8_t cmd_read;
	bool need_4b = SF32LB_QSPI_NEEDS_4B_ADDR(mpi2);
	uint8_t qer = SF32LB_QSPI_QER(mpi2);

	/* Single-line XIP is fine; quad-mode requires QE + HRCCR/HCMDR below. */
	if (!SF32LB_QSPI_IS_QUAD(mpi2)) {
		return;
	}

	/* Enable the flash quad-enable bit using single-line register commands.
	 * The helpers run from RAM so instruction fetches do not race the MPI
	 * command path while XIP is still single-line.
	 */
	switch (qer) {
	case SF32LB_QSPI_QER_VAL_S1B6:
		sf32lb52x_qspi_rdsr(mpi, SF32LB_SPI_NOR_CMD_RDSR, &sr[0]);
		if ((sr[0] & BIT(6)) == 0U) {
			sr[0] |= BIT(6);
			sf32lb52x_qspi_wrsr(mpi, SF32LB_SPI_NOR_CMD_WRSR, &sr[0], 1U);
		}
		break;
	case SF32LB_QSPI_QER_VAL_S2B1v1:
	case SF32LB_QSPI_QER_VAL_S2B1v4:
	case SF32LB_QSPI_QER_VAL_S2B1v5:
		sf32lb52x_qspi_rdsr(mpi, SF32LB_SPI_NOR_CMD_RDSR, &sr[0]);
		sf32lb52x_qspi_rdsr(mpi, SF32LB_SPI_NOR_CMD_RDSR2, &sr[1]);
		if ((sr[1] & BIT(1)) == 0U) {
			sr[1] |= BIT(1);
			sf32lb52x_qspi_wrsr(mpi, SF32LB_SPI_NOR_CMD_WRSR, sr, 2U);
		}
		break;
	case SF32LB_QSPI_QER_VAL_S2B1v6:
		sf32lb52x_qspi_rdsr(mpi, SF32LB_SPI_NOR_CMD_RDSR2, &sr[0]);
		if ((sr[0] & BIT(1)) == 0U) {
			sr[0] |= BIT(1);
			sf32lb52x_qspi_wrsr(mpi, SF32LB_SPI_NOR_CMD_WRSR2, &sr[0], 1U);
		}
		break;
	default:
		/* Unknown/unsupported QE requirement: keep single-line XIP. */
		return;
	}

	/* Enter 4-byte address mode for devices larger than 16 MB. */
	if (need_4b) {
		sf32lb52x_qspi_cinstr(mpi, SF32LB_SPI_NOR_CMD_4BA);
	}

	/* Switch the AHB XIP read path to quad mode. */
	if (need_4b) {
		ccrx_read = MPI_CCRX_CMD_4READ_4B;
		cmd_read = SF32LB_SPI_NOR_CMD_4READ_4B;
	} else {
		ccrx_read = MPI_CCRX_CMD_4READ;
		cmd_read = SF32LB_SPI_NOR_CMD_4READ;
	}
	ll_mpi_set_ahb_read_config(mpi, ccrx_read);
	ll_mpi_set_ahb_read_command(mpi, cmd_read);
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

	/* switch XIP to quad reads (QE + HRCCR/HCMDR) before clock boost */
	sf32lb52x_early_xip_quad_init();

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
