/*
 * Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 *
 * SF32LB57x pin control helpers.
 *
 * The SF32LB57x pinmux register layout differs from the SF32LB52x:
 *  - FSEL is 8 bits wide (bits 0-7) instead of 4 bits
 *  - The pull/input/slew/drive configuration bits sit at bits 8-15
 *    (PE@8, PS@9, IE@10, IS@11, SR@12, DS0@13, DS1@14)
 *  - There is no HPSYS_CFG PINR remap mechanism on the 57x
 *
 * Function numbers follow the SF32LB57x SDK bf0_pin_const.h:
 *  - dedicated functions use their per-pad FSEL value
 *  - arbitrary (remappable) functions (I2C, USART matrix, ...) use the
 *    function enum value itself as FSEL (e.g. I2C1_SCL = 16, I2C1_SDA = 17)
 */

#ifndef _INCLUDE_ZEPHYR_DT_BINDINGS_PINCTRL_SF32LB57X_PINCTRL_H_
#define _INCLUDE_ZEPHYR_DT_BINDINGS_PINCTRL_SF32LB57X_PINCTRL_H_

/** @cond INTERNAL_HIDDEN */

/* FSEL (8 bits, mirrors the 57x pinmux register bits 0-7) */
#define SF32LB_FSEL_POS  0U
#define SF32LB_FSEL_MSK  0x000000FFU

/* Port */
#define SF32LB_PORT_POS  16U
#define SF32LB_PORT_MSK  0x00030000U

/* Pad */
#define SF32LB_PAD_POS   18U
#define SF32LB_PAD_MSK   0x0FFC0000U

/* No PINR remap registers exist on the 57x. */
#define SF32LB_PINR_FIELD_POS  22U
#define SF32LB_PINR_FIELD_MSK  0U
#define SF32LB_PINR_OFFSET_POS 24U
#define SF32LB_PINR_OFFSET_MSK 0U

/** @endcond */

/**
 * @brief Pin configuration bit field.
 *
 * Bitmap:
 * - 0-7:   Function select (8 bits, matches the 57x pinmux register)
 * - 8:     Pull enable (PE)
 * - 9:     Pull select (PS)
 * - 10:    Input enable (IE)
 * - 11:    Input select / Schmitt (IS)
 * - 12:    Slew rate (SR)
 * - 13-14: Drive strength (DS0/DS1)
 * - 16-17: Port (SA, PA)
 * - 18-31: Pad
 *
 * @param port Port
 * @param pad Pad
 * @param fsel Function select
 */
#define SF32LB_PINMUX(port, pad, fsel)                                                             \
	((((port) << SF32LB_PORT_POS) & SF32LB_PORT_MSK) |                                         \
	 (((pad) << SF32LB_PAD_POS) & SF32LB_PAD_MSK) |                                            \
	 (((fsel) << SF32LB_FSEL_POS) & SF32LB_FSEL_MSK))

/* ports */
#define SF32LB_PORT_SA 0U
#define SF32LB_PORT_PA 1U

/*
 * Pin macros. FSEL values are taken from the SF32LB57x SDK
 * bf0_pin_const.h / bf0_pin_const.c function tables.
 */

/* PA18 */
#define PA18_USART1_RXD        SF32LB_PINMUX(SF32LB_PORT_PA, 18U, 1U)

/* PA19 */
#define PA19_USART1_TXD        SF32LB_PINMUX(SF32LB_PORT_PA, 19U, 1U)

/* PA24 */
#define PA24_SPI1_DIO          SF32LB_PINMUX(SF32LB_PORT_PA, 24U, 1U)

/* PA25 */
#define PA25_SPI1_DI           SF32LB_PINMUX(SF32LB_PORT_PA, 25U, 1U)

/* PA28 */
#define PA28_SPI1_CLK          SF32LB_PINMUX(SF32LB_PORT_PA, 28U, 1U)

/* PA29 */
#define PA29_SPI1_CS           SF32LB_PINMUX(SF32LB_PORT_PA, 29U, 1U)

/* PA45 / PA46: I2C1 (arbitrary function, FSEL = function enum value) */
#define PA45_I2C1_SDA          SF32LB_PINMUX(SF32LB_PORT_PA, 45U, 17U)
#define PA46_I2C1_SCL          SF32LB_PINMUX(SF32LB_PORT_PA, 46U, 16U)

/* LCDC1 QSPI (dedicated functions, FSEL 1) */
#define PA02_LCDC1_SPI_TE      SF32LB_PINMUX(SF32LB_PORT_PA, 2U, 1U)
#define PA03_LCDC1_SPI_CS      SF32LB_PINMUX(SF32LB_PORT_PA, 3U, 1U)
#define PA04_LCDC1_SPI_CLK     SF32LB_PINMUX(SF32LB_PORT_PA, 4U, 1U)
#define PA05_LCDC1_SPI_DIO0    SF32LB_PINMUX(SF32LB_PORT_PA, 5U, 1U)
#define PA06_LCDC1_SPI_DIO1    SF32LB_PINMUX(SF32LB_PORT_PA, 6U, 1U)
#define PA07_LCDC1_SPI_DIO2    SF32LB_PINMUX(SF32LB_PORT_PA, 7U, 1U)
#define PA08_LCDC1_SPI_DIO3    SF32LB_PINMUX(SF32LB_PORT_PA, 8U, 1U)

/* MPI1 OPI PSRAM (dedicated functions) */
#define SA00_MPI1_PSRAM_DM     SF32LB_PINMUX(SF32LB_PORT_SA, 0U, 1U)
#define SA01_MPI1_PSRAM_DIO0   SF32LB_PINMUX(SF32LB_PORT_SA, 1U, 1U)
#define SA02_MPI1_PSRAM_DIO1   SF32LB_PINMUX(SF32LB_PORT_SA, 2U, 1U)
#define SA03_MPI1_PSRAM_DIO2   SF32LB_PINMUX(SF32LB_PORT_SA, 3U, 1U)
#define SA04_MPI1_PSRAM_DIO3   SF32LB_PINMUX(SF32LB_PORT_SA, 4U, 1U)
#define SA05_MPI1_PSRAM_CS     SF32LB_PINMUX(SF32LB_PORT_SA, 5U, 1U)
#define SA06_MPI1_PSRAM_CLKB   SF32LB_PINMUX(SF32LB_PORT_SA, 6U, 1U)
#define SA07_MPI1_PSRAM_CLK    SF32LB_PINMUX(SF32LB_PORT_SA, 7U, 1U)
#define SA08_MPI1_PSRAM_DIO4   SF32LB_PINMUX(SF32LB_PORT_SA, 8U, 1U)
#define SA09_MPI1_PSRAM_DIO5   SF32LB_PINMUX(SF32LB_PORT_SA, 9U, 1U)
#define SA10_MPI1_PSRAM_DIO6   SF32LB_PINMUX(SF32LB_PORT_SA, 10U, 1U)
#define SA11_MPI1_PSRAM_DIO7   SF32LB_PINMUX(SF32LB_PORT_SA, 11U, 1U)
/* SA12: DQS in pinmap mode 1, DQSDM in mode 2 */
#define SA12_MPI1_PSRAM_DQS    SF32LB_PINMUX(SF32LB_PORT_SA, 12U, 1U)
#define SA12_MPI1_PSRAM_DQSDM  SF32LB_PINMUX(SF32LB_PORT_SA, 12U, 2U)

#endif /* _INCLUDE_ZEPHYR_DT_BINDINGS_PINCTRL_SF32LB57X_PINCTRL_H_ */
