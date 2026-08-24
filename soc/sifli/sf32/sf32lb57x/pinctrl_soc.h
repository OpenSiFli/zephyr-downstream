/*
 * Copyright (c) 2025 Core Devices LLC
 * Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_SIFLI_SF32_SF32LB57X_PINCTRL_SOC_H_
#define _SOC_SIFLI_SF32_SF32LB57X_PINCTRL_SOC_H_

#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/dt-bindings/pinctrl/sf32lb57x-pinctrl.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Type for SF32LB57x pin.
 *
 * The encoding mirrors the SF32LB57x HPSYS_PINMUX PAD register layout so that
 * the pinctrl driver can pass the configuration bits straight to the LL API:
 * - 0-7:   Function select (8 bits)
 * - 8:     Pull enable (PE)
 * - 9:     Pull select (PS, 0=pulldown, 1=pullup)
 * - 10:    Input enable (IE)
 * - 11:    Input select (IS, 0=CMOS, 1=Schmitt)
 * - 12:    Slew rate (SR, 0=fast, 1=slow)
 * - 13-14: Drive strength (DS0/DS1)
 * - 16-17: Port (SA, PA, ...)
 * - 18-31: Pad
 */
typedef uint32_t pinctrl_soc_pin_t;

#define SF32LB_PE_MSK BIT(8U)
#define SF32LB_PS_MSK BIT(9U)
#define SF32LB_IE_MSK BIT(10U)
#define SF32LB_IS_MSK BIT(11U)
#define SF32LB_SR_MSK BIT(12U)
#define SF32LB_DS_MSK GENMASK(14U, 13U)

/* Drive strength enum index position and mask (stored in bits 13-14) */
#define SF32LB_DS_IDX_POS 13U
#define SF32LB_DS_IDX_MSK GENMASK(14U, 13U)

/*
 * Pin configuration mask for bits that should be modified.
 * The 57x has no PINR remap, so only the PAD register fields are touched.
 */
#define SF32LB_PINMUX_CFG_MSK                                                                      \
	(SF32LB_FSEL_MSK | SF32LB_PE_MSK | SF32LB_PS_MSK | SF32LB_IE_MSK | SF32LB_IS_MSK |       \
	 SF32LB_SR_MSK | SF32LB_DS_MSK)

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                               \
	(DT_PROP_BY_IDX(node_id, prop, idx) |                                                      \
	 FIELD_PREP(SF32LB_PE_MSK,                                                                 \
		    (DT_PROP(node_id, bias_pull_up) | DT_PROP(node_id, bias_pull_down))) |         \
	 FIELD_PREP(SF32LB_PS_MSK, DT_PROP(node_id, bias_pull_up)) |                               \
	 FIELD_PREP(SF32LB_IE_MSK, DT_PROP(node_id, input_enable)) |                               \
	 FIELD_PREP(SF32LB_IS_MSK, DT_PROP(node_id, input_schmitt_enable)) |                       \
	 COND_CODE_0(DT_PROP(node_id, slew_rate), (SF32LB_SR_MSK), (0U)) |                         \
	 FIELD_PREP(SF32LB_DS_IDX_MSK, DT_ENUM_IDX(node_id, drive_strength))),

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop), DT_FOREACH_PROP_ELEM, pinmux,           \
				Z_PINCTRL_STATE_PIN_INIT)}

#ifdef __cplusplus
}
#endif

#endif /* _SOC_SIFLI_SF32_SF32LB57X_PINCTRL_SOC_H_ */