/*
 * Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/toolchain.h>

#include <ll_pmuc.h>

#define PMUC_NODE DT_NODELABEL(pmuc)

void z_sys_poweroff(void)
{
	PMUC_TypeDef *pmuc = (PMUC_TypeDef *)DT_REG_ADDR(PMUC_NODE);

	pmuc->WCR = pmuc->WSR;

	ll_pmuc_clear_hibernate_flag(pmuc);
	k_busy_wait(100 * USEC_PER_MSEC);
	ll_pmuc_enter_hibernate(pmuc);

	while (1) {
	}

	CODE_UNREACHABLE;
}
