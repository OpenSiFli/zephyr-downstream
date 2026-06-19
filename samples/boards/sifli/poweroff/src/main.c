/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/sys/poweroff.h>

int main(void)
{
	int rc;

	printk("\n%s system off demo\n", CONFIG_BOARD);

	sys_poweroff();

	return 0;
}
