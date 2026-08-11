/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * The SFBL bootloader re-initialises the lower SRAM on every boot, which
 * destroys the wdt_basic_api test's noinit state (m_state, m_testcase_index,
 * m_testvalue, m_magic). Without it the test can never detect that the
 * watchdog reset happened and loops forever waiting for one.
 *
 * Workaround: back the cross-reset state in a dedicated NOLOAD section at a
 * fixed SRAM address (ram + 256 KiB, 0x20040000) that the bootloader does
 * not touch. If the application image grows past 256 KiB the linker will
 * report an overlap error - move the address up in that case.
 */
.wdt_test_state 0x20040000 (NOLOAD) :
{
	KEEP(*(.wdt_test_state))
}
