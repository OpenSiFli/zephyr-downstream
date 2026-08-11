# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(wdt_basic_api)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

# Back the wdt test's cross-reset state at a fixed SRAM address that the
# SFBL bootloader does not re-initialise (see boards/sf32lb52_devkit_lcd.ld).
if(CONFIG_BOARD_SF32LB52_DEVKIT_LCD)
       zephyr_linker_sources(SECTIONS boards/sf32lb52_devkit_lcd.ld)
endif()