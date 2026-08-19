.. zephyr:board:: sf32lb52_nano_n16r16

Overview

SF32LB52-DevKit-Nano-N16R16 is a compact development board based on the SF32LB52x
series SoC. It is mainly used for developing and evaluating applications
based on the SF32LB52x series chip.

More information about the board can be found at the
SF32LB52-DevKit-Nano-N16R16 website_.

Hardware

SF32LB52-DevKit-Nano-N16R16 provides the following hardware components:

SF32LB52x SoC

  ARM Cortex-M33 processor
  48MHz crystal
  32.768KHz crystal
  16MB PSRAM
  16MB QSPI-NOR flash
  Bluetooth Low Energy and 2.4GHz wireless connectivity

Memory

  On-chip SRAM
  16MB PSRAM
  16MB external QSPI-NOR flash

GPIO

  LCC half-hole supports 17 GPIOs
  LGA pins support 13 GPIOs
  Supports common peripheral interfaces such as UART, SPI, I2C and PWM

USB

  Type-C interface with CH340N serial chip for power supply,
    program download and software debugging
  USB2.0 FS interface exposed through LCC half-holes

Display

  FPC 16P, 0.5mm pitch connector for display expansion
  Supports SPI/DSPI/Quad SPI display interface with DDR mode

Audio

  Supports audio ADC input (analog or silicon microphone)
  Supports PDM digital microphone input
  Analog audio output (requires external audio PA)

Buttons

  1x Function button
  1x Power button (supports long press 10s reset)

LEDs

  2x LEDs, GPIO controlled

Debug

  Onboard debug/programming interface
  Supports software download and debugging through the USB interface

Supported Features
==================

.. zephyr:board-supported-hw::

Programming and Debugging

.. zephyr:board-supported-runners::

Refer to sftool website_ for more information.

References

.. target-notes::

.. _SF32LB52-DevKit-Nano-N16R16 website:
  https://wiki.sifli.com/board/sf32lb52x/SF32LB52-DevKit-Nano.html#

.. _sftool website:
   https://github.com/OpenSiFli/sftool
