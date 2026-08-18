.. zephyr:board:: sf32lb52_core_n4

Overview
********

SF32LB52-Core-N4 is a compact development board based on the SF32LB52x
series SoC. It is mainly used for developing and evaluating applications
based on the SF32LB52x series chip.

More information about the board can be found at the
`SF32LB52-Core-N4 website`_.

Hardware
********

SF32LB52-Core-N4 provides the following hardware components:

- SF32LB52BU56 SoC

  - ARM Cortex-M33 processor
  - 48MHz crystal
  - 32.768KHz crystal
  - 4Mb QSPI-NOR @ 96MHz
  - Bluetooth Low Energy and 2.4GHz wireless connectivity

- Memory

  - On-chip SRAM
  - External QSPI-NOR flash

- GPIO

  - GPIO pins are available through the board headers
  - Supports common peripheral interfaces such as UART, SPI, I2C and PWM

- USB

  - Type-C interface for power supply
  - USB interface for programming and debugging

- SD card

  - Supports TF cards using SPI interface, onboard Micro SD card slot.

- Debug

  - Onboard debug/programming interface
  - Supports software download and debugging through the USB interface

Supported Features
==================

.. zephyr:board-supported-hw::

Programming and Debugging
*************************

.. zephyr:board-supported-runners::

Refer to `sftool website`_ for more information.

References
**********

.. target-notes::

.. _SF32LB52-Core-N4 website:
   https://wiki.sifli.com/en/board/sf32lb52x/SF32LB52-DevKit-Core-3p3.html

.. _sftool website:
   https://github.com/OpenSiFli/sftool