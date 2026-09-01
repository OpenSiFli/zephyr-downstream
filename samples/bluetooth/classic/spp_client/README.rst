.. zephyr:code-sample:: bluetooth_spp_client
   :name: SPP Client
   :relevant-api: bt_rfcomm bt_sdp bluetooth

   Actively connect to a peer board's Bluetooth Classic SPP service over RFCOMM.

Overview
********

This is a shell-driven BR/EDR SPP client for board-to-board testing. It does
not host a serial-port server; instead it actively connects to a peer board
that advertises a custom UUID SPP service (e.g. ``0x3001``) via
``spp search``/``spp connect``, and then supports data transfer, file transfer
and throughput testing over the established RFCOMM channel. Received
file/bulk data is stored on a littlefs partition mounted at ``/lfs``.

Requirements
************

* A board with Bluetooth BR/EDR (Classic) support
* A peer board running the :zephyr:code-sample:`bluetooth_spp_server` sample
  (or any other board exposing a custom UUID Serial Port RFCOMM service)

Building and Running
********************

Build and flash the SPP server sample on a second board, then build and flash
this client:

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/classic/spp_client
   :board: sf32lb52_devkit_lcd/sf32lb525uc6
   :goals: build flash
   :compact:

After boot the serial log prints the local BD address. Use ``spp addr`` to
show it again.

Board-to-Board Connection
*************************

Many phones only support the standard ``0x1101`` SPP, and a board-initiated
connection to a phone is often dropped quickly. For board-to-board use, the
peer board registers a custom UUID (``0x3001``) SPP channel; this client
actively connects to it.

1. Power on the peer (server) board and note its BD address and board SPP
   channel from its serial log::

      My BD address: AB:89:67:45:23:01
      Board SPP (uuid 0x3001) registered on RFCOMM channel 7

2. On this client board, connect to the peer::

      uart:~$ spp addr
      uart:~$ spp search AB:89:67:45:23:01 2 3001
      uart:~$ spp connect AB:89:67:45:23:01 2 3001

   ``spp connect <bd_addr> <uuid_len> <uuid>``: a 16-bit UUID is written as 4
   hex digits (``3001``) without the ``0x`` prefix. Connection is confirmed by
   ``RFCOMM connected`` on both sides.

3. Send data and files::

      uart:~$ spp send_data AB:89:67:45:23:01 7
      uart:~$ spp ls
      uart:~$ spp send_file AB:89:67:45:23:01 7 /lfs/spp_rx_xxxxx.bin

4. Throughput test::

      uart:~$ spp through_put AB:89:67:45:23:01 7 200000

   The sender prints ``through_put done: ...`` and the receiver prints
   ``[FILE RX] done: ... avg speed ... kbps`` when the transfer completes.

Shell Commands
**************

===================== ==================================================
Command               Description
===================== ==================================================
``spp addr``          Print the local BR/EDR address
``spp ls [dir]``      List files under a directory (default ``/lfs``)
``spp send_data <bd> <ch>``  Send a test message to a connected peer
``spp send_file <bd> <ch> <path>``  Send a local file to a peer
``spp search <bd> <uuid_len> <uuid>``  Query a peer for a UUID service
``spp connect <bd> <uuid_len> <uuid>``  Discover and connect to a peer
``spp through_put <bd> <ch> <size>``  Send random data and measure speed
===================== ==================================================

Notes
*****

* ``uuid_len`` supports ``2``/``4``/``16`` (4 hex digits for 16-bit, 8 hex
  digits for 32-bit, or a standard 36-character UUID for 128-bit).
* File reception uses an idle timeout to detect the end of a transfer and
  saves the data to ``/lfs`` automatically.
* On boot, files left in ``/lfs`` from the previous run are cleared to avoid
  confusion.

See :zephyr:code-sample-category:`bluetooth_classic` samples for details.
