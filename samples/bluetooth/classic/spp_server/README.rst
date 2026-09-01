.. zephyr:code-sample:: bluetooth_spp_server
   :name: SPP Server
   :relevant-api: bt_rfcomm bt_sdp bluetooth

   Expose a Bluetooth Classic Serial Port service over RFCOMM.

Overview
********

This sample registers an RFCOMM server and advertises two SPP services in SDP:

* The standard Serial Port service (UUID ``0x1101``) for phones and other
  Classic Bluetooth clients.
* A board-to-board SPP service with a custom UUID (``0x3001``) that another
  development board can connect to, enabling multiple SPP connections between
  boards.

It also provides a shell interface for data/file transfer, board-to-board
search/connect, and throughput testing. Received file/bulk data is stored on a
littlefs partition mounted at ``/lfs``.

Requirements
************

* A board with Bluetooth BR/EDR (Classic) support
* An on-board or external NOR flash providing a ``/lfs`` storage partition for
  file transfer

Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/bluetooth/classic/spp_server
   :board: sf32lb52_devkit_lcd/sf32lb525uc6
   :goals: build flash
   :compact:

After flashing, the device becomes discoverable as ``spp_server``. The serial
log prints the local BD address and the two SPP channels (phone channel from
the ``0x1101`` service, and board channel from the ``0x3001`` service).

Phone Data and File Transfer (SPP 0x1101)
*****************************************

1. Discover and connect to ``spp_server`` from a phone Bluetooth debug app.
2. When the phone sends normal data, the board prints the received content and
   replies with ``Hello from SPP server``.
3. When the phone pushes file/bulk data, the board saves it to a file under
   ``/lfs`` and prints the total bytes, duration and average speed once the
   transfer completes.
4. Shell commands::

      uart:~$ spp ls
      uart:~$ spp send_data 94:7B:AE:7B:BF:18 6
      uart:~$ spp send_file 94:7B:AE:7B:BF:18 6 /lfs/spp_rx_xxxxx.bin

Board-to-Board Connection Test (SPP 0x3001)
*******************************************

Many phones only support the standard ``0x1101`` SPP, and a board-initiated
connection to a phone is often dropped quickly. For board-to-board use, a
custom UUID (``0x3001``) is used instead: flash this sample on two boards (or
flash ``spp_server`` on one and ``spp_client`` on the other) and let one side
actively connect to the other's ``0x3001`` channel.

1. Power on both boards and note the service board's BD address and the board
   SPP channel printed at boot::

      My BD address: AB:89:67:45:23:01
      Board SPP (uuid 0x3001) registered on RFCOMM channel 7

2. On the client board, connect to the service board::

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

The board channel (``0x3001``) is independent from the phone channel
(``0x1101``), so a board can talk to a phone and another board at the same
time. The initiating side needs ``CONFIG_BT_CENTRAL=y``.

Shell Commands
**************

===================== ==================================================
Command               Description
===================== ==================================================
``spp addr``          Print the local BR/EDR address and board channel
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
