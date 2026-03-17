#!/bin/bash

esptool.py --chip esp32c6 -p /dev/ttyACM0 -b 460800 --before=default_reset --after=hard_reset write_flash --flash_mode dio --flash_freq 80m --flash_size 8MB 0x0 build/bootloader/bootloader.bin 0x10000 build/Test.bin 0x8000 build/partition_table/partition-table.bin 0x3C0000 build/esp-idf/main/lp_core_main/lp_core_main.bin
