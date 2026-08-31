# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024 STMicroelectronics

if(CONFIG_STM32N6_BOOT_SERIAL)
  board_runner_args(stm32cubeprogrammer "--port=usb1")
  board_runner_args(stm32cubeprogrammer "--download-modifiers=0x1")
  board_runner_args(stm32cubeprogrammer "--start-modifiers=noack")
else()
  board_runner_args(stm32cubeprogrammer "--port=swd")
  board_runner_args(stm32cubeprogrammer "--no-reset")
  board_runner_args(stm32cubeprogrammer "--tool-opt= mode=HOTPLUG ap=1")
  board_runner_args(stm32cubeprogrammer "--extload=Template_FSBL_XIP_ExtMemLoader.stldr")
  board_runner_args(stm32cubeprogrammer "--download-address=0x70000000")
endif()

board_runner_args(stlink_gdbserver "--apid=1")
board_runner_args(stlink_gdbserver "--extload=Template_FSBL_XIP_ExtMemLoader.stldr")

include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
include(${ZEPHYR_BASE}/boards/common/stlink_gdbserver.board.cmake)
