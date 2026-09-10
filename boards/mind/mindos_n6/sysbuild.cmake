# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2025 STMicroelectronics

if(SB_CONFIG_BOOTLOADER_MCUBOOT)
  set_target_properties(mcuboot PROPERTIES BOARD mindos_n6/stm32n657xx/fsbl)
  set_config_int(mcuboot CONFIG_LOG_DEFAULT_LEVEL 4)
  set_config_bool(mcuboot CONFIG_LOG_MODE_IMMEDIATE n)
endif()
