# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/rf/rf.h)

zephyr_library()

zephyr_sources_ifdef(CONFIG_RF_SHELL shell.c)
zephyr_library_sources_ifdef(CONFIG_NORDIC_NRF24L1 nrf24l1.c)

