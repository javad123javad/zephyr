# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/scilator.h)

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_SI5351 si5351.c)

#zephyr_library_sources_ifdef(CONFIG_LED_SHELL   led_shell.c)

#zephyr_library_sources_ifdef(CONFIG_USERSPACE   led_handlers.c)
