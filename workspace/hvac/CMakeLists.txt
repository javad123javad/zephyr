# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hvac)

target_sources(app PRIVATE src/main.c src/adc_fake.c src/eeprom_fake.c src/fixedpoint-pid.c)
