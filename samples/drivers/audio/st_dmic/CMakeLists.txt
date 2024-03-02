# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(st_dmic)

target_sources(app PRIVATE src/main.c 
    src/stm32wb5mm_dk_conf.h
    src/stm32wb5mm_dk_errno.h
    src/stm32wb5mm_dk_audio.h
    src/stm32wb5mm_dk_audio.c)

zephyr_library_import(lib ${CMAKE_CURRENT_SOURCE_DIR}/src/libPDMFilter_CM4_GCC_wc16_softfp.a)
target_link_libraries(app PRIVATE lib)