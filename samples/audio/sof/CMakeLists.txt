# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

set(sof_module $ENV{ZEPHYR_BASE}/../modules/audio/sof)

# This needs to be before find_package, see
# https://github.com/zephyrproject-rtos/zephyr/issues/24512
set(sof_defconfigs ${sof_module}/src/arch/xtensa/configs)
if (${BOARD} STREQUAL up_squared_adsp)
  set(OVERLAY_CONFIG ${sof_defconfigs}/apollolake_defconfig)
endif()

find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})
project(sample_sof)

target_sources(app PRIVATE
    src/main.c
  )

zephyr_interface_library_named(sof_lib)

zephyr_library_include_directories(app PUBLIC
    ${sof_module}/src/arch/xtensa/include
    ${sof_module}/src/include
  )
