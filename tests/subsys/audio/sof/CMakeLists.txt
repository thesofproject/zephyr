# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})
project(audio_sof)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

set(sof_module ${ZEPHYR_BASE}/../modules/audio/sof)

# Needed for using SOF headers
target_include_directories(app PRIVATE ${sof_module}/zephyr/include)
target_include_directories(app PRIVATE ${sof_module}/src/include)
target_include_directories(app PRIVATE ${sof_module}/src/arch/xtensa/include)
