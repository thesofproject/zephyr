# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})
project(sample_sof)

target_sources(app PRIVATE
    src/main.c
  )

zephyr_interface_library_named(sof_lib)
zephyr_library_sources(
    src/dai.c
    src/clk.c
    src/dma.c
    src/pm_memory.c
    src/pm_runtime.c
  )

set_source_files_properties(src/clk.c PROPERTIES COMPILE_FLAGS -std=gnu99)
set_source_files_properties(src/dai.c PROPERTIES COMPILE_FLAGS -std=gnu99)
set_source_files_properties(src/dma.c PROPERTIES COMPILE_FLAGS -std=gnu99)
set_source_files_properties(src/pm_memory.c PROPERTIES COMPILE_FLAGS -std=gnu99)
set_source_files_properties(src/pm_runtime.c PROPERTIES COMPILE_FLAGS -std=gnu99)

set(sof_module ${ZEPHYR_BASE}/../modules/audio/sof)
zephyr_library_include_directories(app PUBLIC
    ${sof_module}/src/arch/xtensa/include
    ${sof_module}/src/include
  )
