# Intel CAVS SoC family CMake file
#
# Copyright (c) 2020 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

zephyr_interface_library_named(INTEL_ADSP_COMMON)

zephyr_library_named(intel_adsp_common)
zephyr_library_include_directories(include)

if(CONFIG_SOF)
	zephyr_library_include_directories(INTEL_ADSP_COMMON INTERFACE  ${PROJECT_SOURCE_DIR}/../modules/audio/sof/src/include)
	zephyr_library_include_directories(INTEL_ADSP_COMMON INTERFACE  ${PROJECT_SOURCE_DIR}/../modules/audio/sof/src/arch/xtensa/include)
endif()

set_source_files_properties(adsp.c PROPERTIES COMPILE_FLAGS -std=gnu99)
zephyr_library_sources(adsp.c)

set_source_files_properties(clk.c PROPERTIES COMPILE_FLAGS -std=gnu99)
zephyr_library_sources(clk.c)

set_source_files_properties(dai.c PROPERTIES COMPILE_FLAGS -std=gnu99)
zephyr_library_sources(dai.c)

set_source_files_properties(dai.c PROPERTIES COMPILE_FLAGS -std=gnu99)
zephyr_library_sources(dma.c)

zephyr_library_sources(main_entry.S)
zephyr_library_sources(mem_window.c)
zephyr_library_sources(soc.c)

set_source_files_properties(pm_memory.c PROPERTIES COMPILE_FLAGS -std=gnu99)
zephyr_library_sources(pm_memory.c)

set_source_files_properties(pm_runtime.c PROPERTIES COMPILE_FLAGS -std=gnu99)
zephyr_library_sources(pm_runtime.c)

# Common files for CAVS v1.8 onwards
if(CONFIG_SOC_SERIES_INTEL_CAVS_V18
   OR CONFIG_SOC_SERIES_INTEL_CAVS_V20
   OR CONFIG_SOC_SERIES_INTEL_CAVS_V25)
  set_source_files_properties(power_down.S PROPERTIES COMPILE_FLAGS -DASSEMBLY)
  zephyr_library_sources(power_down.S)

  set_source_files_properties(lps_pic_restore_vector.S PROPERTIES COMPILE_FLAGS -DASSEMBLY)
  zephyr_library_sources(lps_pic_restore_vector.S)

  set_source_files_properties(lps_wait.c PROPERTIES COMPILE_FLAGS -std=gnu99)
  zephyr_library_sources(lps_wait.c)
endif()

if(CONFIG_SMP)
  set_source_files_properties(arch/cpu.c PROPERTIES COMPILE_FLAGS -std=gnu99)
  zephyr_library_sources(arch/cpu.c)
endif()

zephyr_library_link_libraries(INTEL_ADSP_COMMON)

target_include_directories(INTEL_ADSP_COMMON INTERFACE include)
target_link_libraries(INTEL_ADSP_COMMON INTERFACE intel_adsp_common)

include(bootloader.cmake)
