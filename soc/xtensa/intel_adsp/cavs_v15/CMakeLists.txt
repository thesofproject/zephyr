# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_include_directories(${ZEPHYR_BASE}/drivers)
zephyr_library_include_directories(${ZEPHYR_BASE}/soc/xtensa/intel_adsp/common/include)
zephyr_library_sources(adsp.c)
zephyr_library_sources(soc.c)
zephyr_library_sources(main_entry.S)

zephyr_library_sources_ifdef(CONFIG_SMP soc_mp.c)

set_source_files_properties(power_down.S PROPERTIES COMPILE_FLAGS -DASSEMBLY)
zephyr_library_sources(power_down.S)
