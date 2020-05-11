# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_link_libraries(intel_adsp_common)
zephyr_library_include_directories(${ZEPHYR_BASE}/drivers)

set_source_files_properties(clk.c PROPERTIES COMPILE_FLAGS -std=gnu99)
zephyr_library_sources(clk.c)
