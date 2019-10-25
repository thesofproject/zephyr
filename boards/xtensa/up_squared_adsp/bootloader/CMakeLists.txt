# Copyright (c) 2019 Intel Corporation
#
# SPDX-License-Identifier: Apache-2.0

add_library(base_module base_module.c)
target_include_directories(base_module PUBLIC
  ${SOC_DIR}/${ARCH}/${SOC_PATH}/include
  )

add_library(boot_module boot_module.c)
target_include_directories(boot_module PUBLIC
  ${SOC_DIR}/${ARCH}/${SOC_PATH}/include
  )

add_executable(bootloader
  _vectors.S
  boot_entry.S
  memctl_default.S
  reset-vector.S
  boot_loader.c
  )

set(zephyr_sdk $ENV{ZEPHYR_SDK_INSTALL_DIR})

target_include_directories(bootloader PUBLIC
  ./
  ${ZEPHYR_BASE}/include
  # TODO: SDK: This should come automatically
  ${zephyr_sdk}/xtensa/intel_apl_adsp/xtensa-zephyr-elf/include/
  ${zephyr_sdk}/xtensa/intel_apl_adsp/xtensa-zephyr-elf/include/arch/include/
  ${SOC_DIR}/${ARCH}/${SOC_PATH}/include
  )

set_source_files_properties(_vectors.S PROPERTIES COMPILE_FLAGS -DASSEMBLY)
set_source_files_properties(boot_entry.S PROPERTIES COMPILE_FLAGS -DASSEMBLY)
set_source_files_properties(reset-vector.S PROPERTIES COMPILE_FLAGS -DASSEMBLY)

target_compile_options(bootloader PUBLIC -fno-inline-functions -mlongcalls -mtext-section-literals -DCONFIG_BOOT_LOADER -DCONFIG_APOLLOLAKE -DCONFIG_BUILD_VM_ROM -imacros${CMAKE_BINARY_DIR}/zephyr/include/generated/autoconf.h)

target_link_libraries(bootloader PUBLIC -Wl,--no-check-sections -ucall_user_start -Wl,-static -nostdlib)
target_link_libraries(bootloader PRIVATE -lhal -L${zephyr_sdk}/xtensa/intel_apl_adsp/xtensa-zephyr-elf/lib)
target_link_libraries(bootloader PRIVATE -T${BOARD_DIR}/bootloader/boot_ldr.x)
