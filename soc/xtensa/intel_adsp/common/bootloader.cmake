# Copyright (c) 2019 Intel Corporation
#
# SPDX-License-Identifier: Apache-2.0

if(EXISTS ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/bootloader/CMakeLists.txt)
  if(USING_OUT_OF_TREE_BOARD)
    set(build_dir boards/${ARCH}/${BOARD}/bootloader)
  else()
    unset(build_dir)
  endif()

  add_subdirectory(${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/bootloader ${build_dir})
endif()

set(ELF_FIX ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/fix_elf_addrs.py)

add_custom_target(
  process_elf ALL
  DEPENDS base_module
  DEPENDS ${ZEPHYR_FINAL_EXECUTABLE}
  COMMAND ${CMAKE_OBJCOPY} --dump-section .data=mod-apl.bin ${CMAKE_BINARY_DIR}/zephyr/soc/xtensa/${SOC_FAMILY}/common/bootloader/libbase_module.a
  COMMAND ${CMAKE_OBJCOPY} --add-section .module=mod-apl.bin --set-section-flags .module=load,readonly ${CMAKE_BINARY_DIR}/zephyr/zephyr.elf ${CMAKE_BINARY_DIR}/zephyr/zephyr.elf.mod

  # Adjust final section addresses so they all appear in the cached region.
  COMMAND ${ELF_FIX} ${CMAKE_OBJCOPY} ${CMAKE_BINARY_DIR}/zephyr/zephyr.elf.mod
  )

add_custom_target(
  process_bootloader ALL
  DEPENDS bootloader boot_module
  COMMAND ${CMAKE_OBJCOPY} --dump-section .data=mod-boot.bin ${CMAKE_BINARY_DIR}/zephyr/soc/xtensa/${SOC_FAMILY}/common/bootloader/libboot_module.a
  COMMAND ${CMAKE_OBJCOPY} --add-section .module=mod-boot.bin --set-section-flags .module=load,readonly ${CMAKE_BINARY_DIR}/zephyr/soc/xtensa/${SOC_FAMILY}/common/bootloader/bootloader.elf ${CMAKE_BINARY_DIR}/zephyr/bootloader.elf.mod
  )
