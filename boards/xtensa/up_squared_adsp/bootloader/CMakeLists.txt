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
