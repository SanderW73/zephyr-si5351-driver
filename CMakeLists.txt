# SPDX-License-Identifier: Apache-2.0

if (CONFIG_SI5351)
zephyr_library()
zephyr_syscall_header(${ZEPHYR_CURRENT_MODULE_DIR}/include/si5351.h)
zephyr_include_directories(include)
zephyr_library_sources(si5351.c)
endif (CONFIG_SI5351)
