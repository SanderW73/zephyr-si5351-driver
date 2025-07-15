cmake_minimum_required(VERSION 3.20.0)

# list(APPEND EXTRA_ZEPHYR_MODULES
#     ${CMAKE_CURRENT_SOURCE_DIR}/..
# )

if(NOT cdc-acm-console IN_LIST SNIPPET)
  set(SNIPPET cdc-acm-console ${SNIPPET} CACHE STRING "" FORCE)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)
target_sources(app PRIVATE src/main.c)
