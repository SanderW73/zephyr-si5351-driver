cmake_minimum_required(VERSION 3.20.0)

if(NOT cdc-acm-console IN_LIST SNIPPET)
  set(SNIPPET cdc-acm-console ${SNIPPET} CACHE STRING "" FORCE)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)
target_sources(app PRIVATE src/main.c)
