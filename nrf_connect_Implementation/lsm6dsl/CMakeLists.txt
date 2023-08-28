# SPDX-License-Identifier: Apache-2.0
# Include the Kconfig file
set(KCONFIG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Kconfig)
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lsm6dsl)

FILE(GLOB app_sources src/*.c lsm6dsl_custom/*.c remote_service/*.c features/*.c)

target_include_directories(app PRIVATE lsm6dsl_custom remote_service features)

target_sources(app PRIVATE ${app_sources})



