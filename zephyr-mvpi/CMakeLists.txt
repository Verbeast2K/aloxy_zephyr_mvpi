# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0
#
# This CMake file is picked by the Zephyr build system because it is defined
# as the module CMake entry point (see zephyr/module.yml).

cmake_minimum_required(VERSION 3.21)
project(mvpi-pulse)

zephyr_include_directories(include)

add_subdirectory(drivers)
add_subdirectory(lib)
