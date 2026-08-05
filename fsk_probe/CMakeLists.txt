#
# Copyright (c) 2026
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

get_filename_component(SIDEWALK_DEVKIT_BOARD_MODULE
    "${CMAKE_CURRENT_LIST_DIR}/../../../Sidewalk_Devkit_Board_Files"
    ABSOLUTE
)
if(EXISTS "${SIDEWALK_DEVKIT_BOARD_MODULE}/zephyr/module.yml")
    list(APPEND ZEPHYR_EXTRA_MODULES "${SIDEWALK_DEVKIT_BOARD_MODULE}")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(fsk_probe)

target_sources(app PRIVATE src/main.c)
