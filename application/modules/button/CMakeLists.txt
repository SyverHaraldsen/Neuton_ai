#
# Copyright (c) 2026 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

# Button module sources
target_sources(app PRIVATE ${CMAKE_CURRENT_LIST_DIR}/button.c)

# Button module include directories
target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR})
