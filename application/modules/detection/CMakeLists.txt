#
# Copyright (c) 2026 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

target_sources(app PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/detection.c
	${CMAKE_CURRENT_LIST_DIR}/nrf_edgeai_generated/nrf_edgeai_user_model.c
)

target_include_directories(app PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
	${CMAKE_CURRENT_LIST_DIR}/../../../external/edge-ai/include
)

# Link EdgeAI library based on CPU architecture
if(CONFIG_NRF_EDGEAI)
	set(EDGEAI_PATH ${CMAKE_CURRENT_LIST_DIR}/../../../external/edge-ai)
	if(CONFIG_CPU_CORTEX_M33)
		target_link_libraries(app PRIVATE ${EDGEAI_PATH}/lib/nrf_edgeai/cortex-m33/libnrf_edgeai_cortex-m33.a)
	elseif(CONFIG_CPU_CORTEX_M4)
		target_link_libraries(app PRIVATE ${EDGEAI_PATH}/lib/nrf_edgeai/cortex-m4/libnrf_edgeai_cortex-m4.a)
	endif()
endif()
