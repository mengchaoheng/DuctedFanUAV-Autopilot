############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

add_library(DuctedFanPlugin SHARED
	DuctedFanPlugin.cpp
	Spline.cpp
)

target_link_libraries(DuctedFanPlugin
	PUBLIC ${GZ_PLUGIN_TARGET}
	PUBLIC ${GZ_SIM_TARGET}
	PUBLIC ${GZ_TRANSPORT_TARGET}
)

target_compile_features(DuctedFanPlugin PUBLIC cxx_std_17)

target_include_directories(DuctedFanPlugin
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
	PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
)

add_library(TorqueDisturbancePlugin SHARED
	TorqueDisturbancePlugin.cpp
)

target_link_libraries(TorqueDisturbancePlugin
	PUBLIC ${GZ_PLUGIN_TARGET}
	PUBLIC ${GZ_SIM_TARGET}
)

target_compile_features(TorqueDisturbancePlugin PUBLIC cxx_std_17)

target_include_directories(TorqueDisturbancePlugin
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
	PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
)

if (NOT CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	install(TARGETS DuctedFanPlugin TorqueDisturbancePlugin LIBRARY DESTINATION lib/px4_gz_plugins)
endif()
