# Catch2
if(UFO_BUILD_TESTS)
	if(USE_SYSTEM_CATCH2)
		find_package(Catch2 CONFIG)
		if(TARGET Catch2::Catch2WithMain)
			message(STATUS "Using installed third-party library Catch2 ${Catch2_VERSION}")
		else()
			message(STATUS "Unable to find installed third-party library Catch2")
			set(USE_SYSTEM_CATCH2 OFF)
		endif()
	endif()
	if(NOT USE_SYSTEM_CATCH2)
		message(STATUS "Fetching and building Catch2 from source")
		include("${CMAKE_CURRENT_LIST_DIR}/catch2/catch2.cmake")
	endif()
endif()

# CLI11
if(USE_SYSTEM_CLI11)
	find_package(CLI11 CONFIG)
	if(TARGET CLI11::CLI11)
		message(STATUS "Using installed third-party library CLI11 ${CLI11_VERSION}")
	else()
		message(STATUS "Unable to find installed third-party library CLI11")
		set(USE_SYSTEM_CLI11 OFF)
	endif()
endif()
if(NOT USE_SYSTEM_CLI11)
	message(STATUS "Fetching and building CLI11 from source")
	include("${CMAKE_CURRENT_LIST_DIR}/cli11/cli11.cmake")
endif()

# Doxygen
if(UFO_BUILD_DOCS)
	find_package(Doxygen REQUIRED dot)
endif()

# ImGUI

# # libjpeg-turbo
# message(STATUS "Fetching and building libjpeg-turbo from source")
# include("${CMAKE_CURRENT_LIST_DIR}/libjpeg_turbo/libjpeg_turbo.cmake")

# # libspng
# message(STATUS "Fetching and building libspng from source")
# include("${CMAKE_CURRENT_LIST_DIR}/libspng/libspng.cmake")

# # Rply
# message(STATUS "Fetching and building rply from source")
# # include("${CMAKE_CURRENT_LIST_DIR}/rply/rply.cmake")
# add_subdirectory("${CMAKE_CURRENT_LIST_DIR}/rply")

# WGPU Native
include("${CMAKE_CURRENT_LIST_DIR}/wgpu_native/wgpu_native.cmake")