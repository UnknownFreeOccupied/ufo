cmake_minimum_required(VERSION 3.5...3.16)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR AND NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()


project(ufotime
	VERSION 2.0.0
	DESCRIPTION "The UFO time library"
	LANGUAGES CXX
)

add_library(Time SHARED 
	src/timer.cpp
	src/timing.cpp
)
add_library(UFO::Time ALIAS Time)

set_target_properties(Time PROPERTIES
	VERSION ${PROJECT_VERSION}
	SOVERSION ${PROJECT_VERSION_MAJOR}
	CXX_STANDARD 17
	CXX_EXTENSIONS OFF
)

target_include_directories(Time PUBLIC
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

target_compile_features(Time PUBLIC cxx_std_17)

include(GNUInstallDirs)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Only enable tests when we're the root project
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
	# Treat warnings as errors
	set_target_properties(Time PROPERTIES COMPILE_WARNING_AS_ERROR ON)

	# Turn all warnings on
	if (MSVC)
		target_compile_options(Time PUBLIC /W4)
	else()
		target_compile_options(Time PUBLIC -Wall -Wextra -pedantic)
	endif()

	add_subdirectory(docs)
  add_subdirectory(tests)
endif()

install(TARGETS Time
	EXPORT ufotimeTargets
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(EXPORT ufotimeTargets
	FILE "ufotimeTargets.cmake"
	NAMESPACE UFO::
	DESTINATION lib/cmake/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
	"${PROJECT_SOURCE_DIR}/cmake/ufotimeConfig.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/ufotimeConfig.cmake"
	INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/ufotimeConfigVersion.cmake"
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion
)

INSTALL(
	FILES
	"${CMAKE_CURRENT_BINARY_DIR}/ufotimeConfig.cmake"
	"${CMAKE_CURRENT_BINARY_DIR}/ufotimeConfigVersion.cmake"
	DESTINATION lib/cmake/${PROJECT_NAME}
)

install(
	DIRECTORY ${PROJECT_SOURCE_DIR}/include/
	DESTINATION include
)