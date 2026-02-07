cmake_minimum_required(VERSION 3.16...3.24)

project(UFO
	VERSION 1.0.0
	DESCRIPTION "UFO"
	LANGUAGES CXX C
)

option(UFO_BUILD_DOCS       "Generate documentation"             OFF)
option(UFO_BUILD_TESTS      "Build all unit tests"               OFF)
option(UFO_BUILD_EXAMPLES   "Build example applications"         OFF)
option(UFO_COVERAGE         "Test Coverage"                      OFF)
option(UFO_DEV_MODE         "Set up development helper settings" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(external/dependencies.cmake)


add_subdirectory(lib/cloud)
add_subdirectory(lib/compute)
add_subdirectory(lib/container)
add_subdirectory(lib/core)
add_subdirectory(lib/execution)
add_subdirectory(lib/geometry)
add_subdirectory(lib/map)
add_subdirectory(lib/math)
add_subdirectory(lib/morton)
add_subdirectory(lib/plan)
add_subdirectory(lib/time)
add_subdirectory(lib/utility)
add_subdirectory(lib/vision)
# add_subdirectory(lib/viz)


add_library(ufo INTERFACE)
add_library(UFO::UFO ALIAS ufo)

target_link_libraries(ufo INTERFACE 
    UFO::Cloud
    UFO::Compute
    UFO::Container
    UFO::Core
    UFO::Execution
    UFO::Geometry
    UFO::Map
    UFO::Math
    UFO::Morton
    UFO::Plan
    UFO::Time
    UFO::Utility
    UFO::Vision
    # UFO::Viz
)

if(UFO_BUILD_TESTS)
    enable_testing()
endif()

if(UFO_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Installation & Export Logic
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# Generate the version file
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/ufo-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

# Generate the master Config file
configure_package_config_file(
    "${CMAKE_SOURCE_DIR}/cmake/ufo-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/ufo-config.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/UFO"
)

# Export all targets under the UFO:: namespace
install(EXPORT UFO-targets
    FILE ufo-targets.cmake
    NAMESPACE UFO::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/UFO"
)

# Install the config files
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/ufo-config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/ufo-config-version.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/UFO"
)