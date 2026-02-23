cmake_minimum_required(VERSION 3.23...3.31)

# set(CMAKE_EXPERIMENTAL_EXPORT_PACKAGE_DEPENDENCIES "1942b4fa-b2c5-4546-9385-83f254070067")

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(UFO
	VERSION 1.0.0
	DESCRIPTION "UFO"
	LANGUAGES CXX C
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)


option(UFO_BUILD_DOCS                 "Generate documentation"             OFF)
option(UFO_BUILD_TESTS                "Build all unit tests"               OFF)
option(UFO_BUILD_EXAMPLES             "Build example applications"         OFF)
option(UFO_COVERAGE                   "Test Coverage"                      OFF)
option(UFO_DEV_MODE                   "Set up development helper settings" ON)
# option(UFO_ENABLE_NATIVE_OPTIMIZATION "Optimize for the host CPU"          ON)
option(UFO_ENABLE_LTO                 "Enable Link Time Optimization"      ON)

option(USE_SYSTEM_CATCH2 "Use system-installed Catch2 instead of fetching/building it" OFF)
option(USE_SYSTEM_CLI11 "Use system-installed CLI11 instead of fetching/building it" OFF)
option(USE_SYSTEM_IMGUI "Use system-installed ImGUI instead of fetching/building it" OFF)
option(USE_SYSTEM_JPEG "Use system-installed libjpeg-turbo instead of fetching/building it" OFF)
option(USE_SYSTEM_SPNG "Use system-installed libspng instead of fetching/building it" OFF)

include(external/find_dependencies.cmake)

if(UFO_BUILD_TESTS)
    # NOTE: Important that this is first 
    enable_testing()
    include(CTest)
    include(Catch)
    add_subdirectory(tests)
endif()

# add_subdirectory(lib/cloud)
# add_subdirectory(lib/compute)
# add_subdirectory(lib/container)
add_subdirectory(lib/core)
# add_subdirectory(lib/execution)
# add_subdirectory(lib/geometry)
# add_subdirectory(lib/io)
# add_subdirectory(lib/map)
add_subdirectory(lib/math)
# add_subdirectory(lib/morton)
# add_subdirectory(lib/plan)
# add_subdirectory(lib/sensor)
# add_subdirectory(lib/time)
add_subdirectory(lib/utility)
# add_subdirectory(lib/vision)
# # add_subdirectory(lib/viz)


add_library(UFO INTERFACE)
add_library(UFO::UFO ALIAS UFO)

target_link_libraries(UFO INTERFACE 
    # UFO::Cloud
    # UFO::Compute
    # UFO::Container
    UFO::Core
    # UFO::Execution
    # UFO::Geometry
    # UFO::IO
    # UFO::Map
    UFO::Math
    # UFO::Morton
    # UFO::Plan
    # UFO::Sensor
    # UFO::Time
    UFO::Utility
    # UFO::Vision
    # # UFO::Viz
)

add_subdirectory(apps)

if(UFO_BUILD_DOCS)
    find_package(Doxygen REQUIRED OPTIONAL_COMPONENTS dot)

    if(DOXYGEN_DOT_FOUND)
        set(DOXYGEN_HAVE_DOT YES)
    else()
        set(DOXYGEN_HAVE_DOT NO)
    endif()

    set(DOXYGEN_OUTPUT_DIR "${PROJECT_BINARY_DIR}/docs")

    configure_file(
        "${PROJECT_SOURCE_DIR}/Doxyfile.in"
        "${PROJECT_BINARY_DIR}/Doxyfile"
        @ONLY
    )

    add_custom_target(Docs
        COMMAND "${DOXYGEN_EXECUTABLE}" "${PROJECT_BINARY_DIR}/Doxyfile"
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
    )
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
install(
    EXPORT UFOTargets
    FILE ufo-targets.cmake
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/UFO"
    NAMESPACE UFO::
    # EXPORT_PACKAGE_DEPENDENCIES
)

# Install the config files
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/ufo-config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/ufo-config-version.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/UFO"
)