cmake_minimum_required(VERSION 3.5...3.16)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR AND NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

project(ufocompute
	VERSION 2.0.0
	DESCRIPTION ""
	LANGUAGES CXX
)

# add_subdirectory(1stparty)

add_library(ufocompute INTERFACE)
add_library(UFO::Compute ALIAS ufocompute)
