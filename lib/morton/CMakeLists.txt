cmake_minimum_required(VERSION 3.5...3.16)

project(ufomorton
	VERSION 2.0.0
	DESCRIPTION "The UFO Morton library"
	LANGUAGES CXX
)

add_subdirectory(1stparty)

add_library(ufomorton INTERFACE)
add_library(UFO::Morton ALIAS ufomorton)

set_target_properties(ufomorton PROPERTIES
	VERSION ${PROJECT_VERSION}
	SOVERSION ${PROJECT_VERSION_MAJOR}
	CXX_STANDARD 17
	CXX_EXTENSIONS OFF
)

target_link_libraries(ufomorton INTERFACE UFO::Math)

target_include_directories(ufomorton INTERFACE
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

target_compile_features(ufomorton INTERFACE cxx_std_17)

include(GNUInstallDirs)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Only enable tests when we're the root project
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
	# Treat warnings as errors
	# set_target_properties(ufomorton PROPERTIES COMPILE_WARNING_AS_ERROR ON)

	# # Turn all warnings on
	# if (MSVC)
	# 	target_compile_options(ufomorton INTERFACE /W4)
	# else()
	# 	target_compile_options(ufomorton INTERFACE -Wall -Wextra -pedantic)
	# endif()

  add_subdirectory(tests)
endif()

install(TARGETS ufomorton
	EXPORT ufomortonTargets
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(EXPORT ufomortonTargets
	FILE "ufomortonTargets.cmake"
	NAMESPACE UFO::
	DESTINATION lib/cmake/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
	"${PROJECT_SOURCE_DIR}/cmake/ufomortonConfig.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/ufomortonConfig.cmake"
	INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/ufomortonConfigVersion.cmake"
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion
)

INSTALL(
	FILES
	"${CMAKE_CURRENT_BINARY_DIR}/ufomortonConfig.cmake"
	"${CMAKE_CURRENT_BINARY_DIR}/ufomortonConfigVersion.cmake"
	DESTINATION lib/cmake/${PROJECT_NAME}
)

install(
	DIRECTORY ${PROJECT_SOURCE_DIR}/include/
	DESTINATION include
)