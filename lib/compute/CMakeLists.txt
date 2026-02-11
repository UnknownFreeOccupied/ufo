option(UFO_COMPUTE_BUILD_DOCS "Generate documentation" ON)
option(UFO_COMPUTE_BUILD_TESTS "Unit testing" ON)
option(UFO_COMPUTE_BUILD_COVERAGE "Test Coverage" ON)

include(GNUInstallDirs)

set(SRC_FILES
	src/compute.cpp
)

set(INC_FILES
	include/ufo/compute/compute.hpp
)

add_library(Compute STATIC ${SRC_FILES} ${INC_FILES})
add_library(UFO::Compute ALIAS Compute)

set_target_properties(Compute
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 17
		CXX_EXTENSIONS OFF
		OUTPUT_NAME "UFOCompute"
		POSITION_INDEPENDENT_CODE ON
)

if (MSVC)
	target_compile_options(Compute PRIVATE /W4)
else()
	target_compile_options(Compute PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(Compute 
	PUBLIC
		$<BUILD_INTERFACE:$<IF:$<CONFIG:Debug>,${WGPU_BUILD_STATIC_RUNTIME_LIB_DEBUG},${WGPU_BUILD_STATIC_RUNTIME_LIB_RELEASE}>>
		$<INSTALL_INTERFACE:$<IF:$<CONFIG:Debug>,${WGPU_INSTALL_STATIC_RUNTIME_LIB_DEBUG},${WGPU_INSTALL_STATIC_RUNTIME_LIB_RELEASE}>>
)

target_include_directories(Compute
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<BUILD_INTERFACE:$<IF:$<CONFIG:Debug>,${WGPU_DIR_DEBUG},${WGPU_DIR_RELEASE}>/include>
		$<INSTALL_INTERFACE:include>
)

if(UFO_BUILD_TESTS AND UFO_COMPUTE_BUILD_TESTS)
	add_subdirectory(tests)
endif()

if(UFO_BUILD_DOCS AND UFO_COMPUTE_BUILD_DOCS)
	add_subdirectory(docs)
endif()

install(TARGETS Compute EXPORT Compute-targets
	COMPONENT Compute
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

install(EXPORT Compute-targets
	FILE "Compute-targets.cmake"
	NAMESPACE UFO::
	DESTINATION lib/cmake/${PROJECT_NAME}
	COMPONENT Compute
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/Compute-config-version.cmake"
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake/Compute-config.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/Compute-config.cmake"
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

install(
	FILES
	"${CMAKE_CURRENT_BINARY_DIR}/Compute-config.cmake"
	"${CMAKE_CURRENT_BINARY_DIR}/Compute-config-version.cmake"
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
	COMPONENT Compute
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
	COMPONENT Compute
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)