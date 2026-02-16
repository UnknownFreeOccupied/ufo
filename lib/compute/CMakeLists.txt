option(UFO_COMPUTE_BUILD_DOCS "Generate documentation" ON)
option(UFO_COMPUTE_BUILD_TESTS "Unit testing" ON)
option(UFO_COMPUTE_BUILD_COVERAGE "Test Coverage" ON)

include(GNUInstallDirs)

add_library(Compute STATIC)
add_library(UFO::Compute ALIAS Compute)

target_sources(Compute
	PRIVATE
		src/compute.cpp

	PUBLIC
  	FILE_SET HEADERS
    BASE_DIRS
			include
    FILES
			include/ufo/compute/compute.hpp
)

set_target_properties(Compute
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 23
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
    wgpu_native
    ${OS_LIBRARIES}
)

target_include_directories(Compute
	PUBLIC
		$<BUILD_INTERFACE:${WGPU_DIR}/include>
)

if(UFO_BUILD_TESTS AND UFO_COMPUTE_BUILD_TESTS)
	add_subdirectory(tests)
endif()

if(UFO_BUILD_DOCS AND UFO_COMPUTE_BUILD_DOCS)
	add_subdirectory(docs)
endif()

install(
  TARGETS Compute
	EXPORT UFOTargets
  FILE_SET HEADERS
	COMPONENT Compute
)