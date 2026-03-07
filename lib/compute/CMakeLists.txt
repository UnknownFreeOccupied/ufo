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
		CXX_SCAN_FOR_MODULES OFF
		OUTPUT_NAME "UFOCompute"
		POSITION_INDEPENDENT_CODE ON
)

target_link_libraries(Compute PRIVATE $<BUILD_INTERFACE:UFO::Warnings>)

target_link_libraries(Compute 
  PUBLIC
    wgpu_native
    ${OS_LIBRARIES}
)

target_include_directories(Compute
	PUBLIC
		$<BUILD_INTERFACE:${WGPU_DIR}/include>
)

if(UFO_BUILD_TESTS)
	add_subdirectory(tests)
endif()

install(
  TARGETS Compute
	EXPORT UFOTargets
  FILE_SET HEADERS
	COMPONENT Compute
)