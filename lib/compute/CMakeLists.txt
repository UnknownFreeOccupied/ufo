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
    wgpu_native
    ${OS_LIBRARIES}
)

target_include_directories(Compute
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<BUILD_INTERFACE:${WGPU_DIR}/include>
		$<INSTALL_INTERFACE:include>
)

if(UFO_BUILD_TESTS AND UFO_COMPUTE_BUILD_TESTS)
	add_subdirectory(tests)
endif()

if(UFO_BUILD_DOCS AND UFO_COMPUTE_BUILD_DOCS)
	add_subdirectory(docs)
endif()

# We add this target to the shared 'UFO-targets' export set
install(TARGETS Compute 
	EXPORT UFO-targets
	COMPONENT Compute
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
	COMPONENT Compute
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)