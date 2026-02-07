option(UFOCOMPUTE_BUILD_DOCS "Generate documentation" OFF)
option(UFOCOMPUTE_BUILD_TESTS "Unit testing" OFF)
option(UFOCOMPUTE_BUILD_COVERAGE "Test Coverage" OFF)

include(GNUInstallDirs)

# TODO: Make option to make shared or static
add_library(Compute SHARED
	src/compute.cpp
)
add_library(UFO::Compute ALIAS Compute)

set_target_properties(Compute
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 17
		CXX_EXTENSIONS OFF
		OUTPUT_NAME "UFOCompute"
)

# set(WGPU "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/wgpu")
set(WGPU "${CMAKE_CURRENT_SOURCE_DIR}/../../external/wgpu")
set(WGPU_RUNTIME_LIB "${WGPU}/lib/libwgpu_native.so")

if(WIN32)
	set(OS_LIBRARIES d3dcompiler ws2_32 userenv bcrypt ntdll opengl32)
elseif(UNIX AND NOT APPLE)
	set(OS_LIBRARIES "-lm -ldl")
elseif(APPLE)
	set(OS_LIBRARIES "-framework CoreFoundation -framework QuartzCore -framework Metal")
endif()

# 1. Define the imported target for the raw .so file
add_library(wgpu_native SHARED IMPORTED GLOBAL)
set_target_properties(wgpu_native PROPERTIES
    IMPORTED_LOCATION "${WGPU_RUNTIME_LIB}"
	INTERFACE_INCLUDE_DIRECTORIES "${WGPU}/include"
)

target_link_libraries(Compute
	PUBLIC
		# $<BUILD_INTERFACE:${WGPU_RUNTIME_LIB}>
		# $<INSTALL_INTERFACE:${CMAKE_INSTALL_LIBDIR}/libwgpu_native.so>
		wgpu_native
		${OS_LIBRARIES}
)

target_include_directories(Compute
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<BUILD_INTERFACE:${WGPU}/include>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(UFO_BUILD_TESTS OR UFOCOMPUTE_BUILD_TESTS)
	add_subdirectory(tests)
endif()

if(UFO_BUILD_DOCS OR UFOCOMPUTE_BUILD_DOCS)
	add_subdirectory(docs)
endif()

# Configure the submodule-specific helper
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ComputeConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/Compute-deps.cmake"
    @ONLY
)

# Install it into the same directory where UConfig.cmake will live
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/Compute-deps.cmake" 
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/UFO)

install(TARGETS Compute 
	EXPORT UFO-targets
	COMPONENT Compute
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
	COMPONENT Compute
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${WGPU}/lib/libwgpu_native.so 
	COMPONENT Compute
	DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY ${WGPU}/include/
	COMPONENT Compute
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)