option(UFOCOMPUTE_BUILD_DOCS     "Generate documentation" OFF)
option(UFOCOMPUTE_BUILD_TESTS    "Unit testing"           OFF)
option(UFOCOMPUTE_BUILD_COVERAGE "Test Coverage"          OFF)

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

include("3rdparty/webgpu/webgpu.cmake")

if (WIN32)
	set(OS_LIBRARIES d3dcompiler ws2_32 userenv bcrypt ntdll opengl32)
elseif(UNIX AND NOT APPLE)
	set(OS_LIBRARIES "-lm -ldl")
elseif(APPLE)
	set(OS_LIBRARIES "-framework CoreFoundation -framework QuartzCore -framework Metal")
endif()

target_link_libraries(Compute
	PUBLIC
		webgpu
		${OS_LIBRARIES}
)

include(GNUInstallDirs)

target_include_directories(Compute 
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)

if(UFO_BUILD_TESTS OR UFOCOMPUTE_BUILD_TESTS)
  add_subdirectory(tests)
endif()

if(UFO_BUILD_DOCS OR UFOCOMPUTE_BUILD_DOCS)
	add_subdirectory(docs)
endif()

install(TARGETS Compute EXPORT Compute-targets
	COMPONENT Compute
	LIBRARY  DESTINATION lib
	ARCHIVE  DESTINATION lib
	RUNTIME  DESTINATION bin
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
	DESTINATION ${CMAKE_INSTALL_PREFIX}
)