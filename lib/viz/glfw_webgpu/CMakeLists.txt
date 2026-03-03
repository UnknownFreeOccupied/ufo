include(3rdparty/glfw/glfw.cmake)

add_library(glfw_webgpu STATIC src/glfw_surface.c)
add_library(UFO::glfw_webgpu ALIAS glfw_webgpu)

set_target_properties(glfw_webgpu
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		C_STANDARD 17
		C_EXTENSIONS OFF
		OUTPUT_NAME "UFOGLFW_WebGPU"
		# FIXME: Why is this needed here?
		POSITION_INDEPENDENT_CODE ON
)

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
	target_compile_definitions(glfw_webgpu PRIVATE GLFW_EXPOSE_NATIVE_WIN32)
	set(OS_LIBRARIES d3dcompiler ws2_32 userenv bcrypt ntdll opengl32 Propsys RuntimeObject)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
	target_compile_definitions(glfw_webgpu PRIVATE GLFW_EXPOSE_NATIVE_X11 GLFW_EXPOSE_NATIVE_WAYLAND)
	set(OS_LIBRARIES "-lm -ldl")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
	target_compile_definitions(glfw_webgpu PRIVATE GLFW_EXPOSE_NATIVE_COCOA)
	target_compile_options(glfw_webgpu PRIVATE -x objective-c)
	set(OS_LIBRARIES "-framework CoreFoundation -framework QuartzCore -framework Metal")
else()
	message(FATAL_ERROR "Platform system '${CMAKE_SYSTEM_NAME}' not supported by this release of UFOViz.")
endif()

target_link_libraries(glfw_webgpu
	PUBLIC
		UFO::Compute
		glfw
	PRIVATE
		${OS_LIBRARIES}
)

target_include_directories(glfw_webgpu
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)

include(GNUInstallDirs)

install(TARGETS glfw_webgpu EXPORT glfw_webgpu-targets
	COMPONENT Compute
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

install(EXPORT glfw_webgpu-targets
	FILE "glfw_webgpu-targets.cmake"
	NAMESPACE UFO::
	DESTINATION lib/cmake/${PROJECT_NAME}
	COMPONENT Compute
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/glfw_webgpu-config-version.cmake"
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake/glfw_webgpu-config.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/glfw_webgpu-config.cmake"
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

install(
	FILES
	"${CMAKE_CURRENT_BINARY_DIR}/glfw_webgpu-config.cmake"
	"${CMAKE_CURRENT_BINARY_DIR}/glfw_webgpu-config-version.cmake"
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
	COMPONENT Compute
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
	COMPONENT Compute
	DESTINATION ${CMAKE_INSTALL_PREFIX}
)