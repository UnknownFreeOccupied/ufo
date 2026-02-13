Include(FetchContent)

FetchContent_Declare(
	imgui
	GIT_REPOSITORY https://github.com/ocornut/imgui.git
	GIT_TAG 6d910d5487d11ca567b61c7824b0c78c569d62f0 # v1.92.5
	GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(imgui)
set(IMGUI_DIR "${imgui_SOURCE_DIR}")

set(SOURCES
	# Among the different backends available, we are interested in connecting
	# the GUI to GLFW and WebGPU:
	"${IMGUI_DIR}/backends/imgui_impl_wgpu.cpp"
	"${IMGUI_DIR}/backends/imgui_impl_glfw.cpp"

	# Bonus to add some C++ specific features (the core ImGUI is a C library)
	"${IMGUI_DIR}/misc/cpp/imgui_stdlib.cpp"

	# The core ImGui files
	"${IMGUI_DIR}/imgui.cpp"
	"${IMGUI_DIR}/imgui_draw.cpp"
	"${IMGUI_DIR}/imgui_tables.cpp"
	"${IMGUI_DIR}/imgui_widgets.cpp"
)

add_library(imgui STATIC "${SOURCES}")
add_library(ImGUI::imgui ALIAS imgui)

target_compile_definitions(imgui PUBLIC IMGUI_IMPL_WEBGPU_BACKEND_WGPU)

# C++11 is required by ImGui
set_target_properties(imgui
	PROPERTIES
		CXX_STANDARD 11
		CXX_STANDARD_REQUIRED ON
		# FIXME: Why is this needed here?
		POSITION_INDEPENDENT_CODE ON
)

target_link_libraries(imgui
	PUBLIC
		UFO::glfw_webgpu
)

target_include_directories(imgui
	PUBLIC
		$<BUILD_INTERFACE:${IMGUI_DIR}>
		$<INSTALL_INTERFACE:include>
)

include(GNUInstallDirs)

install(TARGETS imgui EXPORT imgui-targets
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

install(EXPORT imgui-targets
	FILE "imgui-targets.cmake"
	NAMESPACE ImGUI::
	DESTINATION lib/cmake/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/imgui-config-version.cmake"
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake/imgui-config.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/imgui-config.cmake"
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

install(
	FILES
	"${CMAKE_CURRENT_BINARY_DIR}/imgui-config.cmake"
	"${CMAKE_CURRENT_BINARY_DIR}/imgui-config-version.cmake"
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

install(
	FILES
		"${IMGUI_DIR}/imconfig.h"
		"${IMGUI_DIR}/imgui.h"
		"${IMGUI_DIR}/imgui_internal.h"
		"${IMGUI_DIR}/imstb_rectpack.h"
		"${IMGUI_DIR}/imstb_textedit.h"
		"${IMGUI_DIR}/imstb_truetype.h"
	DESTINATION include/imgui
)

install(
	FILES
		"${IMGUI_DIR}/backends/imgui_impl_glfw.h"
		"${IMGUI_DIR}/backends/imgui_impl_wgpu.h"
	DESTINATION include/imgui/backends
)

install(
	FILES
		"${IMGUI_DIR}/misc/cpp/imgui_stdlib.h"
	DESTINATION include/imgui/misc/cpp
)