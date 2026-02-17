option(UFO_VIZ_BUILD_TESTS "Unit testing" ON)
option(UFO_VIZ_BUILD_COVERAGE "Test Coverage" ON)

include(GNUInstallDirs)

add_subdirectory(apps)

add_subdirectory(glfw_webgpu)

add_library(Viz STATIC)
add_library(UFO::Viz ALIAS Viz)

target_sources(Viz
	PRIVATE
		src/viz.cpp

	PUBLIC
  	FILE_SET HEADERS
    BASE_DIRS
			include
    FILES
			include/ufo/viz/renderable/map.hpp
			include/ufo/viz/renderable/path.hpp
			include/ufo/viz/renderable/plan.hpp
			include/ufo/viz/renderable/renderable_trianglelist.hpp
			include/ufo/viz/renderable/triangulate.hpp

			include/ufo/viz/renderable.hpp
			include/ufo/viz/viz.hpp
)

set_target_properties(Viz
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 23
		CXX_EXTENSIONS OFF
		OUTPUT_NAME "UFOViz"
)

if (MSVC)
	target_compile_options(Viz PRIVATE /W4)
else()
	target_compile_options(Viz PRIVATE -Wall -Wextra -Wpedantic)
endif()

# add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/imgui")
target_link_libraries(Viz
	PUBLIC
		UFO::Compute
		UFO::Map
		UFO::Math
		UFO::Plan
		UFO::Vision

	# imgui
	PRIVATE
		glfw_webgpu
)

if(UFO_DEV_MODE)
	# In dev mode, we load resources from the source tree
	target_compile_definitions(Viz PUBLIC
		UFOVIZ_SHADER_DIR="${CMAKE_CURRENT_SOURCE_DIR}/shaders"
	)
else()
	# In release mode, we just load resources relatively to wherever the
	# executable is launched from, so that the binary is portable
	target_compile_definitions(Viz PUBLIC
		UFOVIZ_SHADER_DIR="./shaders"
	)
endif()

if(UFO_BUILD_TESTS AND UFO_VIZ_BUILD_TESTS)
	add_subdirectory(tests)
endif()


install(
  TARGETS Viz glfw_webgpu
	EXPORT UFOTargets
  FILE_SET HEADERS
	COMPONENT Viz
)