option(UFO_MATH_BUILD_DOCS     "Generate documentation" ON)
option(UFO_MATH_BUILD_TESTS    "Unit testing"           ON)
option(UFO_MATH_BUILD_COVERAGE "Test Coverage"          ON)

include(GNUInstallDirs)

add_library(Math INTERFACE)
add_library(UFO::Math ALIAS Math)

target_sources(Math
	INTERFACE
  	FILE_SET HEADERS
    BASE_DIRS
			include
    FILES
			include/ufo/math/detail/mat_fun.hpp
			include/ufo/math/detail/mat.hpp
			include/ufo/math/detail/quat_fun.hpp
			include/ufo/math/detail/quat.hpp
			include/ufo/math/detail/transform_fun.hpp
			include/ufo/math/detail/transform.hpp
			include/ufo/math/detail/vec_fun.hpp
			include/ufo/math/detail/vec.hpp

			include/ufo/math/mat.hpp
			include/ufo/math/mat2x2.hpp
			include/ufo/math/mat3x3.hpp
			include/ufo/math/mat4x4.hpp
			include/ufo/math/math.hpp
			include/ufo/math/numbers.hpp
			include/ufo/math/quat.hpp
			include/ufo/math/transform.hpp
			include/ufo/math/transform2.hpp
			include/ufo/math/transform3.hpp
			include/ufo/math/vec.hpp
			include/ufo/math/vec1.hpp
			include/ufo/math/vec2.hpp
			include/ufo/math/vec3.hpp
			include/ufo/math/vec4.hpp
)

target_link_libraries(Math 
	INTERFACE
		UFO::Execution
		UFO::Utility
)

if(UFO_BUILD_TESTS AND UFO_MATH_BUILD_TESTS)
  add_subdirectory(tests)
endif()

if(UFO_BUILD_DOCS AND UFO_MATH_BUILD_DOCS)
	add_subdirectory(docs)
endif()

install(
  TARGETS Math
	EXPORT UFOTargets
  FILE_SET HEADERS
	COMPONENT Math
)