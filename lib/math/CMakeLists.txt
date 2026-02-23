add_library(Math INTERFACE)
add_library(UFO::Math ALIAS Math)

target_sources(Math
	INTERFACE
  	FILE_SET HEADERS
    BASE_DIRS
			include
    FILES
			include/ufo/math/mat.hpp
			include/ufo/math/math.hpp
			include/ufo/math/quat.hpp
			include/ufo/math/transform.hpp
			include/ufo/math/vec.hpp
)

set_target_properties(Math PROPERTIES CXX_SCAN_FOR_MODULES OFF)

if(UFO_BUILD_TESTS)
	add_subdirectory(tests)
endif()

install(
  TARGETS Math
	EXPORT UFOTargets
  FILE_SET HEADERS
	COMPONENT Math
)