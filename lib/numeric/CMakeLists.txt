add_library(Numeric INTERFACE)
add_library(UFO::Numeric ALIAS Numeric)

target_sources(Numeric
	INTERFACE
  	FILE_SET HEADERS
    BASE_DIRS
			include
    FILES
			include/ufo/numeric/mat.hpp
			include/ufo/numeric/math.hpp
			include/ufo/numeric/quat.hpp
			include/ufo/numeric/transform.hpp
			include/ufo/numeric/vec.hpp
)

set_target_properties(Numeric PROPERTIES CXX_SCAN_FOR_MODULES OFF)

if(UFO_BUILD_TESTS)
	add_subdirectory(tests)
endif()

install(
  TARGETS Numeric
	EXPORT UFOTargets
  FILE_SET HEADERS
	COMPONENT Numeric
)