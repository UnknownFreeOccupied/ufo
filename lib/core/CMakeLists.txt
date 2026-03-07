add_library(Core INTERFACE)
add_library(UFO::Core ALIAS Core)

target_sources(Core
	INTERFACE
  	FILE_SET HEADERS
    BASE_DIRS
			include
    FILES
			include/ufo/core/confidence.hpp
			include/ufo/core/core.hpp
			include/ufo/core/intensity.hpp
			include/ufo/core/label.hpp
			include/ufo/core/normal.hpp
			include/ufo/core/range.hpp
			include/ufo/core/semantic.hpp
			include/ufo/core/surfel.hpp
			include/ufo/core/weight.hpp
)

set_target_properties(Core PROPERTIES CXX_SCAN_FOR_MODULES OFF)

target_link_libraries(Core INTERFACE UFO::Numeric)

if(UFO_BUILD_TESTS)
  add_subdirectory(tests)
endif()

install(
  TARGETS Core
	EXPORT UFOTargets
  FILE_SET HEADERS
	COMPONENT Core
)