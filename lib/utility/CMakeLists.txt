add_library(Utility STATIC)
add_library(UFO::Utility ALIAS Utility)

target_sources(Utility
	PRIVATE
		src/spinlock.cpp
		src/string.cpp
	PUBLIC
  	FILE_SET HEADERS
    BASE_DIRS
			include
    FILES
			include/ufo/utility/bit_set.hpp
			include/ufo/utility/create_array.hpp
			include/ufo/utility/macros.hpp
			include/ufo/utility/proxy_arrow_result.hpp
			include/ufo/utility/scalar_type.hpp
			include/ufo/utility/spinlock.hpp
			include/ufo/utility/string.hpp
			include/ufo/utility/type_traits.hpp
)

set_target_properties(Utility
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 23
		CXX_EXTENSIONS OFF
		CXX_SCAN_FOR_MODULES OFF
		OUTPUT_NAME "UFOUtility"
		POSITION_INDEPENDENT_CODE ON
)

target_link_libraries(Utility PRIVATE $<BUILD_INTERFACE:UFO::Warnings>)

if(UFO_BUILD_TESTS)
  add_subdirectory(tests)
endif()

install(
  TARGETS Utility
	EXPORT UFOTargets
  FILE_SET HEADERS
	COMPONENT Utility
)