option(UFO_UTILITY_BUILD_TESTS    "Unit testing"           ON)
option(UFO_UTILITY_BUILD_COVERAGE "Test Coverage"          ON)

include(GNUInstallDirs)

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
		OUTPUT_NAME "UFOUtility"
		POSITION_INDEPENDENT_CODE ON
)

if (MSVC)
	target_compile_options(Utility PRIVATE /W4)
else()
	target_compile_options(Utility PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(UFO_BUILD_TESTS AND UFO_UTILITY_BUILD_TESTS)
  add_subdirectory(tests)
endif()


install(
  TARGETS Utility
	EXPORT UFOTargets
  FILE_SET HEADERS
	COMPONENT Utility
)