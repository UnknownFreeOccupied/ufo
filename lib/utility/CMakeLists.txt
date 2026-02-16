option(UFO_UTILITY_BUILD_DOCS     "Generate documentation" ON)
option(UFO_UTILITY_BUILD_TESTS    "Unit testing"           ON)
option(UFO_UTILITY_BUILD_COVERAGE "Test Coverage"          ON)

include(GNUInstallDirs)

add_library(Utility STATIC)
add_library(UFO::Utility ALIAS Utility)

target_sources(Utility
	PRIVATE
		src/io/buffer.cpp
		src/io/read_buffer.cpp
		src/io/write_buffer.cpp

	PUBLIC
  	FILE_SET HEADERS
    BASE_DIRS
			include
    FILES
			include/ufo/utility/io/buffer.hpp
			include/ufo/utility/io/read_buffer.hpp
			include/ufo/utility/io/write_buffer.hpp

			include/ufo/utility/bit_set.hpp
			include/ufo/utility/create_array.hpp
			include/ufo/utility/enum.hpp
			include/ufo/utility/index_iterator.hpp
			include/ufo/utility/iterator_wrapper.hpp
			include/ufo/utility/macros.hpp
			include/ufo/utility/proxy_arrow_result.hpp
			include/ufo/utility/spinlock.hpp
			include/ufo/utility/string.hpp
			include/ufo/utility/tuple_iterator.hpp
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

if(UFO_BUILD_DOCS AND UFO_UTILITY_BUILD_DOCS)
	add_subdirectory(docs)
endif()

install(
  TARGETS Utility
	EXPORT UFOTargets
  FILE_SET HEADERS
	COMPONENT Utility
)