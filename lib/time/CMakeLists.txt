option(UFO_TIME_BUILD_TESTS    "Unit testing"           ON)
option(UFO_TIME_BUILD_COVERAGE "Test Coverage"          ON)

include(GNUInstallDirs)

add_library(Time STATIC)
add_library(UFO::Time ALIAS Time)

target_sources(Time
	PRIVATE
		src/timer.cpp
		src/timing.cpp

	PUBLIC
  	FILE_SET HEADERS
    BASE_DIRS
			include
    FILES
			include/ufo/time/timer.hpp
			include/ufo/time/timing.hpp
)

set_target_properties(Time 
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 23
		CXX_EXTENSIONS OFF
		OUTPUT_NAME "UFOTime"
		POSITION_INDEPENDENT_CODE ON
)

if (MSVC)
	target_compile_options(Time PRIVATE /W4)
else()
	target_compile_options(Time PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(UFO_BUILD_TESTS AND UFO_TIME_BUILD_TESTS)
  add_subdirectory(tests)
endif()


install(
  TARGETS Time
	EXPORT UFOTargets
  FILE_SET HEADERS
	COMPONENT Time
)