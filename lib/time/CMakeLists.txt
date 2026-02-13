option(UFO_TIME_BUILD_DOCS     "Generate documentation" ON)
option(UFO_TIME_BUILD_TESTS    "Unit testing"           ON)
option(UFO_TIME_BUILD_COVERAGE "Test Coverage"          ON)

include(GNUInstallDirs)

add_library(Time STATIC 
	src/timer.cpp
	src/timing.cpp
)
add_library(UFO::Time ALIAS Time)

set_target_properties(Time 
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 17
		CXX_EXTENSIONS OFF
		OUTPUT_NAME "UFOTime"
		POSITION_INDEPENDENT_CODE ON
)

if (MSVC)
	target_compile_options(Time PRIVATE /W4)
else()
	target_compile_options(Time PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_include_directories(Time 
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(UFO_BUILD_TESTS AND UFO_TIME_BUILD_TESTS)
  add_subdirectory(tests)
endif()

if(UFO_BUILD_DOCS AND UFO_TIME_BUILD_DOCS)
	add_subdirectory(docs)
endif()

install(TARGETS Time 
	EXPORT UFO-targets
	COMPONENT Time
	LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
	COMPONENT Time
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)