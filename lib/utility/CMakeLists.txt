option(UFO_UTILITY_BUILD_DOCS     "Generate documentation" ON)
option(UFO_UTILITY_BUILD_TESTS    "Unit testing"           ON)
option(UFO_UTILITY_BUILD_COVERAGE "Test Coverage"          ON)

# --- ufo/utility/CMakeLists.txt ---
include(GNUInstallDirs)

# 1. Define the library
# Note: Since it has .cpp files, it's SHARED/STATIC (usually STATIC in monorepos)
add_library(Utility STATIC 
    src/io/read_buffer.cpp
    src/io/write_buffer.cpp
    src/io/buffer.cpp
)
add_library(UFO::Utility ALIAS Utility)

set_target_properties(Utility
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 17
		CXX_EXTENSIONS OFF
		OUTPUT_NAME "UFOUtility"
		POSITION_INDEPENDENT_CODE ON
)

if (MSVC)
	target_compile_options(Utility PRIVATE /W4)
else()
	target_compile_options(Utility PRIVATE -Wall -Wextra -Wpedantic)
endif()

# 4. Include Directories
# Use the "Hall of Mirrors" fix: point directly to your include folder
target_include_directories(Utility 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(UFO_BUILD_TESTS AND UFO_UTILITY_BUILD_TESTS)
  add_subdirectory(tests)
endif()

if(UFO_BUILD_DOCS AND UFO_UTILITY_BUILD_DOCS)
	add_subdirectory(docs)
endif()

# 5. Global Install (Simplified)
# We add this target to the shared 'UFO-targets' export set
install(TARGETS Utility 
	EXPORT UFO-targets
    COMPONENT Utility
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    COMPONENT Utility
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)