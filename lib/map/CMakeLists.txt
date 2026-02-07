option(UFOMAP_BUILD_DOCS "Generate documentation" OFF)
option(UFOMAP_BUILD_TESTS "Unit testing" OFF)
option(UFOMAP_BUILD_COVERAGE "Test Coverage" OFF)

include(GNUInstallDirs)

add_library(Map INTERFACE)
add_library(UFO::Map ALIAS Map)

target_link_libraries(Map
	INTERFACE
		UFO::Cloud
		UFO::Compute
		UFO::Container
		UFO::Core
		UFO::Execution
		UFO::Geometry
		UFO::Math
		UFO::Morton
		UFO::Time
		UFO::Utility
		UFO::Vision
)

target_include_directories(Map 
	INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(UFO_BUILD_TESTS OR UFOMAP_BUILD_TESTS)
    add_subdirectory(tests)
endif()

install(TARGETS Map 
    EXPORT UFO-targets
    COMPONENT Map
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
	COMPONENT Map
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)