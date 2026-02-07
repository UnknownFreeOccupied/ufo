option(UFOCLOUD_BUILD_DOCS "Generate documentation" OFF)
option(UFOCLOUD_BUILD_TESTS "Unit testing" OFF)
option(UFOCLOUD_BUILD_COVERAGE "Test Coverage" OFF)

include(GNUInstallDirs)

add_library(Cloud INTERFACE)
add_library(UFO::Cloud ALIAS Cloud)

target_link_libraries(Cloud 
	INTERFACE
		UFO::Execution
		UFO::Utility
)

if(UFO_BUILD_TESTS OR UFOCLOUD_BUILD_TESTS)
    add_subdirectory(tests)
endif()

target_include_directories(Cloud 
	INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

install(TARGETS Cloud 
    EXPORT UFO-targets
    COMPONENT Cloud
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    COMPONENT Cloud
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)