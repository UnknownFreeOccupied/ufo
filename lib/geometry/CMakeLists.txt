option(UFOGEOMETRY_BUILD_DOCS     "Generate documentation" OFF)
option(UFOGEOMETRY_BUILD_TESTS    "Unit testing"           OFF)
option(UFOGEOMETRY_BUILD_COVERAGE "Test Coverage"          OFF)

include(GNUInstallDirs)

add_library(Geometry INTERFACE)
add_library(UFO::Geometry ALIAS Geometry)

target_link_libraries(Geometry 
	INTERFACE 
		UFO::Math
)

target_include_directories(Geometry 
	INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(UFO_BUILD_TESTS OR UFOGEOMETRY_BUILD_TESTS)
  add_subdirectory(tests)
endif()

if(UFO_BUILD_DOCS OR UFOGEOMETRY_BUILD_DOCS)
	add_subdirectory(docs)
endif()

install(TARGETS Geometry 
	EXPORT UFO-targets
	COMPONENT Geometry
	LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
	COMPONENT Geometry
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)