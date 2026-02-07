option(UFOMATH_BUILD_DOCS     "Generate documentation" OFF)
option(UFOMATH_BUILD_TESTS    "Unit testing"           OFF)
option(UFOMATH_BUILD_COVERAGE "Test Coverage"          OFF)

include(GNUInstallDirs)

add_library(Math INTERFACE)
add_library(UFO::Math ALIAS Math)

target_link_libraries(Math 
	INTERFACE
		UFO::Execution
		UFO::Utility
)

target_include_directories(Math 
	INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(UFO_BUILD_TESTS OR UFOMATH_BUILD_TESTS)
  add_subdirectory(tests)
endif()

if(UFO_BUILD_DOCS OR UFOMATH_BUILD_DOCS)
	add_subdirectory(docs)
endif()

install(TARGETS Math 
	EXPORT UFO-targets
	COMPONENT Math
	LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
	COMPONENT Math
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)