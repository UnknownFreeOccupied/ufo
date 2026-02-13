option(UFO_PLAN_BUILD_DOCS     "Generate documentation" ON)
option(UFO_PLAN_BUILD_TESTS    "Unit testing"           ON)
option(UFO_PLAN_BUILD_COVERAGE "Test Coverage"          ON)

include(GNUInstallDirs)

add_library(Plan INTERFACE)
add_library(UFO::Plan ALIAS Plan)

target_link_libraries(Plan 
	INTERFACE
		UFO::Container
		UFO::Map
		UFO::Math
)

target_include_directories(Plan 
	INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(UFO_BUILD_TESTS AND UFO_PLAN_BUILD_TESTS)
  add_subdirectory(tests)
endif()

if(UFO_BUILD_DOCS AND UFO_PLAN_BUILD_DOCS)
	add_subdirectory(docs)
endif()

install(TARGETS Plan 
	EXPORT UFO-targets
	COMPONENT Plan
	LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
	COMPONENT Plan
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)