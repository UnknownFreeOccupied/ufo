option(UFOCONTAINER_BUILD_DOCS     "Generate documentation" OFF)
option(UFOCONTAINER_BUILD_TESTS    "Unit testing"           OFF)
option(UFOCONTAINER_BUILD_COVERAGE "Test Coverage"          OFF)

include(GNUInstallDirs)

add_library(Container INTERFACE)
add_library(UFO::Container ALIAS Container)

target_link_libraries(Container 
	INTERFACE
		UFO::Compute
		UFO::Execution
		UFO::Geometry 
		UFO::Morton 
		UFO::Utility
		UFO::Vision
)

target_include_directories(Container 
	INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(UFO_BUILD_TESTS OR UFOCONTAINER_BUILD_TESTS)
  add_subdirectory(tests)
endif()

if(UFO_BUILD_DOCS OR UFOCONTAINER_BUILD_DOCS)
	add_subdirectory(docs)
endif()

install(TARGETS Container 
	EXPORT UFO-targets
	COMPONENT Container
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
	COMPONENT Container
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)