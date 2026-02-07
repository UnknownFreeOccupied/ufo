option(UFOVISION_BUILD_DOCS     "Generate documentation" OFF)
option(UFOVISION_BUILD_TESTS    "Unit testing"           OFF)
option(UFOVISION_BUILD_COVERAGE "Test Coverage"          OFF)

add_library(Vision INTERFACE)
add_library(UFO::Vision ALIAS Vision)

target_link_libraries(Vision 
	INTERFACE 
		UFO::Geometry
		UFO::Math
		UFO::Utility
)

target_include_directories(Vision 
	INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(UFO_BUILD_TESTS OR UFOVISION_BUILD_TESTS)
  add_subdirectory(tests)
endif()

if(UFO_BUILD_DOCS OR UFOVISION_BUILD_DOCS)
	add_subdirectory(docs)
endif()

install(TARGETS Vision 
EXPORT UFO-targets
	COMPONENT Vision
	LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
	COMPONENT Vision
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)