option(UFO_CORE_DOCS     "Generate documentation" OFF)
option(UFO_CORE_TESTING  "Unit testing"           OFF)
option(UFO_CORE_COVERAGE "Test Coverage"          OFF)

add_library(Core INTERFACE)
add_library(UFO::Core ALIAS Core)

target_link_libraries(Core 
	INTERFACE 
		UFO::Math
)

include(GNUInstallDirs)

target_include_directories(Core 
	INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)

if(UFO_TESTING OR UFO_CORE_TESTING)
  add_subdirectory(tests)
endif()

if(UFO_DOCS OR UFO_CORE_DOCS)
	add_subdirectory(docs)
endif()

install(TARGETS Core EXPORT Core-targets
	COMPONENT Core
	LIBRARY  DESTINATION lib
	ARCHIVE  DESTINATION lib
	RUNTIME  DESTINATION bin
  INCLUDES DESTINATION include
)

install(EXPORT Core-targets
  FILE "Core-targets.cmake"
  NAMESPACE UFO::
  DESTINATION lib/cmake/${PROJECT_NAME}
	COMPONENT Core
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/Core-config-version.cmake"
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake/Core-config.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/Core-config.cmake"
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/Core-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/Core-config-version.cmake"
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  COMPONENT Core
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
	COMPONENT Core
	DESTINATION ${CMAKE_INSTALL_PREFIX}
)