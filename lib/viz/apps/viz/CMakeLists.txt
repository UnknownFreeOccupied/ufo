add_executable(ufoviz_exe
	src/ufoviz.cpp
)

set_target_properties(ufoviz_exe 
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 23
		CXX_EXTENSIONS OFF
		OUTPUT_NAME "ufoviz"
)

target_link_libraries(ufoviz_exe 
	PRIVATE
		UFO::Viz
)

include(GNUInstallDirs)

install(TARGETS ufoviz_exe EXPORT ufoviz_exe-targets
	LIBRARY  DESTINATION lib
	ARCHIVE  DESTINATION lib
	RUNTIME  DESTINATION bin
  INCLUDES DESTINATION include
)

install(EXPORT ufoviz_exe-targets
  FILE "ufoviz_exe-targets.cmake"
  NAMESPACE UFO::
  DESTINATION lib/cmake/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/ufoviz_exe-config-version.cmake"
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake/ufoviz_exe-config.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/ufoviz_exe-config.cmake"
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/ufoviz_exe-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/ufoviz_exe-config-version.cmake"
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)