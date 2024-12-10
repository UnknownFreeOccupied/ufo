option(UFO_UTILITY_DOCS     "Generate documentation" OFF)
option(UFO_UTILITY_TESTING  "Unit testing"           OFF)
option(UFO_UTILITY_COVERAGE "Test Coverage"          OFF)

add_library(Utility SHARED 
	src/io/read_buffer.cpp
	src/io/write_buffer.cpp
	src/io/buffer.cpp
)
add_library(UFO::Utility ALIAS Utility)

message(CHECK_START "Finding parallel processing libraries")
list(APPEND CMAKE_MESSAGE_INDENT "  ")
unset(missingParallelLibs)

message(CHECK_START "Finding Threading Building Blocks (TBB)")
# include("${PROJECT_SOURCE_DIR}/3rdparty/tbb/tbb.cmake")
find_package(TBB QUIET)

if(TBB_FOUND)
	message(CHECK_PASS "found and enabled")
	target_link_libraries(Utility PUBLIC TBB::tbb)
	target_compile_definitions(Utility
		PUBLIC
			UFO_TBB=1
	)
else()
	message(CHECK_FAIL "not found and disabled")
	list(APPEND missingParallelLibs TBB)
endif()

message(CHECK_START "Finding Grand Central Dispatch (GCD)")
if(APPLE)
	message(CHECK_PASS "found and enabled")
	target_compile_definitions(Utility
		PUBLIC
			UFO_GCD=1
	)
else()
	message(CHECK_FAIL "not found and disabled")
	list(APPEND missingParallelLibs GCD)
endif()

message(CHECK_START "Finding OpenMP (OMP)")
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
	message(CHECK_PASS "found and enabled")
	target_link_libraries(Utility PUBLIC OpenMP::OpenMP_CXX)
	target_compile_definitions(Utility
		PUBLIC
			UFO_OMP=1
	)
else()
	message(CHECK_FAIL "not found and disabled")
	list(APPEND missingParallelLibs OMP)
endif()


list(POP_BACK CMAKE_MESSAGE_INDENT)
if(missingParallelLibs)
  message(CHECK_FAIL "disabled parallel processing libraries: ${missingParallelLibs}")
else()
  message(CHECK_PASS "all parallel processing libraries found and enabled")
endif()

set_target_properties(Utility 
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 17
		CXX_EXTENSIONS OFF
		OUTPUT_NAME "UFOUtility"
)

include(GNUInstallDirs)

target_include_directories(Utility 
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)

install(TARGETS Utility EXPORT Utility-targets
	COMPONENT Utility
	LIBRARY  DESTINATION lib
	ARCHIVE  DESTINATION lib
	RUNTIME  DESTINATION bin
  INCLUDES DESTINATION include
)

install(EXPORT Utility-targets
  FILE "Utility-targets.cmake"
  NAMESPACE UFO::
  DESTINATION lib/cmake/${PROJECT_NAME}
	COMPONENT Utility
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/Utility-config-version.cmake"
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake/Utility-config.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/Utility-config.cmake"
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/Utility-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/Utility-config-version.cmake"
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  COMPONENT Utility
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
	COMPONENT Utility
	DESTINATION ${CMAKE_INSTALL_PREFIX}
)