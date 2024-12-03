include("${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/catch2/catch2.cmake")

# SET(GCC_COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage")
# SET(GCC_COVERAGE_LINK_FLAGS    "--coverage")
# SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
# SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )
# # set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)

add_executable(ufogeometry_tests
	aabb_test.cpp
	line_test.cpp
	frustum_test.cpp
)

target_link_libraries(ufogeometry_tests PRIVATE UFO::Geometry Catch2::Catch2WithMain)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(CTest)
include(Catch)
catch_discover_tests(ufogeometry_tests)

# add_custom_command(
# 	TARGET ufogeometry_tests
# 	COMMENT "Running UFO geometry tests"
# 	POST_BUILD 
# 	COMMAND ufogeometry_tests
# )