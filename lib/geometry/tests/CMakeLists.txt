add_executable(GeometryTests)

target_sources(GeometryTests
	PRIVATE
		aabb_test.cpp
		frustum_test.cpp
		line_test.cpp
)

set_target_properties(GeometryTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 17
		CXX_EXTENSIONS OFF
)

if (MSVC)
	target_compile_options(GeometryTests PRIVATE /W4)
else()
	target_compile_options(GeometryTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(GeometryTests PRIVATE UFO::Geometry Catch2::Catch2WithMain)

catch_discover_tests(GeometryTests)