add_executable(GeometryTests)

target_sources(GeometryTests
	PRIVATE
		aabb_test.cpp
		frustum_test.cpp
		line_test.cpp
)

target_link_libraries(GeometryTests PRIVATE UFO::Geometry Catch2::Catch2WithMain)

if (MSVC)
	target_compile_options(GeometryTests PRIVATE /W4)
else()
	target_compile_options(GeometryTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

catch_discover_tests(GeometryTests)