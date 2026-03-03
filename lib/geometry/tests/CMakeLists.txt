add_executable(GeometryTests)

target_sources(GeometryTests
	PRIVATE
		aabb_test.cpp
		frustum_test.cpp
		min_max_test.cpp
		# line_test.cpp
)

set_target_properties(GeometryTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 23
		CXX_EXTENSIONS OFF
		CXX_SCAN_FOR_MODULES OFF
)

target_link_libraries(GeometryTests PRIVATE Catch2::Catch2WithMain UFO::Geometry UFO::Warnings)

catch_discover_tests(GeometryTests)