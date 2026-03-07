add_executable(NumericTests)

target_sources(NumericTests
	PRIVATE
		mat_test.cpp
		math_test.cpp
		transform_test.cpp
		quat_test.cpp
		vec_test.cpp
)

set_target_properties(NumericTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 23
		CXX_EXTENSIONS OFF
		CXX_SCAN_FOR_MODULES OFF
)

target_link_libraries(NumericTests PRIVATE Catch2::Catch2WithMain UFO::Numeric UFO::Warnings)

catch_discover_tests(NumericTests)