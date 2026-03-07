add_executable(ComputeTests)

target_sources(ComputeTests
	PRIVATE
		compute_test.cpp
)

set_target_properties(ComputeTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 23
		CXX_EXTENSIONS OFF
		CXX_SCAN_FOR_MODULES OFF
)

target_link_libraries(ComputeTests PRIVATE Catch2::Catch2WithMain UFO::Compute UFO::Warnings)

catch_discover_tests(ComputeTests)