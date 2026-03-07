add_executable(ExecutionTests)

target_sources(ExecutionTests
	PRIVATE
		algorithm_test.cpp
		execution_test.cpp
)

set_target_properties(ExecutionTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 23
		CXX_EXTENSIONS OFF
		CXX_SCAN_FOR_MODULES OFF
)

target_link_libraries(ExecutionTests PRIVATE Catch2::Catch2WithMain UFO::Execution UFO::Warnings)

catch_discover_tests(ExecutionTests)