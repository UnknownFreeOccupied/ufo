add_executable(ComputeTests)

target_sources(ComputeTests
	PRIVATE
		compute_test.cpp
)

set_target_properties(ComputeTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 17
		CXX_EXTENSIONS OFF
)

if (MSVC)
	target_compile_options(ComputeTests PRIVATE /W4)
else()
	target_compile_options(ComputeTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(ComputeTests PRIVATE UFO::Compute Catch2::Catch2WithMain)

catch_discover_tests(ComputeTests)