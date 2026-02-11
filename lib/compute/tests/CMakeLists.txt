add_executable(ComputeTests)

target_sources(ComputeTests
	PRIVATE
		compute_test.cpp
)


target_link_libraries(ComputeTests PRIVATE UFO::Compute Catch2::Catch2WithMain)

if (MSVC)
	target_compile_options(ComputeTests PRIVATE /W4)
else()
	target_compile_options(ComputeTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

catch_discover_tests(ComputeTests)