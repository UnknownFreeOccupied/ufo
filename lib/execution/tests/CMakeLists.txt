add_executable(ExecutionTests)

target_sources(ExecutionTests
	PRIVATE
		execution_test.cpp
)
target_link_libraries(ExecutionTests PRIVATE UFO::Execution Catch2::Catch2WithMain)

if (MSVC)
	target_compile_options(ExecutionTests PRIVATE /W4)
else()
	target_compile_options(ExecutionTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

catch_discover_tests(ExecutionTests)