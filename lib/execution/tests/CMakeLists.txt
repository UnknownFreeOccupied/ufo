add_executable(ExecutionTests)

target_sources(ExecutionTests
	PRIVATE
		execution_test.cpp
)

set_target_properties(ExecutionTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 17
		CXX_EXTENSIONS OFF
)

if (MSVC)
	target_compile_options(ExecutionTests PRIVATE /W4)
else()
	target_compile_options(ExecutionTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(ExecutionTests PRIVATE UFO::Execution Catch2::Catch2WithMain)

catch_discover_tests(ExecutionTests)