add_executable(TimeTests)

target_sources(TimeTests
	PRIVATE
		timer_test.cpp
		timing_test.cpp
)

set_target_properties(TimeTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 17
		CXX_EXTENSIONS OFF
)

if (MSVC)
	target_compile_options(TimeTests PRIVATE /W4)
else()
	target_compile_options(TimeTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(TimeTests PRIVATE UFO::Time Catch2::Catch2WithMain)

catch_discover_tests(TimeTests)