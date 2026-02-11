add_executable(TimeTests)

target_sources(TimeTests
	PRIVATE
		timer_test.cpp
		timing_test.cpp
)

target_link_libraries(TimeTests PRIVATE UFO::Time Catch2::Catch2WithMain)

if (MSVC)
	target_compile_options(TimeTests PRIVATE /W4)
else()
	target_compile_options(TimeTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

catch_discover_tests(TimeTests)