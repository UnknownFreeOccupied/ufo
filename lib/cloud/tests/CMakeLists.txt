add_executable(CloudTests)

target_sources(CloudTests
	PRIVATE
		cloud_test.cpp
)

target_link_libraries(CloudTests PRIVATE UFO::Cloud Catch2::Catch2WithMain)

if (MSVC)
	target_compile_options(CloudTests PRIVATE /W4)
else()
	target_compile_options(CloudTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

catch_discover_tests(CloudTests)