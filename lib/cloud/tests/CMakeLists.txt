add_executable(CloudTests)

target_sources(CloudTests
	PRIVATE
		cloud_test.cpp
)

set_target_properties(CloudTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 17
		CXX_EXTENSIONS OFF
)

if (MSVC)
	target_compile_options(CloudTests PRIVATE /W4)
else()
	target_compile_options(CloudTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(CloudTests PRIVATE UFO::Cloud Catch2::Catch2WithMain)

catch_discover_tests(CloudTests)