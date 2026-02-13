add_executable(VisionTests)

target_sources(VisionTests
	PRIVATE
		color_test.cpp
)

set_target_properties(VisionTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 17
		CXX_EXTENSIONS OFF
)

if (MSVC)
	target_compile_options(VisionTests PRIVATE /W4)
else()
	target_compile_options(VisionTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(VisionTests PRIVATE UFO::Vision Catch2::Catch2WithMain)

catch_discover_tests(VisionTests)