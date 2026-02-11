add_executable(VisionTests)

target_sources(VisionTests
	PRIVATE
		color_test.cpp
)

target_link_libraries(VisionTests PRIVATE UFO::Vision Catch2::Catch2WithMain)

if (MSVC)
	target_compile_options(VisionTests PRIVATE /W4)
else()
	target_compile_options(VisionTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

catch_discover_tests(VisionTests)