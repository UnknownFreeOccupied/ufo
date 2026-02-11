add_executable(CoreTests)

target_sources(CoreTests
	PRIVATE
		intensity_test.cpp
)

target_link_libraries(CoreTests PRIVATE UFO::Core Catch2::Catch2WithMain)

if (MSVC)
	target_compile_options(CoreTests PRIVATE /W4)
else()
	target_compile_options(CoreTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

catch_discover_tests(CoreTests)