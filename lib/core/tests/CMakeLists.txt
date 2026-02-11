add_executable(CoreTests)

target_sources(CoreTests
	PRIVATE
		intensity_test.cpp
)

set_target_properties(CoreTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 17
		CXX_EXTENSIONS OFF
)

if (MSVC)
	target_compile_options(CoreTests PRIVATE /W4)
else()
	target_compile_options(CoreTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(CoreTests PRIVATE UFO::Core Catch2::Catch2WithMain)

catch_discover_tests(CoreTests)