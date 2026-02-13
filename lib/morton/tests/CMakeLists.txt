add_executable(MortonTests)

target_sources(MortonTests
	PRIVATE
		morton1_test.cpp
		morton2_test.cpp
		morton3_test.cpp
		morton4_test.cpp
)

set_target_properties(MortonTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 17
		CXX_EXTENSIONS OFF
)

if (MSVC)
	target_compile_options(MortonTests PRIVATE /W4)
else()
	target_compile_options(MortonTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(MortonTests PRIVATE UFO::Morton Catch2::Catch2WithMain)

catch_discover_tests(MortonTests)