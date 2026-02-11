add_executable(MortonTests)

target_sources(MortonTests
	PRIVATE
		morton1_test.cpp
		morton2_test.cpp
		morton3_test.cpp
		morton4_test.cpp
)

target_link_libraries(MortonTests PRIVATE UFO::Morton Catch2::Catch2WithMain)

if (MSVC)
	target_compile_options(MortonTests PRIVATE /W4)
else()
	target_compile_options(MortonTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

catch_discover_tests(MortonTests)