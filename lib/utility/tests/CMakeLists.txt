add_executable(UtilityTests)

target_sources(UtilityTests
	PRIVATE
		iterator_wrapper_test.cpp
)

target_link_libraries(UtilityTests PRIVATE UFO::Utility Catch2::Catch2WithMain)

if (MSVC)
	target_compile_options(UtilityTests PRIVATE /W4)
else()
	target_compile_options(UtilityTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

catch_discover_tests(UtilityTests)