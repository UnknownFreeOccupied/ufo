add_executable(UtilityTests)

target_sources(UtilityTests
	PRIVATE
		iterator_wrapper_test.cpp
)

set_target_properties(UtilityTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 17
		CXX_EXTENSIONS OFF
)

if (MSVC)
	target_compile_options(UtilityTests PRIVATE /W4)
else()
	target_compile_options(UtilityTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(UtilityTests PRIVATE UFO::Utility Catch2::Catch2WithMain)

catch_discover_tests(UtilityTests)