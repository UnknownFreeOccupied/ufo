add_executable(MathTests)

target_sources(MathTests
	PRIVATE
		mat_test.cpp
		math_test.cpp
		transform_test.cpp
		quat_test.cpp
		vec_test.cpp
)

set_target_properties(MathTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 23
		CXX_EXTENSIONS OFF
)

if (MSVC)
	target_compile_options(MathTests PRIVATE /W4)
else()
	target_compile_options(MathTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(MathTests PRIVATE Catch2::Catch2WithMain UFO::Math)

catch_discover_tests(MathTests)