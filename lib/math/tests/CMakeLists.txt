add_executable(MathTests)

target_sources(MathTests
	PRIVATE
		mat2x2_test.cpp
		mat3x3_test.cpp
		mat4x4_test.cpp
		transform2_test.cpp
		transform3_test.cpp
		quat_test.cpp
		vec1_test.cpp
		vec2_test.cpp
		vec3_test.cpp
		vec4_test.cpp
)

set_target_properties(MathTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 17
		CXX_EXTENSIONS OFF
)

if (MSVC)
	target_compile_options(MathTests PRIVATE /W4)
else()
	target_compile_options(MathTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(MathTests PRIVATE UFO::Math Catch2::Catch2WithMain)

catch_discover_tests(MathTests)