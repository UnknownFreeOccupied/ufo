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

target_link_libraries(MathTests PRIVATE UFO::Math Catch2::Catch2WithMain)

target_compile_options(MathTests
	PRIVATE
		# -Wall
		# -Werror
		# -Wextra
		# -pedantic
		# -Wconversion
		# -Wcast-align
		# -Wunused
		# # -Wshadow
		# -Wold-style-cast
		# -Wpointer-arith
		# -Wcast-qual
		# -Wno-missing-braces
)

catch_discover_tests(MathTests)