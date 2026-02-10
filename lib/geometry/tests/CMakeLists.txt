add_executable(GeometryTests)

target_sources(GeometryTests
	PRIVATE
		aabb_test.cpp
		frustum_test.cpp
		line_test.cpp
)

target_link_libraries(GeometryTests PRIVATE UFO::Geometry Catch2::Catch2WithMain)

target_compile_options(GeometryTests
	PUBLIC
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

catch_discover_tests(GeometryTests)