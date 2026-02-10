add_executable(VisionTests)

target_sources(VisionTests
	PRIVATE
		color_test.cpp
)

target_link_libraries(VisionTests PRIVATE UFO::Vision Catch2::Catch2WithMain)

target_compile_options(VisionTests
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

catch_discover_tests(VisionTests)