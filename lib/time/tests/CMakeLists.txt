add_executable(TimeTests)

target_sources(TimeTests
	PRIVATE
		timer_test.cpp
		timing_test.cpp
)

target_link_libraries(TimeTests PRIVATE UFO::Time Catch2::Catch2WithMain)

target_compile_options(TimeTests
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

catch_discover_tests(TimeTests)