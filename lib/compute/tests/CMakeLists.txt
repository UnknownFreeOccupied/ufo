add_executable(ComputeTests)

target_sources(ComputeTests
	PRIVATE
		compute_test.cpp
)


target_link_libraries(ComputeTests PRIVATE UFO::Compute Catch2::Catch2WithMain)

target_compile_options(ComputeTests
	PUBLIC
		-Wall
		-Werror
		-Wextra
		-pedantic
		-Wconversion
		-Wcast-align
		-Wunused
		# -Wshadow
		-Wold-style-cast
		-Wpointer-arith
		-Wcast-qual
		-Wno-missing-braces
)

catch_discover_tests(ComputeTests)