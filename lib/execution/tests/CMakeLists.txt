add_executable(ExecutionTests)

target_sources(ExecutionTests
	PRIVATE
		execution_test.cpp
)
target_link_libraries(ExecutionTests PRIVATE UFO::Execution Catch2::Catch2WithMain)

target_compile_options(ExecutionTests
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

catch_discover_tests(ExecutionTests)