add_executable(UtilityTests)

target_sources(UtilityTests
	PRIVATE
		iterator_wrapper_test.cpp
)

target_link_libraries(UtilityTests PRIVATE UFO::Utility Catch2::Catch2WithMain)

target_compile_options(UtilityTests
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

catch_discover_tests(UtilityTests)