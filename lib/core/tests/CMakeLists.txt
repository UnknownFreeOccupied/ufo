add_executable(CoreTests)

target_sources(CoreTests
	PRIVATE
		intensity_test.cpp
)

target_link_libraries(CoreTests PRIVATE UFO::Core Catch2::Catch2WithMain)

target_compile_options(CoreTests
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

catch_discover_tests(CoreTests)