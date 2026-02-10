add_executable(VizTests)

target_sources(VizTests
	PRIVATE
		map_test.cpp
		# path_test.cpp
		# viz_test.cpp
)

target_link_libraries(VizTests PRIVATE UFO::Viz Catch2::Catch2WithMain)

target_compile_options(VizTests
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

catch_discover_tests(VizTests)