add_executable(MapTests)

target_sources(MapTests
	PRIVATE
		# map_test.cpp
		# integration_test.cpp
		occupancy_map_test.cpp
		# labels_map_test.cpp
		label_map_test.cpp
)

target_link_libraries(MapTests PRIVATE UFO::Map Catch2::Catch2WithMain)

target_compile_options(MapTests
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

catch_discover_tests(MapTests)