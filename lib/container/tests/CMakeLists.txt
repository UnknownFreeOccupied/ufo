add_executable(ContainerTests)

target_sources(ContainerTests
	PRIVATE
	# octree_test.cpp
	# octree_map_test.cpp
	# octree_set_test.cpp
	# quadtree_set_test.cpp
	# range_test.cpp
	# tree_index_test.cpp
	# tree_node_test.cpp
	predicate_test.cpp
)

target_link_libraries(ContainerTests PRIVATE UFO::Container Catch2::Catch2WithMain)

target_compile_options(ContainerTests
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

catch_discover_tests(ContainerTests)