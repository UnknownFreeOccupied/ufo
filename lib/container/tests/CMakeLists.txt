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

set_target_properties(ContainerTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 17
		CXX_EXTENSIONS OFF
)

if (MSVC)
	target_compile_options(ContainerTests PRIVATE /W4)
else()
	target_compile_options(ContainerTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(ContainerTests PRIVATE UFO::Container Catch2::Catch2WithMain)

catch_discover_tests(ContainerTests)