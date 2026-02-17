option(UFO_CONTAINER_BUILD_TESTS    "Unit testing"           ON)
option(UFO_CONTAINER_BUILD_COVERAGE "Test Coverage"          ON)

include(GNUInstallDirs)

add_library(Container INTERFACE)
add_library(UFO::Container ALIAS Container)

target_sources(Container
	INTERFACE
  	FILE_SET HEADERS
    BASE_DIRS
			include
    FILES
			include/ufo/container/structure_of_arrays/element.hpp
			include/ufo/container/structure_of_arrays/value.hpp
			include/ufo/container/structure_of_arrays/view.hpp

			include/ufo/container/tree/map/block.hpp
			include/ufo/container/tree/map/iterator.hpp
			include/ufo/container/tree/map/nearest_iterator.hpp
			include/ufo/container/tree/map/query_iterator.hpp
			include/ufo/container/tree/map/query_nearest_iterator.hpp

			include/ufo/container/tree/predicate/and.hpp
			include/ufo/container/tree/predicate/bool.hpp
			include/ufo/container/tree/predicate/child_of.hpp
			include/ufo/container/tree/predicate/coord.hpp
			include/ufo/container/tree/predicate/depth_interval.hpp
			include/ufo/container/tree/predicate/depth.hpp
			include/ufo/container/tree/predicate/exists.hpp
			include/ufo/container/tree/predicate/false.hpp
			include/ufo/container/tree/predicate/filter.hpp
			include/ufo/container/tree/predicate/if_and_only_if.hpp
			include/ufo/container/tree/predicate/inner.hpp
			include/ufo/container/tree/predicate/leaf_or_depth.hpp
			include/ufo/container/tree/predicate/leaf.hpp
			include/ufo/container/tree/predicate/length_interval.hpp
			include/ufo/container/tree/predicate/length.hpp
			include/ufo/container/tree/predicate/modified.hpp
			include/ufo/container/tree/predicate/offset.hpp
			include/ufo/container/tree/predicate/or.hpp
			include/ufo/container/tree/predicate/parent.hpp
			include/ufo/container/tree/predicate/predicate_compare.hpp
			include/ufo/container/tree/predicate/predicate.hpp
			include/ufo/container/tree/predicate/pure_leaf.hpp
			include/ufo/container/tree/predicate/satisfies_inner.hpp
			include/ufo/container/tree/predicate/satisfies.hpp
			include/ufo/container/tree/predicate/spatial.hpp
			include/ufo/container/tree/predicate/then.hpp
			include/ufo/container/tree/predicate/true.hpp

			include/ufo/container/tree/set/block.hpp
			include/ufo/container/tree/set/iterator.hpp
			include/ufo/container/tree/set/nearest_iterator.hpp
			include/ufo/container/tree/set/query_iterator.hpp
			include/ufo/container/tree/set/query_nearest_iterator.hpp

			include/ufo/container/tree/block.hpp
			include/ufo/container/tree/code.hpp
			include/ufo/container/tree/container_bucket_iterator.hpp
			include/ufo/container/tree/container_iterator.hpp
			include/ufo/container/tree/container.hpp
			include/ufo/container/tree/coord.hpp
			include/ufo/container/tree/data.hpp
			include/ufo/container/tree/distance_node.hpp
			include/ufo/container/tree/index.hpp
			include/ufo/container/tree/iterator.hpp
			include/ufo/container/tree/key.hpp
			include/ufo/container/tree/nearest_iterator.hpp
			include/ufo/container/tree/node.hpp
			include/ufo/container/tree/predicate.hpp
			include/ufo/container/tree/query_iterator.hpp
			include/ufo/container/tree/query_nearest_iterator.hpp
			include/ufo/container/tree/render.hpp
			include/ufo/container/tree/trace_result.hpp
			include/ufo/container/tree/tree.hpp

			include/ufo/container/binary_tree_map.hpp
			include/ufo/container/binary_tree_set.hpp
			include/ufo/container/hextree_map.hpp
			include/ufo/container/hextree_set.hpp
			include/ufo/container/octree_map.hpp
			include/ufo/container/octree_set.hpp
			include/ufo/container/quadtree_map.hpp
			include/ufo/container/quadtree_set.hpp
			include/ufo/container/range_map.hpp
			include/ufo/container/range_set.hpp
			include/ufo/container/range.hpp
			include/ufo/container/small_vector.hpp
			include/ufo/container/structure_of_arrays.hpp
			include/ufo/container/tree_map.hpp
			include/ufo/container/tree_set.hpp
)

target_link_libraries(Container 
	INTERFACE
		UFO::Compute
		UFO::Execution
		UFO::Geometry 
		UFO::Morton 
		UFO::Utility
		UFO::Vision
)

if(UFO_BUILD_TESTS AND UFO_CONTAINER_BUILD_TESTS)
  add_subdirectory(tests)
endif()


install(
  TARGETS Container
	EXPORT UFOTargets
  FILE_SET HEADERS
	COMPONENT Container
)