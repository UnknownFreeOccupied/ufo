option(UFO_MAP_BUILD_TESTS "Unit testing" ON)
option(UFO_MAP_BUILD_COVERAGE "Test Coverage" ON)

include(GNUInstallDirs)

add_library(Map INTERFACE)
add_library(UFO::Map ALIAS Map)

target_sources(Map
	INTERFACE
  	FILE_SET HEADERS
    BASE_DIRS
			include
    FILES
			include/ufo/map/color/predicate/alpha_interval.hpp
			include/ufo/map/color/predicate/alpha.hpp
			include/ufo/map/color/predicate/color_map.hpp
			include/ufo/map/color/predicate/has_color.hpp

			include/ufo/map/color/block.hpp
			include/ufo/map/color/map.hpp
			include/ufo/map/color/predicate.hpp

			include/ufo/map/cost/predicate/cost_interval.hpp
			include/ufo/map/cost/predicate/cost.hpp

			include/ufo/map/cost/block.hpp
			include/ufo/map/cost/map.hpp
			include/ufo/map/cost/predicate.hpp
			include/ufo/map/cost/propagation_criteria.hpp

			include/ufo/map/count/predicate/count_interval.hpp
			include/ufo/map/count/predicate/count_map.hpp
			include/ufo/map/count/predicate/count.hpp

			include/ufo/map/count/count_map.hpp
			include/ufo/map/count/count_predicate.hpp

			include/ufo/map/distance/block.hpp
			include/ufo/map/distance/info.hpp
			include/ufo/map/distance/interpolate.hpp
			include/ufo/map/distance/map.hpp
			include/ufo/map/distance/predicate.hpp

			include/ufo/map/integrator/detail/bool_grid.hpp
			include/ufo/map/integrator/detail/count_grid.hpp
			include/ufo/map/integrator/detail/grid_map.hpp
			include/ufo/map/integrator/detail/hit_grid.hpp
			include/ufo/map/integrator//detail/hit.hpp
			include/ufo/map/integrator/detail/miss_grid.hpp
			include/ufo/map/integrator/detail/miss.hpp
			include/ufo/map/integrator/detail/simple_integrator.hpp

			include/ufo/map/integrator/angular_integrator.hpp
			include/ufo/map/integrator/count_sampling_method.hpp
			include/ufo/map/integrator/integrator.hpp
			include/ufo/map/integrator/misses.hpp
			include/ufo/map/integrator/sensor_error.hpp

			include/ufo/map/intensity/predicate/intensity_interval.hpp
			include/ufo/map/intensity/predicate/intensity_map.hpp
			include/ufo/map/intensity/predicate/intensity.hpp

			include/ufo/map/intensity/intensity_map.hpp
			include/ufo/map/intensity/intensity_predicate.hpp

			include/ufo/map/label/predicate/label_map.hpp
			include/ufo/map/label/predicate/label.hpp

			include/ufo/map/label/block.hpp
			include/ufo/map/label/map.hpp
			include/ufo/map/label/predicate.hpp

			include/ufo/map/labels/predicate/has_label.hpp
			include/ufo/map/labels/predicate/label.hpp

			include/ufo/map/labels/block.hpp
			include/ufo/map/labels/label_set.hpp
			include/ufo/map/labels/labels_map.hpp
			include/ufo/map/labels/map.hpp
			include/ufo/map/labels/predicate.hpp
			include/ufo/map/labels/propagation_criteria.hpp

			include/ufo/map/mesh/block.hpp
			include/ufo/map/mesh/map.hpp

			include/ufo/map/occupancy/predicate/contains_occupancy_state.hpp
			include/ufo/map/occupancy/predicate/contains_occupancy_states.hpp
			include/ufo/map/occupancy/predicate/occupancy_interval.hpp
			include/ufo/map/occupancy/predicate/occupancy_map.hpp
			include/ufo/map/occupancy/predicate/occupancy_state.hpp
			include/ufo/map/occupancy/predicate/occupancy_states.hpp
			include/ufo/map/occupancy/predicate/occupancy.hpp

			include/ufo/map/occupancy/block.hpp
			include/ufo/map/occupancy/map.hpp
			include/ufo/map/occupancy/predicate.hpp
			include/ufo/map/occupancy/propagation_criteria.hpp
			include/ufo/map/occupancy/state.hpp

			include/ufo/map/predicate/dynamic.hpp
			include/ufo/map/predicate/predicate_compare.hpp

			include/ufo/map/ray_caster/ray_caster.hpp

			include/ufo/map/reflection/predicate/hits_interval.hpp
			include/ufo/map/reflection/predicate/hits.hpp
			include/ufo/map/reflection/predicate/misses_interval.hpp
			include/ufo/map/reflection/predicate/misses.hpp
			include/ufo/map/reflection/predicate/reflection_map.hpp
			include/ufo/map/reflection/predicate/reflectiveness_interval.hpp
			include/ufo/map/reflection/predicate/reflectiveness.hpp

			include/ufo/map/reflection/reflection_map.hpp
			include/ufo/map/reflection/reflection_predicate.hpp
			include/ufo/map/reflection/reflection.hpp

			include/ufo/map/semantic/semantic_map.hpp
			include/ufo/map/semantic/semantic_mapping.hpp
			include/ufo/map/semantic/semantic_predicate.hpp
			include/ufo/map/semantic/semantic.hpp

			include/ufo/map/semantic_set/semantic_propagation.hpp
			include/ufo/map/semantic_set/semantic_set_block.hpp
			include/ufo/map/semantic_set/semantic_set_map.hpp
			include/ufo/map/semantic_set/semantic_set_predicate.hpp
			include/ufo/map/semantic_set/semantic_set.hpp
			include/ufo/map/semantic_set/semantic_util.hpp

			include/ufo/map/surfel/surfel_map.hpp
			include/ufo/map/surfel/surfel_predicate.hpp
			include/ufo/map/surfel/surfel.hpp

			include/ufo/map/time/predicate/time_interval.hpp
			include/ufo/map/time/predicate/time_map.hpp
			include/ufo/map/time/predicate/time.hpp

			include/ufo/map/time/time_map.hpp
			include/ufo/map/time/time_predicate.hpp

			include/ufo/map/view_direction/block.hpp
			include/ufo/map/view_direction/map.hpp
			include/ufo/map/view_direction/predicate.hpp

			include/ufo/map/void_region/predicate/void_region_map.hpp
			include/ufo/map/void_region/predicate/void_region.hpp
			include/ufo/map/void_region/block.hpp
			include/ufo/map/void_region/map.hpp
			include/ufo/map/void_region/predicate.hpp

			include/ufo/map/block.hpp
			include/ufo/map/header.hpp
			include/ufo/map/map_full.hpp
			include/ufo/map/type.hpp
			include/ufo/map/ufomap.hpp
			include/ufo/map/utility.hpp
)

target_link_libraries(Map
	INTERFACE
		UFO::Cloud
		UFO::Compute
		UFO::Container
		UFO::Core
		UFO::Execution
		UFO::Geometry
		UFO::Math
		UFO::Morton
		UFO::Time
		UFO::Utility
		UFO::Vision
)

if(UFO_BUILD_TESTS AND UFO_MAP_BUILD_TESTS)
    add_subdirectory(tests)
endif()

install(
  TARGETS Map
	EXPORT UFOTargets
  FILE_SET HEADERS
	COMPONENT Map
)