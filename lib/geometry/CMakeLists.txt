add_library(Geometry INTERFACE)
add_library(UFO::Geometry ALIAS Geometry)

target_sources(Geometry
	INTERFACE
  	FILE_SET HEADERS
    BASE_DIRS
			include
    FILES
			include/ufo/geometry/detail/closest_point.hpp
			include/ufo/geometry/detail/contains.hpp
			include/ufo/geometry/detail/distance.hpp
			include/ufo/geometry/detail/fun.hpp
			include/ufo/geometry/detail/helper.hpp
			include/ufo/geometry/detail/intersects.hpp

			include/ufo/geometry/aabb.hpp
			include/ufo/geometry/capsule.hpp
			include/ufo/geometry/closest_point.hpp
			include/ufo/geometry/contains.hpp
			include/ufo/geometry/cylinder.hpp
			include/ufo/geometry/disjoint.hpp
			include/ufo/geometry/distance.hpp
			include/ufo/geometry/ellipsoid.hpp
			include/ufo/geometry/frustum.hpp
			include/ufo/geometry/fun.hpp
			include/ufo/geometry/geometry.hpp
			include/ufo/geometry/inside.hpp
			include/ufo/geometry/intersects.hpp
			include/ufo/geometry/line_segment.hpp
			include/ufo/geometry/line.hpp
			include/ufo/geometry/obb.hpp
			include/ufo/geometry/plane.hpp
			include/ufo/geometry/ray.hpp
			include/ufo/geometry/sphere.hpp
			include/ufo/geometry/triangle.hpp
)

target_link_libraries(Geometry 
	INTERFACE 
		UFO::Numeric
)

if(UFO_BUILD_TESTS)
  add_subdirectory(tests)
endif()


install(
  TARGETS Geometry
	EXPORT UFOTargets
  FILE_SET HEADERS
	COMPONENT Geometry
)