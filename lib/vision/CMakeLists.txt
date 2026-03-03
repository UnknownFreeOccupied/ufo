add_library(Vision INTERFACE)
add_library(UFO::Vision ALIAS Vision)

target_sources(Vision
	INTERFACE
  	FILE_SET HEADERS
    BASE_DIRS
			include
    FILES
			include/ufo/vision/camera.hpp
			include/ufo/vision/camera/orthogonal.hpp
			include/ufo/vision/camera/orthogonal_intrinsics.hpp
			include/ufo/vision/camera/perspective.hpp
			include/ufo/vision/camera/perspective_intrinsics.hpp

			include/ufo/vision/color/alpha.hpp
			include/ufo/vision/color/arithmetic.hpp
			include/ufo/vision/color/average.hpp
			include/ufo/vision/color/blend.hpp
			include/ufo/vision/color/concepts.hpp
			include/ufo/vision/color/convert.hpp
			include/ufo/vision/color/delta_e.hpp
			include/ufo/vision/color/detail/arithmetic.hpp
			include/ufo/vision/color/detail/convert.hpp
			include/ufo/vision/color/flags.hpp
			include/ufo/vision/color/gray.hpp
			include/ufo/vision/color.hpp
			include/ufo/vision/color/lab.hpp
			include/ufo/vision/color/lch.hpp
			include/ufo/vision/color/lrgb.hpp
			include/ufo/vision/color/map/black_body.hpp
			include/ufo/vision/color/map/extended_kindlmann.hpp
			include/ufo/vision/color/map.hpp
			include/ufo/vision/color/map/inferno.hpp
			include/ufo/vision/color/map/kindlmann.hpp
			include/ufo/vision/color/rgb.hpp
			include/ufo/vision/color/space.hpp
			include/ufo/vision/color/type_traits.hpp
			include/ufo/vision/color/weight.hpp

			include/ufo/vision/image.hpp

			include/ufo/vision/vision.hpp
)

target_link_libraries(Vision 
	INTERFACE 
		UFO::Execution
		UFO::Geometry
		UFO::Math
		UFO::Utility
)

if(UFO_BUILD_TESTS)
  add_subdirectory(tests)
endif()


install(
  TARGETS Vision
	EXPORT UFOTargets
  FILE_SET HEADERS
	COMPONENT Vision
)