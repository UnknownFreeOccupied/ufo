option(UFO_VISION_BUILD_DOCS     "Generate documentation" ON)
option(UFO_VISION_BUILD_TESTS    "Unit testing"           ON)
option(UFO_VISION_BUILD_COVERAGE "Test Coverage"          ON)

add_library(Vision INTERFACE)
add_library(UFO::Vision ALIAS Vision)

target_sources(Vision
	INTERFACE
  	FILE_SET HEADERS
    BASE_DIRS
			include
    FILES
			include/ufo/vision/color_map/black_body.hpp
			include/ufo/vision/color_map/extended_kindlmann.hpp
			include/ufo/vision/color_map/inferno.hpp
			include/ufo/vision/color_map/kindlmann.hpp

			include/ufo/vision/camera.hpp
			include/ufo/vision/color_map.hpp
			include/ufo/vision/color.hpp
			include/ufo/vision/image_io.hpp
			include/ufo/vision/image.hpp
			
			include/ufo/vision/stb_image_write.h
			include/ufo/vision/stb_image.h
)

target_link_libraries(Vision 
	INTERFACE 
		UFO::Geometry
		UFO::Math
		UFO::Utility
)

target_include_directories(Vision 
	INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(UFO_BUILD_TESTS AND UFO_VISION_BUILD_TESTS)
  add_subdirectory(tests)
endif()

if(UFO_BUILD_DOCS AND UFO_VISION_BUILD_DOCS)
	add_subdirectory(docs)
endif()

install(
  TARGETS Vision
	EXPORT UFOTargets
  FILE_SET HEADERS
	COMPONENT Vision
)