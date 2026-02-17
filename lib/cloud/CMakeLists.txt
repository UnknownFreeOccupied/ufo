option(UFO_CLOUD_BUILD_TESTS "Unit testing" ON)
option(UFO_CLOUD_BUILD_COVERAGE "Test Coverage" ON)

include(GNUInstallDirs)

add_library(Cloud INTERFACE)
add_library(UFO::Cloud ALIAS Cloud)

target_sources(Cloud
	INTERFACE
  	FILE_SET HEADERS
    BASE_DIRS
			include
    FILES
			include/ufo/cloud/cloud.hpp
			include/ufo/cloud/point_cloud.hpp
)

target_link_libraries(Cloud 
	INTERFACE
		UFO::Container
		UFO::Execution
		UFO::Utility
)

if(UFO_BUILD_TESTS AND UFO_CLOUD_BUILD_TESTS)
	add_subdirectory(tests)
endif()

install(
  TARGETS Cloud
	EXPORT UFOTargets
  FILE_SET HEADERS
	COMPONENT Cloud
)