option(UFO_PLAN_BUILD_TESTS    "Unit testing"           ON)
option(UFO_PLAN_BUILD_COVERAGE "Test Coverage"          ON)

include(GNUInstallDirs)

add_library(Plan INTERFACE)
add_library(UFO::Plan ALIAS Plan)

target_sources(Plan
	INTERFACE
  	FILE_SET HEADERS
    BASE_DIRS
			include
    FILES
			include/ufo/plan/edge.hpp
			include/ufo/plan/nav_map.hpp
			include/ufo/plan/node.hpp
)

target_link_libraries(Plan 
	INTERFACE
		UFO::Container
		UFO::Map
		UFO::Math
)

if(UFO_BUILD_TESTS AND UFO_PLAN_BUILD_TESTS)
  add_subdirectory(tests)
endif()


install(
  TARGETS Plan
	EXPORT UFOTargets
  FILE_SET HEADERS
	COMPONENT Plan
)