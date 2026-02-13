add_executable(VizTests)

target_sources(VizTests
	PRIVATE
		map_test.cpp
		# path_test.cpp
		# viz_test.cpp
)

set_target_properties(VizTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 17
		CXX_EXTENSIONS OFF
)

if (MSVC)
	target_compile_options(VizTests PRIVATE /W4)
else()
	target_compile_options(VizTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(VizTests PRIVATE UFO::Viz Catch2::Catch2WithMain)

catch_discover_tests(VizTests)