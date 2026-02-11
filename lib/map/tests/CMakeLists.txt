add_executable(MapTests)

target_sources(MapTests
	PRIVATE
		# map_test.cpp
		# integration_test.cpp
		occupancy_map_test.cpp
		# labels_map_test.cpp
		label_map_test.cpp
)

set_target_properties(MapTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 17
		CXX_EXTENSIONS OFF
)

if (MSVC)
	target_compile_options(MapTests PRIVATE /W4)
else()
	target_compile_options(MapTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(MapTests PRIVATE UFO::Map Catch2::Catch2WithMain)

catch_discover_tests(MapTests)