add_executable(VizTests)

target_sources(VizTests
	PRIVATE
		map_test.cpp
		# path_test.cpp
		# viz_test.cpp
)

target_link_libraries(VizTests PRIVATE UFO::Viz Catch2::Catch2WithMain)

if (MSVC)
	target_compile_options(VizTests PRIVATE /W4)
else()
	target_compile_options(VizTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

catch_discover_tests(VizTests)