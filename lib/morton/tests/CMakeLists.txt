add_executable(MortonTests)

target_sources(MortonTests
	PRIVATE
		morton_test.cpp
)

set_target_properties(MortonTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 23
		CXX_EXTENSIONS OFF
		CXX_SCAN_FOR_MODULES OFF
)

target_link_libraries(MortonTests PRIVATE Catch2::Catch2WithMain UFO::Morton UFO::Warnings)

catch_discover_tests(MortonTests)