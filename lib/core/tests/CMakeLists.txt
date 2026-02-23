add_executable(CoreTests)

target_sources(CoreTests
	PRIVATE
		confidence_test.cpp
		intensity_test.cpp
		label_test.cpp
		normal_test.cpp
		range_test.cpp
		semantic_test.cpp
		surfel_test.cpp
		weight_test.cpp
)

set_target_properties(CoreTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 23
		CXX_EXTENSIONS OFF
)

if (MSVC)
	target_compile_options(CoreTests PRIVATE /W4)
else()
	target_compile_options(CoreTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(CoreTests PRIVATE Catch2::Catch2WithMain UFO::Core)

catch_discover_tests(CoreTests)