add_executable(UtilityTests)

target_sources(UtilityTests
	PRIVATE
		create_array_test.cpp
		spinlock_test.cpp
		string_test.cpp
)

set_target_properties(UtilityTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 23
		CXX_EXTENSIONS OFF
		CXX_SCAN_FOR_MODULES OFF
)

target_link_libraries(UtilityTests PRIVATE Catch2::Catch2WithMain UFO::Utility UFO::Warnings)

catch_discover_tests(UtilityTests)