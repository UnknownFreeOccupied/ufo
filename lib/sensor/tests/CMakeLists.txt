add_executable(SensorTests)

target_sources(SensorTests
	PRIVATE
		imu_test.cpp
)

set_target_properties(SensorTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 23
		CXX_EXTENSIONS OFF
		CXX_SCAN_FOR_MODULES OFF
)
target_link_libraries(SensorTests PRIVATE Catch2::Catch2WithMain UFO::Sensor UFO::Warnings)

catch_discover_tests(SensorTests)