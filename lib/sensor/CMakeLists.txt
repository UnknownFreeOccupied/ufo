add_library(Sensor INTERFACE)
add_library(UFO::Sensor ALIAS Sensor)

target_sources(Sensor
	INTERFACE
  	FILE_SET HEADERS
    BASE_DIRS
			include
    FILES
			include/ufo/sensor/imu.hpp
			include/ufo/sensor/intensity.hpp
)

target_link_libraries(Sensor 
	INTERFACE
		UFO::Math
)

if(UFO_BUILD_TESTS)
	add_subdirectory(tests)
endif()

install(
  TARGETS Sensor
	EXPORT UFOTargets
  FILE_SET HEADERS
	COMPONENT Sensor
)