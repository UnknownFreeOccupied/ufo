add_executable(App)

target_sources(App
	PRIVATE
		src/system_info.cpp
		src/app.cpp
	PUBLIC
  	FILE_SET HEADERS
    BASE_DIRS
			include
    FILES
			include/ufo/apps/system_info.hpp
)

set_target_properties(App
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 23
		CXX_EXTENSIONS OFF
		CXX_SCAN_FOR_MODULES OFF
		OUTPUT_NAME "ufo"
)

target_link_libraries(App PRIVATE CLI11::CLI11 UFO::UFO)

if(WIN32)
	target_link_libraries(App PRIVATE advapi32)
endif()

if (MSVC)
	target_compile_options(App PRIVATE /W4 /Zc:__cplusplus)
else()
	target_compile_options(App PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(UFO_ENABLE_LTO)
	include(CheckIPOSupported)
	check_ipo_supported(RESULT lto_supported OUTPUT lto_error)

	if(lto_supported)
		set_target_properties(App PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
		message(STATUS "LTO: Enabled (Cross-file optimizations active)")
	else()
		message(WARNING "LTO: Not supported by this compiler/linker: ${lto_error}")
	endif()
endif()

install(
  TARGETS App
	EXPORT UFOTargets
  FILE_SET HEADERS
)