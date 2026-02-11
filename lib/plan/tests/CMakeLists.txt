add_executable(PlanTests)

target_sources(PlanTests
	PRIVATE
		plan_test.cpp
)

set_target_properties(PlanTests
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 17
		CXX_EXTENSIONS OFF
)

if (MSVC)
	target_compile_options(PlanTests PRIVATE /W4)
else()
	target_compile_options(PlanTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

target_link_libraries(PlanTests PRIVATE UFO::Plan Catch2::Catch2WithMain)

catch_discover_tests(PlanTests)