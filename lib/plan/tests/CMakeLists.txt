add_executable(PlanTests)

target_sources(PlanTests
	PRIVATE
		plan_test.cpp
)

target_link_libraries(PlanTests PRIVATE UFO::Plan Catch2::Catch2WithMain)

if (MSVC)
	target_compile_options(PlanTests PRIVATE /W4)
else()
	target_compile_options(PlanTests PRIVATE -Wall -Wextra -Wpedantic)
endif()

catch_discover_tests(PlanTests)