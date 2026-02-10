add_executable(PlanTests)

target_sources(PlanTests
	PRIVATE
		plan_test.cpp
)

target_link_libraries(PlanTests PRIVATE UFO::Plan Catch2::Catch2WithMain)

target_compile_options(PlanTests
	PUBLIC
		-Wall
		-Werror
		-Wextra
		-pedantic
		-Wconversion
		-Wcast-align
		-Wunused
		# -Wshadow
		-Wold-style-cast
		-Wpointer-arith
		-Wcast-qual
		-Wno-missing-braces
)

catch_discover_tests(PlanTests)