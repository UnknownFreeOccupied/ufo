add_executable(CloudTests)

target_sources(CloudTests
	PRIVATE
		cloud_test.cpp
)

target_link_libraries(CloudTests PRIVATE UFO::Cloud Catch2::Catch2WithMain)

target_compile_options(CloudTests
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

catch_discover_tests(CloudTests)