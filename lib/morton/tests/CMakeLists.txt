add_executable(MortonTests)

target_sources(MortonTests
	PRIVATE
		morton1_test.cpp
		morton2_test.cpp
		morton3_test.cpp
		morton4_test.cpp
)

target_link_libraries(MortonTests PRIVATE UFO::Morton Catch2::Catch2WithMain)

target_compile_options(MortonTests
	PUBLIC
		# -Wall
		# -Werror
		# -Wextra
		# -pedantic
		# -Wconversion
		# -Wcast-align
		# -Wunused
		# # -Wshadow
		# -Wold-style-cast
		# -Wpointer-arith
		# -Wcast-qual
		# -Wno-missing-braces
)

catch_discover_tests(MortonTests)