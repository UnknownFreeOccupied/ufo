# add_executable(Tests)
# target_link_libraries(Tests PRIVATE Catch2::Catch2WithMain)

# set_target_properties(Tests
# 	PROPERTIES
# 		VERSION ${PROJECT_VERSION}
# 		SOVERSION ${PROJECT_VERSION_MAJOR}
# 		CXX_STANDARD 17
# 		CXX_EXTENSIONS OFF
# )

# if (MSVC)
# 	target_compile_options(Tests PRIVATE /W4)
# else()
# 	target_compile_options(Tests PRIVATE -Wall -Wextra -Wpedantic)
# endif()

# include(CTest)
# include(Catch)
# catch_discover_tests(Tests)