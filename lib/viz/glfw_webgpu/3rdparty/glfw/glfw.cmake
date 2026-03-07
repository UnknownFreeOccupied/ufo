message(CHECK_START "Finding glfw")
find_package(glfw3 3.4 QUIET)

if(glfw3_FOUND)
	message(CHECK_PASS "found, it is installed on the system")
else()
	message(CHECK_FAIL "not found, will fetch it instead")

	Include(FetchContent)

	FetchContent_Declare(
		glfw
		GIT_REPOSITORY https://github.com/glfw/glfw.git
		GIT_TAG 7b6aead9fb88b3623e3b3725ebb42670cbe4c579 # 3.4
		GIT_PROGRESS TRUE
	)

	set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
	set(GLFW_BUILD_TESTS. OFF CACHE BOOL "" FORCE)
	set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

	FetchContent_MakeAvailable(glfw)
endif()