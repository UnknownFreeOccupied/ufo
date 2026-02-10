include(FetchContent)

FetchContent_Declare(
	libspng
	GIT_REPOSITORY https://github.com/randy408/libspng.git
	GIT_TAG        fb768002d4288590083a476af628e51c3f1d47cd # v0.7.4
	GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(libspng)