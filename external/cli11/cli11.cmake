include(FetchContent)

FetchContent_Declare(
	cli11_proj
	GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
	GIT_TAG        bfffd37e1f804ca4fae1caae106935791696b6a9 # v2.6.1
	GIT_PROGRESS   TRUE
)
FetchContent_MakeAvailable(cli11_proj)