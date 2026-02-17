Include(FetchContent)

FetchContent_Declare(
  doxygen
  GIT_REPOSITORY https://github.com/doxygen/doxygen.git
  GIT_TAG        669aeeefca743c148e2d935b3d3c69535c7491e6 # 1.16.1
  GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(doxygen)