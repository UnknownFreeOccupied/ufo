# include(FetchContent)

# FetchContent_Declare(
# 	libjpeg-turbo
# 	GIT_REPOSITORY https://github.com/libjpeg-turbo/libjpeg-turbo.git
# 	GIT_TAG        4e151a4ad91001b3aa8c2ece2205c15f487ce320 # 3.1.2
# 	GIT_PROGRESS   TRUE
# )

# FetchContent_MakeAvailable(libjpeg-turbo)

include(ExternalProject)
include(GNUInstallDirs)

if(MSVC)
    set(JPEG_TURBO_LIBRARIES "turbojpeg-static")
else()
    set(JPEG_TURBO_LIBRARIES "turbojpeg")
endif()

ExternalProject_Add(
	libjpeg_turbo
	GIT_REPOSITORY https://github.com/libjpeg-turbo/libjpeg-turbo.git
	GIT_TAG        4e151a4ad91001b3aa8c2ece2205c15f487ce320 # 3.1.2
  DOWNLOAD_DIR   "${CMAKE_BINARY_DIR}/_deps/libjpeg-turbo-src"
  UPDATE_COMMAND ""
  CMAKE_ARGS 
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    -DENABLE_STATIC=ON
    -DENABLE_SHARED=OFF
    -UCMAKE_INSTALL_LIBDIR
    -DCMAKE_INSTALL_DEFAULT_LIBDIR=${CMAKE_INSTALL_LIBDIR}
    <INSTALL_DIR>/${CMAKE_INSTALL_LIBDIR}/${CMAKE_STATIC_LIBRARY_PREFIX}${JPEG_TURBO_LIBRARIES}${CMAKE_STATIC_LIBRARY_SUFFIX}
)