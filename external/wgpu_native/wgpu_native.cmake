include(FetchContent)
include(GNUInstallDirs)

set(WGPU_VERSION "v27.0.4.0" CACHE STRING "\
	Version of the wgpu-native WebGPU implementation to use. Must correspond \
	to the tag name of an existing release on https://github.com/gfx-rs/wgpu-native/releases.")

set(WGPU_BINARY_MIRROR "https://github.com/gfx-rs/wgpu-native" CACHE STRING "\
	The repository where to find precompiled releases of wgpu-native.")

if (NOT ARCH)
	set(SYSTEM_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR})
	if (SYSTEM_PROCESSOR STREQUAL "AMD64" OR SYSTEM_PROCESSOR STREQUAL "x86_64")
		if (CMAKE_SIZEOF_VOID_P EQUAL 8)
			set(ARCH "x86_64")
		elseif (CMAKE_SIZEOF_VOID_P EQUAL 4)
			set(ARCH "i686")
		endif()
	elseif (SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|armv8|arm)$")
		set(ARCH "aarch64")
	elseif(SYSTEM_PROCESSOR MATCHES "^(armv7|armv6|armhf)$")
		set(ARCH "arm")
	else()
		message(WARNING "Unknown architecture: ${SYSTEM_PROCESSOR}")
		set(ARCH "unknown")
	endif()
endif()

set(URL_OS)
set(URL_COMPILER)
set(OS_LIBRARIES)
set(SHARED_LIB_PREFIX)
set(STATIC_LIB_PREFIX)
set(SHARED_LIB_EXT)
set(STATIC_LIB_EXT)
if (CMAKE_SYSTEM_NAME STREQUAL "Windows")
	set(URL_OS "windows")
	set(OS_LIBRARIES d3dcompiler ws2_32 userenv bcrypt ntdll opengl32 Propsys RuntimeObject)
	set(SHARED_LIB_EXT "dll")
	if (MSVC)
		set(URL_COMPILER "-msvc")
		set(STATIC_LIB_EXT "lib")
	else()
		set(URL_COMPILER "-gnu")
		set(STATIC_LIB_EXT "a")
		set(STATIC_LIB_PREFIX "lib")
	endif()
elseif (CMAKE_SYSTEM_NAME STREQUAL "Linux")
	set(OS_LIBRARIES "-lm -ldl")
	set(URL_OS "linux")
	set(STATIC_LIB_EXT "a")
	set(SHARED_LIB_EXT "so")
	set(STATIC_LIB_PREFIX "lib")
	set(SHARED_LIB_PREFIX "lib")
elseif (CMAKE_SYSTEM_NAME STREQUAL "Darwin")
	set(OS_LIBRARIES "-framework Foundation -framework CoreFoundation -framework QuartzCore -framework Metal")
	set(URL_OS "macos")
	set(STATIC_LIB_EXT "a")
	set(SHARED_LIB_EXT "dylib")
	set(STATIC_LIB_PREFIX "lib")
	set(SHARED_LIB_PREFIX "lib")
else()
	message(FATAL_ERROR "Platform system '${CMAKE_SYSTEM_NAME}' not supported by this release of UFOCompute.")
endif()

set(URL_ARCH)
if (ARCH STREQUAL "x86_64")
	set(URL_ARCH "x86_64")
elseif (ARCH STREQUAL "aarch64")
	set(URL_ARCH "aarch64")
elseif (ARCH STREQUAL "i686" AND CMAKE_SYSTEM_NAME STREQUAL "Windows")
	set(URL_ARCH "i686")
else()
	message(FATAL_ERROR "Platform architecture '${ARCH}' not supported by this release of UFOCompute.")
endif()

set(URL_CONFIG "release")
set(URL_NAME "wgpu-${URL_OS}-${URL_ARCH}${URL_COMPILER}-${URL_CONFIG}")
set(URL "${WGPU_BINARY_MIRROR}/releases/download/${WGPU_VERSION}/${URL_NAME}.zip")

string(TOLOWER "${URL_NAME}" FC_NAME)

FetchContent_Declare(${FC_NAME}
	URL ${URL}
)

message(STATUS "Fetching WebGPU implementation from '${URL}'")
FetchContent_MakeAvailable(${FC_NAME})
set(WGPU_DIR "${${FC_NAME}_SOURCE_DIR}")
message(STATUS "WebGPU implementation source directory: ${WGPU_DIR}")

set(WGPU_BUILD_SHARED_RUNTIME_LIB "${WGPU_DIR}/lib/${SHARED_LIB_PREFIX}wgpu_native.${SHARED_LIB_EXT}")
set(WGPU_BUILD_STATIC_RUNTIME_LIB "${WGPU_DIR}/lib/${STATIC_LIB_PREFIX}wgpu_native.${STATIC_LIB_EXT}")

set(WGPU_INSTALL_SHARED_RUNTIME_LIB "${CMAKE_INSTALL_FULL_LIBDIR}/${SHARED_LIB_PREFIX}wgpu_native.${SHARED_LIB_EXT}")
set(WGPU_INSTALL_STATIC_RUNTIME_LIB "${CMAKE_INSTALL_FULL_LIBDIR}/${STATIC_LIB_PREFIX}wgpu_native.${STATIC_LIB_EXT}")

# Create a logical target for the external library
add_library(wgpu_native STATIC IMPORTED GLOBAL)

# Tell CMake where the file is DURING THE BUILD
set_target_properties(wgpu_native PROPERTIES
  IMPORTED_LOCATION                "${WGPU_BUILD_STATIC_RUNTIME_LIB}"
)

# Generate and install a cmake target file so consumers of the installed UFO
# package can find and link the wgpu_native static libraries.
configure_file(
	"${CMAKE_CURRENT_LIST_DIR}/wgpu_native-targets.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/wgpu_native-targets.cmake"
	@ONLY
)

install(
	FILES "${CMAKE_CURRENT_BINARY_DIR}/wgpu_native-targets.cmake"
	DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/UFO"
	COMPONENT Compute
)

install(
	FILES ${WGPU_BUILD_SHARED_RUNTIME_LIB} ${WGPU_BUILD_STATIC_RUNTIME_LIB}
	DESTINATION ${CMAKE_INSTALL_LIBDIR}
	COMPONENT Compute
)

install(DIRECTORY ${WGPU_DIR}/include/
	COMPONENT Compute
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)