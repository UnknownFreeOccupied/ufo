set(wgpu "/home/dduberg/ufo_ws/src/ufo/lib/compute/wgpu")

find_library(wgpu-native_LIBRARIES NAMES "wgpu_native" HINTS "${wgpu}/lib")
# find_path(wgpu-native_INCLUDE_DIRS NAMES "webgpu" HINTS "${wgpu}/include")

set(wgpu-native_INCLUDE_DIRS "${wgpu}/include")

message(WARNING "wgpu-native_LIBRARIES: ${wgpu-native_LIBRARIES}")
message(WARNING "wgpu-native_INCLUDE_DIRS: ${wgpu-native_INCLUDE_DIRS}")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(wgpu-native DEFAULT_MSG wgpu-native_INCLUDE_DIRS wgpu-native_LIBRARIES)

if(wgpu-native_FOUND AND NOT TARGET wgpu-native)
	add_library(wgpu-native SHARED IMPORTED)
	set_target_properties(wgpu-native PROPERTIES
		IMPORTED_LOCATION "${wgpu-native_LIBRARIES}"
		# IMPORTED_NO_SONAME TRUE
		INTERFACE_INCLUDE_DIRECTORIES "${wgpu-native_INCLUDE_DIRS}"
	)
	# target_include_directories(wgpu-native
	# 	INTERFACE
	# 		$<BUILD_INTERFACE:${native_INCLUDE_DIRS}>
	# 		$<INSTALL_INTERFACE:include>
	# )
	# install(FILES "${wgpu-native_LIBRARIES}" DESTINATION lib)
	# install(DIRECTORY "${wgpu-native_INCLUDE_DIRS}/" DESTINATION include)
endif()

mark_as_advanced(wgpu-native_INCLUDE_DIRS wgpu-native_LIBRARIES)