option(UFO_IO_BUILD_DOCS "Generate documentation" ON)
option(UFO_IO_BUILD_TESTS "Unit testing" ON)
option(UFO_IO_BUILD_COVERAGE "Test Coverage" ON)

include(GNUInstallDirs)

add_library(IO STATIC)
add_library(UFO::IO ALIAS IO)

target_sources(IO
	PRIVATE
		src/file_handler.cpp
		src/file_type.cpp
		src/io.cpp
		src/jpeg.cpp
		src/obj.cpp
		src/pcd.cpp
		src/ply.cpp
		src/png.cpp
		src/pts.cpp
		src/qtp.cpp
		src/ufo.cpp
		src/xyz.cpp
		src/xyzi.cpp
		src/xyzn.cpp
		src/xyzrgb.cpp

	PUBLIC
  	FILE_SET HEADERS
    BASE_DIRS
			include
    FILES
			include/ufo/io/cloud_properties.hpp
			include/ufo/io/file_handler.hpp
			include/ufo/io/file_type.hpp
			include/ufo/io/image_properties.hpp
			include/ufo/io/io.hpp
			include/ufo/io/jpeg.hpp
			include/ufo/io/obj.hpp
			include/ufo/io/pcd.hpp
			include/ufo/io/ply.hpp
			include/ufo/io/png.hpp
			include/ufo/io/pts.hpp
			include/ufo/io/qtp.hpp
			include/ufo/io/ufo.hpp
			include/ufo/io/xyz.hpp
			include/ufo/io/xyzi.hpp
			include/ufo/io/xyzn.hpp
			include/ufo/io/xyzrgb.hpp
)

set_target_properties(IO
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
		CXX_STANDARD 23
		CXX_EXTENSIONS OFF
		OUTPUT_NAME "UFOIO"
		POSITION_INDEPENDENT_CODE ON
)

ExternalProject_Get_Property(libjpeg_turbo INSTALL_DIR)
set(JPEG_TURBO_INCLUDE_DIRS ${INSTALL_DIR}/include/) # "/" is critical.
set(JPEG_TURBO_LIB_DIR ${INSTALL_DIR}/${CMAKE_INSTALL_LIBDIR})
if(MSVC)
    set(JPEG_TURBO_LIBRARIES "turbojpeg-static")
else()
    set(JPEG_TURBO_LIBRARIES "turbojpeg")
endif()

add_dependencies(IO libjpeg_turbo)
target_include_directories(IO PRIVATE ${JPEG_TURBO_INCLUDE_DIRS})
target_link_directories(IO PRIVATE ${JPEG_TURBO_LIB_DIR})

target_link_libraries(IO 
	PUBLIC
		UFO::Cloud
		UFO::Core
		UFO::Execution
		UFO::Math
		UFO::Sensor
		UFO::Vision

	PRIVATE
		rply
		spng
		${JPEG_TURBO_LIBRARIES}
)

if(UFO_BUILD_TESTS AND UFO_IO_BUILD_TESTS)
	add_subdirectory(tests)
endif()

install(
  TARGETS IO rply
	EXPORT UFOTargets
  FILE_SET HEADERS
	COMPONENT IO
)