# --- ufo/utility/CMakeLists.txt ---
include(GNUInstallDirs)

# 1. Define the library
# Note: Since it has .cpp files, it's SHARED/STATIC (usually STATIC in monorepos)
add_library(Utility STATIC 
    src/io/read_buffer.cpp
    src/io/write_buffer.cpp
    src/io/buffer.cpp
)
add_library(UFO::Utility ALIAS Utility)

# 2. Dependencies
find_package(TBB) #should ideally be in the root Dependencies.cmake
# but we link it here.
target_link_libraries(Utility PUBLIC TBB::tbb)

target_compile_definitions(Utility
    PUBLIC
        UFO_TBB=1
)

# 3. Standard and Properties
# You can keep these if this lib has unique requirements, 
# but usually CXX_STANDARD is set globally at the root.
set_target_properties(Utility PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# 4. Include Directories
# Use the "Hall of Mirrors" fix: point directly to your include folder
target_include_directories(Utility 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# 5. Global Install (Simplified)
# We add this target to the shared 'UFO-targets' export set
install(TARGETS Utility 
	EXPORT UFO-targets
    COMPONENT Utility
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    COMPONENT Utility
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)