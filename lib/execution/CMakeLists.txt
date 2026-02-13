option(UFO_EXECUTION_BUILD_DOCS "Generate documentation" ON)
option(UFO_EXECUTION_BUILD_TESTS "Unit testing" ON)
option(UFO_EXECUTION_BUILD_COVERAGE "Test Coverage" ON)

include(GNUInstallDirs)

add_library(Execution INTERFACE)
add_library(UFO::Execution ALIAS Execution)

target_link_libraries(Execution INTERFACE UFO::Utility)

# Threading library requires explicit linking on Linux (-pthread); macOS and
# Windows provide thread support automatically without a separate link flag.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
	find_package(Threads REQUIRED)
	target_link_libraries(Execution INTERFACE Threads::Threads)
endif()

include(CheckCXXSourceCompiles)
include(CMakePushCheckState)

message(CHECK_START "Finding parallel processing libraries")
list(APPEND CMAKE_MESSAGE_INDENT "  ")
unset(foundParallelLibs)
unset(missingParallelLibs)

message(CHECK_START "Finding Threading Building Blocks (TBB)")

find_package(TBB 2021 QUIET CONFIG)

if(TBB_FOUND)
	message(CHECK_PASS "found and enabled")
	target_link_libraries(Execution INTERFACE TBB::tbb)
	target_compile_definitions(Execution INTERFACE UFO_PAR_TBB=1)
	list(APPEND foundParallelLibs TBB)
else()
	message(CHECK_FAIL "not found and disabled")
	list(APPEND missingParallelLibs TBB)
endif()

message(CHECK_START "Checking Standard Template Library (STL) parallel support")

cmake_push_check_state()

set(CMAKE_REQUIRED_FLAGS "-O0")
if(TBB_FOUND)
	set(CMAKE_REQUIRED_LIBRARIES "TBB::tbb")
endif()

check_cxx_source_compiles("
		#include <algorithm>
		#include <array>
		#include <execution>
		int main() {
			std::array<int, 10> a{};
			std::for_each(std::execution::par, a.begin(), a.end(), [](auto){});
			(void)a;
			return 0;
		}
	" HAVE_STL_PAR)

if(HAVE_STL_PAR)
	message(CHECK_PASS "has native support")
	target_compile_definitions(Execution INTERFACE UFO_PAR_STL=1)
	list(APPEND foundParallelLibs STL)
else()
	message(CHECK_FAIL "no support")
	list(APPEND missingParallelLibs STL)
endif()

cmake_pop_check_state()

message(CHECK_START "Finding Grand Central Dispatch (GCD)")

if(APPLE)
	message(CHECK_PASS "found and enabled")
	target_compile_definitions(Execution INTERFACE UFO_PAR_GCD=1)
	list(APPEND foundParallelLibs GCD)
else()
	message(CHECK_FAIL "not found and disabled")
	list(APPEND missingParallelLibs GCD)
endif()

message(CHECK_START "Finding OpenMP (OMP)")
find_package(OpenMP QUIET)

if(OpenMP_CXX_FOUND)
	message(CHECK_PASS "found and enabled")
	target_link_libraries(Execution INTERFACE OpenMP::OpenMP_CXX)
	target_compile_definitions(Execution INTERFACE UFO_PAR_OMP=1)
	list(APPEND foundParallelLibs OMP)
else()
	message(CHECK_FAIL "not found and disabled")
	list(APPEND missingParallelLibs OMP)
endif()

list(POP_BACK CMAKE_MESSAGE_INDENT)

if(missingParallelLibs)
	if(foundParallelLibs)
		message(CHECK_FAIL "some parallel processing libraries found but some are missing. Enabled: [${foundParallelLibs}] Disabled: [${missingParallelLibs}]")
	else()
		message(CHECK_FAIL "no parallel processing libraries found, all disabled: [${missingParallelLibs}]")
		message(WARNING "Execution module will be built but no parallel processing libraries will be enabled. This may lead to suboptimal performance. To enable parallel processing, install one of the supported libraries, oneTBB or OpenMP, and rebuild UFO.")
	endif()
else()
	message(CHECK_PASS "all parallel processing libraries found and enabled: [${foundParallelLibs}]")
endif()

target_include_directories(Execution
	INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(UFO_BUILD_TESTS AND UFO_EXECUTION_BUILD_TESTS)
	add_subdirectory(tests)
endif()

if(UFO_BUILD_DOCS AND UFO_EXECUTION_BUILD_DOCS)
	add_subdirectory(docs)
endif()

install(TARGETS Execution 
	EXPORT UFO-targets
	COMPONENT Execution
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
	COMPONENT Execution
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)