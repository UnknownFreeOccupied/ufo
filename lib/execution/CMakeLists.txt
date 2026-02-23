# Backend toggle options
option(UFO_WITH_TBB    "Build with Threading Building Blocks (TBB) support" ON)
option(UFO_WITH_OPENMP "Build with OpenMP support"                          ON)
option(UFO_WITH_GCD    "Build with Grand Central Dispatch (GCD) support"    ON)

add_library(Execution INTERFACE)
add_library(UFO::Execution ALIAS Execution)

target_sources(Execution
	INTERFACE
  	FILE_SET HEADERS
    BASE_DIRS
			include
    FILES
			include/ufo/execution/algorithm.hpp
			include/ufo/execution/execution.hpp
)

target_link_libraries(Execution INTERFACE UFO::Utility)

# Portable threading support
find_package(Threads REQUIRED)
target_link_libraries(Execution INTERFACE Threads::Threads)

include(CheckCXXSourceCompiles)
include(CMakePushCheckState)

message(CHECK_START "Finding parallel processing libraries")
list(APPEND CMAKE_MESSAGE_INDENT "  ")
unset(foundParallelLibs)
unset(missingParallelLibs)

# --- TBB ---
if(UFO_WITH_TBB)
	message(CHECK_START "Finding Threading Building Blocks (TBB)")
	find_package(TBB 2021 QUIET CONFIG)
	if(NOT TBB_FOUND)
		# Fallback to MODULE mode if CONFIG fails
		find_package(TBB 2021 QUIET)
	endif()

	if(TBB_FOUND)
		message(CHECK_PASS "found and enabled")
		target_link_libraries(Execution INTERFACE TBB::tbb)
		target_compile_definitions(Execution INTERFACE UFO_PAR_TBB=1)
		list(APPEND foundParallelLibs TBB)
	else()
		message(CHECK_FAIL "not found and disabled")
		list(APPEND missingParallelLibs TBB)
	endif()
else()
	message(STATUS "TBB support disabled by user (UFO_WITH_TBB=OFF)")
	list(APPEND missingParallelLibs TBB)
endif()

# --- STL ---
message(CHECK_START "Checking Standard Template Library (STL) parallel support")
cmake_push_check_state()
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

# --- GCD ---
if(UFO_WITH_GCD)
	message(CHECK_START "Finding Grand Central Dispatch (GCD)")
	if(APPLE)
		message(CHECK_PASS "found and enabled")
		target_compile_definitions(Execution INTERFACE UFO_PAR_GCD=1)
		list(APPEND foundParallelLibs GCD)
	else()
		message(CHECK_FAIL "not available on this platform")
		list(APPEND missingParallelLibs GCD)
	endif()
else()
	message(STATUS "GCD support disabled by user (UFO_WITH_GCD=OFF)")
	list(APPEND missingParallelLibs GCD)
endif()

# --- OpenMP ---
if(UFO_WITH_OPENMP)
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
else()
	message(STATUS "OpenMP support disabled by user (UFO_WITH_OPENMP=OFF)")
	list(APPEND missingParallelLibs OMP)
endif()

list(POP_BACK CMAKE_MESSAGE_INDENT)

if(foundParallelLibs)
	message(CHECK_PASS "Parallel backends enabled: [${foundParallelLibs}]")
	if(missingParallelLibs)
		message(STATUS "Optional backends not found or disabled: [${missingParallelLibs}]")
	endif()
else()
	message(CHECK_FAIL "no parallel processing backends found")
	message(WARNING "Execution module will be built but no parallel processing libraries will be enabled. This may lead to suboptimal performance. To enable parallel processing, install one of the supported libraries (oneTBB or OpenMP) or ensure your compiler has native STL parallel support.")
endif()

# Propagate to parent scope so configure_package_config_file can substitute
# @TBB_FOUND@ and @OpenMP_CXX_FOUND@ in ufo-config.cmake.in
set(TBB_FOUND "${TBB_FOUND}" PARENT_SCOPE)
set(OpenMP_CXX_FOUND "${OpenMP_CXX_FOUND}" PARENT_SCOPE)
# message(WARNING "TBB: ${TBB_FOUND}, OpenMP: ${OpenMP_CXX_FOUND}")

if(UFO_BUILD_TESTS)
	add_subdirectory(tests)
endif()

install(
  TARGETS Execution
	EXPORT UFOTargets
  FILE_SET HEADERS
	COMPONENT Execution
)