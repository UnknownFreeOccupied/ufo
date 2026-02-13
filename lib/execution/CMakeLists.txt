option(UFO_EXECUTION_BUILD_DOCS "Generate documentation" ON)
option(UFO_EXECUTION_BUILD_TESTS "Unit testing" ON)
option(UFO_EXECUTION_BUILD_COVERAGE "Test Coverage" ON)

include(GNUInstallDirs)

add_library(Execution INTERFACE)
add_library(UFO::Execution ALIAS Execution)

target_link_libraries(Execution INTERFACE UFO::Utility)

# Threading library required in Linux (not in MacOS and Windows, they should just ignore it. Windows might give a warning)
find_package(Threads REQUIRED)
target_link_libraries(Execution INTERFACE Threads::Threads)

include(CheckCXXSourceCompiles)
include(CMakePushCheckState)

message(CHECK_START "Finding parallel processing libraries")
list(APPEND CMAKE_MESSAGE_INDENT "  ")
unset(foundParallelLibs)
unset(missingParallelLibs)

message(CHECK_START "Finding Threading Building Blocks (TBB)")

if(TARGET TBB::tbb)
	# Detect whether TBB::tbb is an ALIAS (built from source via FetchContent,
	# indicated by ALIASED_TARGET being set) or an IMPORTED target (a system
	# library found via find_package).
	get_target_property(_tbb_aliased TBB::tbb ALIASED_TARGET)
	get_target_property(_tbb_req_includes TBB::tbb INTERFACE_INCLUDE_DIRECTORIES)
	unset(_tbb_req_lib)
	if(NOT _tbb_aliased)
		# IMPORTED target: resolve the actual library file path for compile checks.
		# (check_cxx_source_compiles creates a separate project that cannot access
		# targets from the parent build, so we must pass concrete paths.)
		foreach(_cfg "" _RELWITHDEBINFO _RELEASE _DEBUG)
			get_target_property(_tbb_req_lib TBB::tbb IMPORTED_LOCATION${_cfg})
			if(_tbb_req_lib AND NOT _tbb_req_lib MATCHES "NOTFOUND")
				break()
			endif()
		endforeach()
	endif()

	list(APPEND CMAKE_MESSAGE_INDENT "  ")
	message(CHECK_START "Checking if oneTBB and not TBB")

	if(_tbb_aliased)
		# Built from source (e.g. FetchContent): always oneTBB by construction.
		# The library binary does not exist yet at configure time, so a
		# compile+link check is not possible; skip it and assume oneTBB.
		set(HAVE_ONETBB_PAR TRUE CACHE INTERNAL "")
	else()
		cmake_push_check_state()
		set(CMAKE_REQUIRED_FLAGS "-O0")
		if(_tbb_req_includes AND NOT _tbb_req_includes MATCHES "NOTFOUND")
			list(APPEND CMAKE_REQUIRED_INCLUDES "${_tbb_req_includes}")
		endif()
		if(_tbb_req_lib AND NOT _tbb_req_lib MATCHES "NOTFOUND")
			set(CMAKE_REQUIRED_LIBRARIES "${_tbb_req_lib}")
		endif()

		check_cxx_source_compiles("
				// STL
				#include <cstddef>
				#include <vector>

				// oneTBB
				#include <oneapi/tbb.h>

				struct S {
					std::size_t value;
				};

				int main() {
					std::vector<S> v(10);

					oneapi::tbb::parallel_for(std::size_t(0), v.size(), [&v](std::size_t i) { v[i].value = i; });

					return 0;
				}
			" HAVE_ONETBB_PAR)

		cmake_pop_check_state()
	endif()

	if(HAVE_ONETBB_PAR)
		message(CHECK_PASS "has oneTBB")
		list(POP_BACK CMAKE_MESSAGE_INDENT)
		message(CHECK_PASS "found and enabled")
		target_link_libraries(Execution INTERFACE TBB::tbb)
		target_compile_definitions(Execution INTERFACE UFO_PAR_TBB=1)
		list(APPEND foundParallelLibs TBB)
	else()
		message(CHECK_FAIL "only has old TBB (not oneTBB)")
		list(POP_BACK CMAKE_MESSAGE_INDENT)
		message(CHECK_FAIL "not found and disabled")
		list(APPEND missingParallelLibs TBB)
	endif()

else()
	message(CHECK_FAIL "not found and disabled")
	list(APPEND missingParallelLibs TBB)
endif()

message(CHECK_START "Checking Standard Template Library (STL) parallel support")

cmake_push_check_state()

set(CMAKE_REQUIRED_FLAGS "-O0")
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
elseif(TARGET TBB::tbb)
	if(_tbb_req_includes AND NOT _tbb_req_includes MATCHES "NOTFOUND")
		list(APPEND CMAKE_REQUIRED_INCLUDES "${_tbb_req_includes}")
	endif()
	if(_tbb_req_lib AND NOT _tbb_req_lib MATCHES "NOTFOUND")
		set(CMAKE_REQUIRED_LIBRARIES "${_tbb_req_lib}")
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
		" HAVE_STL_PAR_USING_TBB)

	if(HAVE_STL_PAR_USING_TBB)
		message(CHECK_PASS "has support using TBB")
		target_link_libraries(Execution INTERFACE TBB::tbb)
		target_compile_definitions(Execution INTERFACE UFO_PAR_STL=1)
		list(APPEND foundParallelLibs STL)
	else()
		message(CHECK_FAIL "no support")
		list(APPEND missingParallelLibs STL)
	endif()

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