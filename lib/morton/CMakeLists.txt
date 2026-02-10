option(UFO_MORTON_BUILD_DOCS "Generate documentation" ON)
option(UFO_MORTON_BUILD_TESTS "Unit testing" ON)
option(UFO_MORTON_BUILD_COVERAGE "Test Coverage" ON)

option(UFO_MORTON_BMI2 "Enable BMI2 instruction set (if available)" OFF)

include(GNUInstallDirs)

add_library(Morton INTERFACE)
add_library(UFO::Morton ALIAS Morton)

target_link_libraries(Morton INTERFACE UFO::Math)

if(UFOMORTON_BMI2)
	message(CHECK_START "Checking BMI2 instruction set support")

	include(CheckCXXSourceCompiles)
	include(CMakePushCheckState)

	cmake_push_check_state()

	set(TEST_FLAG "-mbmi2")
	set(CMAKE_REQUIRED_FLAGS "${TEST_FLAG} -O0")
	check_cxx_source_compiles("
			#include <immintrin.h>
			int main() {
				auto a = _pdep_u64(0, 0);
				(void)a;
				return 0;
			}
		" HAVE_BMI2)

	if(HAVE_BMI2)
		message(CHECK_PASS "supported and enabled")
		target_compile_definitions(Morton INTERFACE UFO_MORTON_BMI2=1)
		target_compile_options(Morton INTERFACE -mbmi2)
	else()
		message(CHECK_FAIL "not supported and disabled")
	endif()

	cmake_pop_check_state()
endif()


target_include_directories(Morton
	INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(UFO_BUILD_TESTS AND UFO_MORTON_BUILD_TESTS)
	add_subdirectory(tests)
endif()

if(UFO_BUILD_DOCS AND UFO_MORTON_BUILD_DOCS)
	add_subdirectory(docs)
endif()

install(TARGETS Morton 
	EXPORT UFO-targets
	COMPONENT Morton
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
	COMPONENT Morton
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)