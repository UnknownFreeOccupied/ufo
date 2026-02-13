project(rply)

add_library(rply STATIC rply/rply.c rply/rply.h)

set_target_properties(rply PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

target_include_directories(rply 
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
		# $<INSTALL_INTERFACE:include>
)