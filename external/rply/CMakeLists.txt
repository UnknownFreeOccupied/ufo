project(rply)

add_library(rply STATIC)

target_sources(rply
	PRIVATE
		rply/rply.c
	PUBLIC
		FILE_SET HEADERS
		BASE_DIRS
			rply
		FILES
			rply/rply.h
)

set_target_properties(rply PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

target_include_directories(rply 
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
		# $<INSTALL_INTERFACE:include>
)