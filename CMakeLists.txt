cmake_minimum_required( VERSION 2.8.12 )

project( vfMath CXX )

file( GLOB SOURCES_CPP "source/*.cpp" )
include_directories( . 
	"include/vfMath" )

add_library( ${PROJECT_NAME} STATIC ${SOURCES_CPP} )
target_link_libraries( ${PROJECT_NAME} )

set( ${PROJECT_NAME}_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include" CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE )
set( ${PROJECT_NAME}_LIB "${BUILD_OUTPUT_DIRECTORY}/${PROJECT_NAME}.lib" CACHE INTERNAL "${PROJECT_NAME}: Library" FORCE )