cmake_minimum_required(VERSION 2.8.3)
project(aerial_mapper_ortho)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package( GDAL )
include_directories( ${GDAL_INCLUDE_DIRS} )


#############
# LIBRARIES #
#############
add_definitions(-std=c++11)

if(APPLE)
  add_definitions(-DGTEST_HAS_TR1_TUPLE=0)
endif()
SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS} -lpthread")

cs_add_library(${PROJECT_NAME} 
  src/fw-online-orthomosaic.cc
)

#############
# QTCREATOR #
#############
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})
  
##########
# EXPORT #
##########
cs_install()
cs_export()
