cmake_minimum_required(VERSION 2.8.3)
project(fw_ortho_from_pcl)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package( GDAL )
include_directories( ${GDAL_INCLUDE_DIRS} )

############
# BINARIES #
############
add_definitions(-std=c++11)

cs_add_executable(fw_ortho_from_pcl src/ortho_from_pcl.cc src/fw-digital-elevation-map.cc)
add_dependencies(fw_ortho_from_pcl ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(fw_ortho_from_pcl ${catkin_LIBRARIES} ${GDAL_LIBRARY})

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
