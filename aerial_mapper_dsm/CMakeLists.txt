cmake_minimum_required(VERSION 2.8.3)
project(fw_online_digital_elevation_map_node)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package( GDAL )
include_directories( ${GDAL_INCLUDE_DIRS} )

############
# BINARIES #
############
add_definitions(-std=c++11)

cs_add_executable(DEM_from_rosstream src/DEM_from_rosstream_node.cc src/fw-online-digital-elevation-map.cc)
#cs_add_executable(DEM_from_vi_map src/DEM_from_vi_map_node.cc src/fw-online-digital-elevation-map.cc)

add_dependencies(DEM_from_rosstream ${${PROJECT_NAME}_EXPORTED_TARGETS}})
#add_dependencies(DEM_from_vi_map ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(DEM_from_rosstream ${catkin_LIBRARIES} ${GDAL_LIBRARY})
#target_link_libraries(DEM_from_vi_map ${catkin_LIBRARIES})

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
