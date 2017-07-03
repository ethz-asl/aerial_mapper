cmake_minimum_required(VERSION 2.8.3)
project(aerial_mapper_demos_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

############
# BINARIES #
############
add_definitions(-std=c++11)

# ORTHO-MOSAIC
cs_add_executable(${PROJECT_NAME}_ortho_ros_stream_node
    src/ortho/main-ros-stream-node.cc)
add_dependencies(${PROJECT_NAME}_ortho_ros_stream_node ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_ortho_ros_stream_node ${catkin_LIBRARIES})

# DSM
cs_add_executable(${PROJECT_NAME}_dsm_ros_stream_node
    src/dsm/DEM_from_rosstream_node.cc)
add_dependencies(${PROJECT_NAME}_dsm_ros_stream_node ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_dsm_ros_stream_node ${catkin_LIBRARIES})

# DENSE POINT CLOUD
cs_add_executable(${PROJECT_NAME}_dense_pcl_ros_stream_node
    src/dense-pcl/main.cc)
add_dependencies(${PROJECT_NAME}_dense_pcl_ros_stream_node ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_dense_pcl_ros_stream_node ${catkin_LIBRARIES})

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
