cmake_minimum_required(VERSION 2.8.3)
project(aerial_mapper_demos_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


############
# BINARIES #
############
add_definitions(-std=c++11)


#########
# ORTHO #
#########
#_forward_file
cs_add_executable(${PROJECT_NAME}_ortho_forward_homography
    src/ortho/main-ortho-forward-homography)

#_backward_file
cs_add_executable(${PROJECT_NAME}_ortho_backward_grid
    src/ortho/main-ortho-backward-grid.cc)

# _forward_rosstream
cs_add_executable(${PROJECT_NAME}_ortho_ros_stream_node
    src/ortho/main-ros-stream-node.cc)

# from pcl
cs_add_executable(${PROJECT_NAME}_ortho_from_pcl
    src/ortho/main-ortho-from-pcl.cc)


#######
# DSM #
#######

# from file
cs_add_executable(${PROJECT_NAME}_dsm_from_file
    src/dsm/main-dsm-from-file.cc)


#cs_add_executable(${PROJECT_NAME}_backward_vimap
#    src/node-backprojection.cc
#    src/orthomosaic-backprojection.cc)

#########
# ORTHO #
#########

add_dependencies(${PROJECT_NAME}_ortho_forward_homography ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_ortho_forward_homography ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}_ortho_backward_grid ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_ortho_backward_grid ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}_ortho_ros_stream_node ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_ortho_ros_stream_node ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}_ortho_from_pcl ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_ortho_from_pcl ${catkin_LIBRARIES})

#add_dependencies(${PROJECT_NAME}_backward_vimap ${${PROJECT_NAME}_EXPORTED_TARGETS}})
#target_link_libraries(${PROJECT_NAME}_backward_vimap ${catkin_LIBRARIES} ${GDAL_LIBRARY})

#######
# DSM #
#######
add_dependencies(${PROJECT_NAME}_dsm_from_file ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_dsm_from_file ${catkin_LIBRARIES})


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
