cmake_minimum_required(VERSION 2.8.3)
project(aerial_mapper_demos_no_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

############
# BINARIES #
############
add_definitions(-std=c++11)

cs_add_library(${PROJECT_NAME}
  src/dsm/main-dsm-from-file.cc
)

# ORTHO-MOSAIC: forward/homography
cs_add_executable(${PROJECT_NAME}_ortho_forward_homography
    src/ortho/main-ortho-forward-homography.cc)
add_dependencies(${PROJECT_NAME}_ortho_forward_homography ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_ortho_forward_homography ${catkin_LIBRARIES})

# ORTHO-MOSAIC: backward/grid
cs_add_executable(${PROJECT_NAME}_ortho_backward_grid
    src/ortho/main-ortho-backward-grid.cc)
add_dependencies(${PROJECT_NAME}_ortho_backward_grid ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_ortho_backward_grid ${catkin_LIBRARIES})

# ORTHO-MOSAIC: from point cloud
cs_add_executable(${PROJECT_NAME}_ortho_from_pcl
    src/ortho/main-ortho-from-pcl.cc)
add_dependencies(${PROJECT_NAME}_ortho_from_pcl ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_ortho_from_pcl ${catkin_LIBRARIES})

# DSM
cs_add_executable(${PROJECT_NAME}_dsm_from_file
    src/dsm/main-dsm-from-file.cc)
add_dependencies(${PROJECT_NAME}_dsm_from_file ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_dsm_from_file ${catkin_LIBRARIES})

# DENSE POINT CLOUD
cs_add_executable(${PROJECT_NAME}_dense_pcl_from_file
    src/dense-pcl/main_from_file.cc)
add_dependencies(${PROJECT_NAME}_dense_pcl_from_file ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_dense_pcl_from_file ${catkin_LIBRARIES})

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
