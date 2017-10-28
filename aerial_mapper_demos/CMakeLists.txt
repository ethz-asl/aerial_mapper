cmake_minimum_required(VERSION 2.8.3)
project(aerial_mapper_demos)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

############
# BINARIES #
############
add_definitions(-std=c++11)

cs_add_library(${PROJECT_NAME}
  src/dsm/main-dsm.cc
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

# ORTHO-MOSAIC: backward/grid incremental
cs_add_executable(${PROJECT_NAME}_ortho_backward_grid_incremental
    src/ortho/main-ortho-backward-grid-incremental.cc)
add_dependencies(${PROJECT_NAME}_ortho_backward_grid_incremental ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_ortho_backward_grid_incremental ${catkin_LIBRARIES})

# ORTHO-MOSAIC: from point cloud
cs_add_executable(${PROJECT_NAME}_ortho_from_pcl
    src/ortho/main-ortho-from-pcl.cc)
add_dependencies(${PROJECT_NAME}_ortho_from_pcl ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_ortho_from_pcl ${catkin_LIBRARIES})

# DSM
cs_add_executable(${PROJECT_NAME}_dsm
    src/dsm/main-dsm.cc)
add_dependencies(${PROJECT_NAME}_dsm ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_dsm ${catkin_LIBRARIES})

# DENSE POINT CLOUD
cs_add_executable(${PROJECT_NAME}_dense_pcl
    src/dense-pcl/main-dense-pcl.cc)
add_dependencies(${PROJECT_NAME}_dense_pcl ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_dense_pcl ${catkin_LIBRARIES})

# PIX4D GEOFILE EXPORTER
cs_add_executable(${PROJECT_NAME}_export_pix4d_geofile
    src/util/main-generate-pix4d-geofile.cc)
add_dependencies(${PROJECT_NAME}_export_pix4d_geofile ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_export_pix4d_geofile ${catkin_LIBRARIES})

## TEST MULTI THREADING
#cs_add_executable(${PROJECT_NAME}_test_multi_threading
#    src/util/main-test-multi-threading.cc)
#add_dependencies(${PROJECT_NAME}_test_multi_threading ${${PROJECT_NAME}_EXPORTED_TARGETS}})
#target_link_libraries(${PROJECT_NAME}_test_multi_threading ${catkin_LIBRARIES})

## TEST COLOR LAYERS
#cs_add_executable(${PROJECT_NAME}_test_color_layers
#    src/util/main-test-color-layers.cc)
#add_dependencies(${PROJECT_NAME}_test_color_layers ${${PROJECT_NAME}_EXPORTED_TARGETS}})
#target_link_libraries(${PROJECT_NAME}_test_color_layers ${catkin_LIBRARIES})


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
