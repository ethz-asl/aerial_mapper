cmake_minimum_required(VERSION 2.8.3)
project(aerial_mapper_demos_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


############
# BINARIES #
############
add_definitions(-std=c++11)

#cs_add_executable(${PROJECT_NAME}_forward_rosstream
#    src/main.cc
#    src/fw-online-orthomosaic.cc)

cs_add_executable(${PROJECT_NAME}_forward_file
    src/ortho/main-ortho-forward-homography)

#cs_add_executable(${PROJECT_NAME}_backward_vimap
#    src/node-backprojection.cc
#    src/orthomosaic-backprojection.cc)

#cs_add_executable(${PROJECT_NAME}_backward_file
#    src/main-grid-ortho.cc
#    src/orthomosaic-backprojection.cc)

#add_dependencies(${PROJECT_NAME}_forward_rosstream ${${PROJECT_NAME}_EXPORTED_TARGETS}})
#target_link_libraries(${PROJECT_NAME}_forward_rosstream ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}_forward_file ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_forward_file ${catkin_LIBRARIES})

#add_dependencies(${PROJECT_NAME}_backward_vimap ${${PROJECT_NAME}_EXPORTED_TARGETS}})
#target_link_libraries(${PROJECT_NAME}_backward_vimap ${catkin_LIBRARIES} ${GDAL_LIBRARY})

#add_dependencies(${PROJECT_NAME}_backward_file ${${PROJECT_NAME}_EXPORTED_TARGETS}})
#target_link_libraries(${PROJECT_NAME}_backward_file ${catkin_LIBRARIES} ${GDAL_LIBRARY})

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
