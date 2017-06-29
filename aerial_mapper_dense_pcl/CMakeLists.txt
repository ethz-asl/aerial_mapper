cmake_minimum_required(VERSION 2.8.3)
project(fw_online_planar_rectification_node)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

############
# BINARIES #
############
add_definitions(-std=c++11)

cs_add_executable(${PROJECT_NAME} src/main.cc src/fw-online-planar-rectification.cc src/Utils.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

cs_add_executable(${PROJECT_NAME}_file src/main_from_file.cc src/fw-online-planar-rectification.cc src/Utils.cpp)
add_dependencies(${PROJECT_NAME}_file ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_file ${catkin_LIBRARIES})

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
