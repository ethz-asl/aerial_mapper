cmake_minimum_required(VERSION 2.8.3)
project(aerial_mapper_utils)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
  src/utils-common.cc
)

#############
# QTCREATOR #
#############
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})

cs_add_executable(${PROJECT_NAME}_test test/test-kd-tree.cc)
add_dependencies(${PROJECT_NAME}_test ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()
