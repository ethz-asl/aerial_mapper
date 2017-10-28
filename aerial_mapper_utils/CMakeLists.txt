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

##########
# EXPORT #
##########
cs_install()
cs_export()
