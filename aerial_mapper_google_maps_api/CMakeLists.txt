cmake_minimum_required(VERSION 2.8.3)
project(aerial_mapper_google_maps_api)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
add_definitions(--std=c++0x -Wall -Wextra -pedantic)

cs_add_library(${PROJECT_NAME} src/google-maps-api.cc)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
curlpp -lcurlpp -lcurl)

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
