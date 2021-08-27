# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Define project name
project(test_kv2)

find_package(PCL 1.8.0 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Declare the executable target built from your sources
aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})

target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES})
