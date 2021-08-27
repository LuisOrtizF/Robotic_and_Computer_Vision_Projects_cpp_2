# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Define project name
project(zedcorners)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV 3.4.1 REQUIRED)

find_package(PCL 1.8.0 REQUIRED)

find_package(ZED 3 REQUIRED)
find_package(CUDA REQUIRED)

# Add OpenCV headers location to your include paths
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
add_definitions(${CUDA_DEFINITIONS})

include_directories(${ZED_INCLUDE_DIRS})
link_directories(${ZED_LIBRARY_DIR})
add_definitions(${ZED_DEFINITIONS})

add_definitions(-std=c++0x) # -m64)

# Declare the executable target built from your sources
aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})

# Link your application with OpenCV libraries
target_link_libraries (${PROJECT_NAME} ${OpenCV_LIBS})

target_link_libraries (${PROJECT_NAME} ${PCL_LIBRARIES})

target_link_libraries (${PROJECT_NAME} ${ZED_LIBRARIES})

target_link_libraries (${PROJECT_NAME} ${CUDA_LIBRARIES})