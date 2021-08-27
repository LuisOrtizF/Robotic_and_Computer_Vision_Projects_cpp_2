# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Define project name
project(livecorners)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)

# Add OpenCV headers location to your include paths
include_directories(${OpenCV_INCLUDE_DIRS})

# Declare the executable target built from your sources
aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})

# Link your application with OpenCV libraries
target_link_libraries (${PROJECT_NAME} ${OpenCV_LIBS})
