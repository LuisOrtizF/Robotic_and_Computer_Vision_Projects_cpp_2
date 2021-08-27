cmake_minimum_required(VERSION 2.8)

set(execName registration3D)
project(${execName})
add_executable(${execName} "main.cpp")

set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT ${execName} )

#find_package(ZED 2.1 REQUIRED)
#find_package(CUDA 8.0 REQUIRED)
#find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED)

#include_directories(${ZED_INCLUDE_DIRS})
#include_directories(${CUDA_INCLUDE_DIRS})
#include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

#add_definitions(-std=c++0x -g -O3) #ZED
add_definitions(${PCL_DEFINITIONS})
add_definitions(-DPCL_NO_PRECOMPILE)

#link_directories(${ZED_LIBRARY_DIR})
#link_directories(${CUDA_LIBRARY_DIRS})
#link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

target_link_libraries(${execName}
#${ZED_LIBRARIES}
#${CUDA_LIBRARIES} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY}
#${OpenCV_LIBRARIES}
${PCL_LIBRARIES}
)
