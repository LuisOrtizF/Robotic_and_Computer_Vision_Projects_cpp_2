SET(execName save_zed_poses)
CMAKE_MINIMUM_REQUIRED(VERSION 3.5.1)
PROJECT(${execName})

SET(CMAKE_BUILD_TYPE "RelWithDebInfo")

find_package(ZED 3 REQUIRED)
find_package(OpenCV 3.4.1 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(Eigen3 REQUIRED)

SET(SPECIAL_OS_LIBS "pthread" "X11") #ZED

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

ADD_EXECUTABLE(${execName} main.cpp)

add_definitions(-std=c++14 -O3) #ZED

SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})

TARGET_LINK_LIBRARIES(${execName} ${ZED_LIBS} ${OpenCV_LIBRARIES})