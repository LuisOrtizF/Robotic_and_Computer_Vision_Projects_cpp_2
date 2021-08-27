SET(execName record_svo_manual)
CMAKE_MINIMUM_REQUIRED(VERSION 3.5.1)
PROJECT(${execName})

SET(CMAKE_BUILD_TYPE "RelWithDebInfo")

find_package(ZED 3 REQUIRED) #ZED
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED) #ZED
find_package(OpenCV 3.4.1 REQUIRED) 
find_package(PCL REQUIRED)
find_package(aruco REQUIRED)

SET(SPECIAL_OS_LIBS "pthread" "X11") #ZED

include_directories(${ZED_INCLUDE_DIRS}) #ZED
include_directories(${CUDA_INCLUDE_DIRS}) #ZED
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories("/usr/local/Aria/include") #ARIA

link_directories(${ZED_LIBRARY_DIR}) #ZED
link_directories(${CUDA_LIBRARY_DIRS}) #ZED
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories("/usr/local/Aria/lib") #ARIA

FILE(GLOB_RECURSE SRC_FILES "main.cpp")

ADD_EXECUTABLE(${execName} ${SRC_FILES})
add_definitions(-std=c++14 -O3) #ZED

SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})

TARGET_LINK_LIBRARIES(${execName}
  ${ZED_LIBS} 
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  libAria.so libpthread.so libdl.so librt.so #ARIA
  aruco
)