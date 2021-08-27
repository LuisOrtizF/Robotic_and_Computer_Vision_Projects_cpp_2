SET(execName record_svo_automatic)
CMAKE_MINIMUM_REQUIRED(VERSION 3.5.1)
PROJECT(${execName})

SET(CMAKE_BUILD_TYPE "RelWithDebInfo")

find_package(ZED 3 REQUIRED) #ZED
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED) #ZED
find_package(PCL REQUIRED)

SET(SPECIAL_OS_LIBS "pthread" "X11") #ZED

include_directories(${ZED_INCLUDE_DIRS}) #ZED
include_directories(${CUDA_INCLUDE_DIRS}) #ZED
include_directories("/usr/local/Aria/include") #ARIA
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIR}) #ZED
link_directories(${CUDA_LIBRARY_DIRS}) #ZED
link_directories("/usr/local/Aria/lib") #ARIA
link_directories(${PCL_LIBRARY_DIRS})

ADD_EXECUTABLE(${execName} main.cpp)
add_definitions(-std=c++14 -O3) #ZED

SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})

TARGET_LINK_LIBRARIES(${execName}
  ${ZED_LIBS} #ZED
  libAria.so #libpthread.so libdl.so librt.so #ARIA
  ${PCL_LIBRARIES}
)