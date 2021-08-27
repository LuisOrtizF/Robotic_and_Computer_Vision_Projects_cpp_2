cmake_minimum_required(VERSION 2.8)

set(execName save_manual)

project(${execName})

find_package(ZED 3 REQUIRED) #ZED
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED) #ZED
find_package(OpenCV 3.4.1 REQUIRED)
find_package(PCL 1.8 REQUIRED)

add_definitions(-Wno-format-extra-args) #OPENCV
SET(SPECIAL_OS_LIBS "pthread" "X11") #OPENCV

include_directories(${ZED_INCLUDE_DIRS}) #ZED
include_directories(${CUDA_INCLUDE_DIRS}) #ZED
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include) #ZED

link_directories(${ZED_LIBRARY_DIR}) #ZED
link_directories(${CUDA_LIBRARY_DIRS}) #ZED
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

FILE(GLOB_RECURSE SRC_FILES "main.cpp" "Save.cpp") #ZED
FILE(GLOB_RECURSE HDR_FILES include/*.hpp) #ZED

add_executable(${execName} ${HDR_FILES} ${SRC_FILES})

add_definitions(-std=c++0x -g -O3) #ZED
add_definitions(${PCL_DEFINITIONS})
add_definitions(-DPCL_NO_PRECOMPILE)

target_link_libraries(${execName}
${ZED_LIBRARIES} #ZED
${SPECIAL_OS_LIBS} #ZED
${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED} #ZED
${OpenCV_LIBRARIES}
${PCL_LIBRARIES}
)