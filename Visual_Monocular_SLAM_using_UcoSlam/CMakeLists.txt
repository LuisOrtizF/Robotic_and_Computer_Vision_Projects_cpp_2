CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(ucoslam_monocular)
set(CMAKE_CXX_STANDARD 11)

#   Find Dependencies
FIND_PACKAGE(OpenCV 3.4.1 REQUIRED )
FIND_PACKAGE(ucoslam REQUIRED )
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

LINK_LIBRARIES(${OpenCV_LIBS}  ${ucoslam_LIBS} )
#   Add targets
ADD_EXECUTABLE(ucoslam_monocular main.cpp)