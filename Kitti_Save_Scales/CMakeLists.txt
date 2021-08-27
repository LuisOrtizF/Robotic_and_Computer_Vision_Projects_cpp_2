cmake_minimum_required( VERSION 3.5.1 )
set( execName kitti_scales )
project( ${execName} )

# Set C++11 as default
set( CMAKE_CXX_FLAGS "-std=c++11" )

### Find required libs
find_package( OpenCV 3.4.1 REQUIRED )
find_package( Eigen3 REQUIRED )

include_directories( 
${OpenCV_INCLUDE_DIRS} 
${EIGEN3_INCLUDE_DIR}
)

add_executable(${execName} main.cpp)

target_link_libraries( ${execName} ${OpenCV_LIBS} )
