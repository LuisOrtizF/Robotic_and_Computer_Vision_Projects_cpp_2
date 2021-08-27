set( execName panorama )
cmake_minimum_required( VERSION 3.5.1 )
project(${execName})

add_definitions(-std=c++11 -g -O3)

find_package(OpenCV 3.4.1 REQUIRED)

include_directories( 
${OpenCV_INCLUDE_DIRS}
)

add_executable( ${execName} main.cpp )

target_link_libraries( ${execName} ${OpenCV_LIBS} )
