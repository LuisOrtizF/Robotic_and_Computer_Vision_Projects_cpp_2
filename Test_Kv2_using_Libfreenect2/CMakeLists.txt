cmake_minimum_required( VERSION 2.8 FATAL_ERROR )
project( "test_kv2" )

find_package( OpenCV 3.4.1 REQUIRED )
find_package( Threads REQUIRED )
find_package( freenect2 REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${freenect2_INCLUDE_DIRS})
include_directories("/usr/include/libusb-1.0/")

aux_source_directory(. SRC_LIST)
add_executable( ${PROJECT_NAME} ${SRC_LIST} )

target_link_libraries( ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} ${freenect2_LIBRARIES} )