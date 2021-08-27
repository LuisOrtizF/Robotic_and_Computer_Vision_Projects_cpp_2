cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project("test_kv1")

find_package(OpenCV 3.4.1 REQUIRED)
find_package(Threads REQUIRED)

include_directories($ENV{OPENNI_INCLUDE})
include_directories("/usr/include/libusb-1.0/")

aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})

target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS} ${OPENNI_LIB})
