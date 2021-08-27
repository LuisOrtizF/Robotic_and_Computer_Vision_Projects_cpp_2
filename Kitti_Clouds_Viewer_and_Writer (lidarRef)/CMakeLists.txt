cmake_minimum_required(VERSION 2.8)

set(execName clouds_viewer_writer)
project(${execName})
add_executable(${execName} "main.cpp")

set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT ${execName} )

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_definitions(-DPCL_NO_PRECOMPILE)

link_directories(${PCL_LIBRARY_DIRS})

target_link_libraries(${execName}
${PCL_LIBRARIES}
)