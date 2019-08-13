cmake_minimum_required(VERSION 2.8)
project(MetabotAPI)

set (CMAKE_CXX_FLAGS "-std=c++11")

option (PYTHON_BINDING "Enable python binding" OFF)

add_subdirectory (../src/ libmetabot)
include_directories (${METABOT_HEADERS})

# Building program
add_executable (metabot main.cpp)
target_link_libraries (metabot LibMetabot)

