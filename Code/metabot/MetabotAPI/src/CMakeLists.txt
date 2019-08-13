cmake_minimum_required(VERSION 2.8)
project(MetabotAPI)

# Vendors
add_subdirectory (vendor/serial serial)
set (SERIAL_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/vendor/serial/include/")
include_directories (${SERIAL_HEADERS})

# Defines the METABOT_HEADERS (if you want to use this as a CMAKE
# subdirectory
set (METABOT_HEADERS "${CMAKE_CURRENT_SOURCE_DIR};${SERIAL_HEADERS}" CACHE STRING
    "Metabot header files to include")

# C++11, pedantic warnings
set (CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wno-reorder -pedantic")

# API source
set (SOURCES
    Robot.cpp
    MetabotV2.cpp
    Holobot.cpp
    Packet.cpp
    )

set (FULL_SOURCES)

foreach (SOURCE ${SOURCES})
    set (FULL_SOURCES ${FULL_SOURCES} "${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE}")
endforeach ()

set (METABOT_SOURCES ${FULL_SOURCES} CACHE STRING
    "Metabot source files")

set (METABOT_LIBS serial CACHE STRING
    "Metabot libraries")

# Building Metabot library
add_library (LibMetabot ${METABOT_SOURCES})
target_link_libraries (LibMetabot ${METABOT_LIBS})
