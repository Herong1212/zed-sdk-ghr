cmake_minimum_required(VERSION 3.20)
project(PLANE_DETECTION)

option(BUILD_CPP "Build c++ samples else c#" ON)

if(${BUILD_CPP})
    set(TYPE "cpp")
else()
    set(TYPE "csharp")
endif()

add_subdirectory("floor_plane/${TYPE}")
add_subdirectory("plane_detection/${TYPE}")

