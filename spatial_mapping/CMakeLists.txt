cmake_minimum_required(VERSION 3.20)
project(SPATIAL_MAPPING)

option(BUILD_CPP "Build c++ samples else c#" ON)

if(${BUILD_CPP})
    set(TYPE "cpp")
else()
    set(TYPE "csharp")
endif()

add_subdirectory("multi_camera/${TYPE}")
add_subdirectory("spatial_mapping/${TYPE}")
