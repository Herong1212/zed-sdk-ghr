cmake_minimum_required(VERSION 3.20)
project(Positional_Tracking)

option(BUILD_CPP "Build c++ samples else c#" ON)

if(${BUILD_CPP})
    set(TYPE "cpp")
else()
    set(TYPE "csharp")
endif()

add_subdirectory("export/fbx/${TYPE}")
add_subdirectory("positional_tracking/${TYPE}")
