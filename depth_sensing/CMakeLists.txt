cmake_minimum_required(VERSION 3.20)
project(Depth_Sensing)

option(BUILD_CPP "Build c++ samples else c#" ON)

if(${BUILD_CPP})
    set(TYPE "cpp")
else()
    set(TYPE "csharp")
endif()

add_subdirectory("automatic_region_of_interest/${TYPE}")
add_subdirectory("depth_sensing/${TYPE}")
add_subdirectory("export/${TYPE}")
add_subdirectory("fusion/${TYPE}")
add_subdirectory("image_refocus/${TYPE}")
add_subdirectory("manual_region_of_interest/${TYPE}")
add_subdirectory("multi_camera/${TYPE}")
