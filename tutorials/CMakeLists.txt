cmake_minimum_required(VERSION 3.5)
project(Tutos)

# 定义一个开关选项 BUILD_CPP，用于控制构建语言。ON 是默认值，表示默认构建 C++ 示例代码。
option(BUILD_CPP "Build c++ samples else c#" ON)

if(${BUILD_CPP})
    set(TYPE "cpp")
else()
    set(TYPE "csharp") #  C#
endif()

add_subdirectory("tutorial-1-hello_ZED/${TYPE}")
add_subdirectory("tutorial-2-image_capture/${TYPE}")
add_subdirectory("tutorial-3-depth_sensing/${TYPE}")
add_subdirectory("tutorial-4-positional_tracking/${TYPE}")
add_subdirectory("tutorial-5-spatial_mapping/${TYPE}")
add_subdirectory("tutorial-6-object_detection/${TYPE}")
add_subdirectory("tutorial-7-sensor_data/${TYPE}")
add_subdirectory("tutorial-8-body_tracking/${TYPE}")

# 只有当 BUILD_CPP=ON 时，才会添加 tutorial-9-global_localization 模块到构建系统中。
if(${BUILD_CPP})
    add_subdirectory("tutorial-9-global_localization/${TYPE}")
endif()
