cmake_minimum_required(VERSION 3.20)
project(SamplesAndTutos)

# 是否在编译完成后将生成的可执行文件安装到 bin 目录
option(INSTALL_SAMPLES "Once build, deploy samples into bin" ON)
# 选择构建语言，ON 表示构建 C++ 示例；否则切换为 C#
option(BUILD_CPP "Build c++ samples else c#" ON)

message(STATUS "BUILD_CPP: " ${BUILD_CPP})

if(${BUILD_CPP})
    set(TYPE "cpp")
else()
    set(TYPE "csharp")
endif()

# 优化生成的二进制文件
set(CMAKE_BUILD_TYPE "Release")

set(SAMPLE_LIST "")

# 体追踪相关功能，如多相机体追踪、导出 FBX 文件等
add_subdirectory("body_tracking/body_tracking/${TYPE}")
add_subdirectory("body_tracking/export/fbx_export/${TYPE}")
add_subdirectory("body_tracking/multi_camera/${TYPE}")

# 相机控制功能，包括亮度、饱和、色调等
add_subdirectory("camera_control/${TYPE}")

# 流媒体【接收器】模块，接收来自其他设备（如ZED相机或流媒体服务器）发送的视频流或深度流
add_subdirectory("camera_streaming/receiver/${TYPE}")
# 流媒体【发送器】模块，将 ZED 相机捕获的视频流（以及可能的深度数据）通过网络发送到其他设备或应用
add_subdirectory("camera_streaming/sender/${TYPE}")

# 深度感知，包括多相机深度、兴趣区域处理等
add_subdirectory("depth_sensing/depth_sensing/${TYPE}")
add_subdirectory("depth_sensing/multi_camera/${TYPE}")
add_subdirectory("depth_sensing/image_refocus/${TYPE}")
add_subdirectory("depth_sensing/automatic_region_of_interest/${TYPE}")
add_subdirectory("depth_sensing/manual_region_of_interest/${TYPE}")

# 地理跟踪，提供实时定位、录制回放等
# add_subdirectory("global_localization/live/${TYPE}")
# add_subdirectory("global_localization/playback/${TYPE}")
# add_subdirectory("global_localization/recording/${TYPE}")

# 物体检测，支持鸟瞰图、并发检测等
add_subdirectory("object_detection/image_viewer/${TYPE}")
add_subdirectory("object_detection/birds_eye_viewer/${TYPE}")
add_subdirectory("object_detection/concurrent_detections/cpp")

# 平面检测，包含地面平面检测功能
add_subdirectory("plane_detection/plane_detection/${TYPE}")
add_subdirectory("plane_detection/floor_plane/${TYPE}")

# 位置跟踪功能
add_subdirectory("positional_tracking/positional_tracking/${TYPE}")

# 空间建模与地图生成
add_subdirectory("spatial_mapping/spatial_mapping/${TYPE}")

# 支持传感器和视频（SVO）的录制与导出。
add_subdirectory("recording/export/sensors/${TYPE}")
add_subdirectory("recording/export/svo/${TYPE}")
add_subdirectory("recording/playback/mono/${TYPE}")
add_subdirectory("recording/playback/multi_camera/${TYPE}")
add_subdirectory("recording/recording/multi_camera/${TYPE}")
# add_subdirectory("recording/recording/mono/${TYPE}")

# 示例教程
add_subdirectory("tutorials")

# 如果启用 INSTALL_SAMPLES 且构建的是 C++ 示例，将所有生成的二进制文件（可执行文件）安装到项目根目录下的 bin 文件夹。
if(${INSTALL_SAMPLES} AND ${BUILD_CPP})
    install(TARGETS ${SAMPLE_LIST} RUNTIME DESTINATION ${CMAKE_SOURCE_DIR}/bin)
endif()
