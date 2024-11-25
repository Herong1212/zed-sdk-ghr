///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2024, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

// TODO 实现了 平面检测 和 平面网格的渲染与显示

/*************************************************************************
** This sample shows how to capture a real-time 3D reconstruction      **
** of the scene using the Spatial Mapping API. The resulting mesh      **
** is displayed as a wireframe on top of the left image using OpenGL.  **
** Spatial Mapping can be started and stopped with the Space Bar key   **
*************************************************************************/

// ZED includes
#include <sl/Camera.hpp>

// Sample includes
#include "../include/GLViewer.hpp"

// Using std and sl namespaces
using namespace std;
using namespace sl;

void parseArgs(int argc, char **argv, sl::InitParameters &param);

int main(int argc, char **argv)
{
    // 1. 初始化和相机打开
    Camera zed;
    // Setup configuration parameters for the ZED
    InitParameters init_parameters;
    init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA;
    init_parameters.coordinate_units = UNIT::METER;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // OpenGL coordinates system
    parseArgs(argc, argv, init_parameters);                                   // 解析命令行参数，用于动态调整初始化参数（如加载特定的 ROI 掩码或设置分辨率）。

    // 2. 打开相机
    ERROR_CODE zed_open_state = zed.open(init_parameters);
    if (zed_open_state != ERROR_CODE::SUCCESS)
    {
        print("Camera Open", zed_open_state, "Exit program.");
        return EXIT_FAILURE;
    }

    auto camera_infos = zed.getCameraInformation();
    auto has_imu = camera_infos.sensors_configuration.isSensorAvailable(SENSOR_TYPE::GYROSCOPE);
    // auto has_imu_acclerometer = camera_infos.sensors_configuration.isSensorAvailable(sl::SENSOR_TYPE::ACCELEROMETER); // 检查有没有加速度计

    // 3. 初始化 OpenGL 可视化
    GLViewer viewer;                                                                                                         // 创建一个用于 OpenGL 渲染的 GLViewer 实例
    bool error_viewer = viewer.init(argc, argv, camera_infos.camera_configuration.calibration_parameters.left_cam, has_imu); // 初始化 OpenGL 可视化工具，传入相机校准参数和 IMU 可用性
    if (error_viewer)
    {
        viewer.exit();
        zed.close();
        return EXIT_FAILURE;
    }

    Mat image;   // current left image
    Pose pose;   // positional tracking data
    Plane plane; // detected plane
    Mesh mesh;   // plane mesh

    ERROR_CODE find_plane_status = ERROR_CODE::SUCCESS;

    POSITIONAL_TRACKING_STATE tracking_state = POSITIONAL_TRACKING_STATE::OFF;

    // time stamp of the last mesh request
    chrono::high_resolution_clock::time_point ts_last;

    UserAction user_action;
    user_action.clear();

    // 4. 启用位置跟踪
    // Enable positional tracking before starting spatial mapping
    zed.enablePositionalTracking(); // 位置跟踪：通过相机和 IMU 估计设备在 3D 空间中的位姿

    // 5. 设置运行参数
    RuntimeParameters runtime_parameters; // 设置运行时参数，包括深度测量参考帧（WORLD 表示世界坐标系）
    runtime_parameters.measure3D_reference_frame = REFERENCE_FRAME::WORLD;

    // 6. 配置平面检测参数，如最小面积、倾斜角限制等
    sl::PlaneDetectionParameters plane_parameters; // sl::Plane对象中，其中包括有用的信息，例如 3D 位置、法线、多边形边界和平面类型（垂直/水平）
    // plane_parameters.max_distance_threshold = 0.2f;      也可以自己配置参数
    // plane_parameters.normal_similarity_threshold = 30;

    // 7. 主循环：图像采集与平面检测
    while (viewer.isAvailable())
    {
        if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS)
        {
            // Retrieve image in GPU memory
            zed.retrieveImage(image, VIEW::LEFT, MEM::GPU);

            // Update pose data (used for projection of the mesh over the current image)
            tracking_state = zed.getPosition(pose);

            if (tracking_state == POSITIONAL_TRACKING_STATE::OK)
            {
                // Compute elapse time since the last call of plane detection
                auto duration = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - ts_last).count();

                // case1：用户点击屏幕时（user_action.hit），调用 zed.findPlaneAtHit()，检测点击位置的平面。
                if (user_action.hit)
                {
                    auto image_click = sl::uint2(user_action.hit_coord.x * camera_infos.camera_configuration.resolution.width, user_action.hit_coord.y * camera_infos.camera_configuration.resolution.height);
                    find_plane_status = zed.findPlaneAtHit(image_click, plane, plane_parameters);
                }

                // case2：用户按下空格键时（user_action.press_space），调用 zed.findFloorPlane()，检测地板平面。
                if ((duration > 500) && user_action.press_space)
                {
                    // Update pose data (used for projection of the mesh over the current image)
                    Transform resetTrackingFloorFrame;
                    find_plane_status = zed.findFloorPlane(plane, resetTrackingFloorFrame);

                    if (find_plane_status != ERROR_CODE::SUCCESS)
                        std::cout << "No plane found" << std::endl;

                    ts_last = chrono::high_resolution_clock::now();
                }

                if (find_plane_status == ERROR_CODE::SUCCESS)
                {
                    sl::float3 normal = plane.getNormal();                // 获取法线
                    sl::float4 plane_equation = plane.getPlaneEquation(); // 获取平面方程：ax+by+cz=d
                    sl::float3 center = plane.getCenter();                // 获取平面中心
                    sl::Transform plane_transform = plane.getPose();      // 获取地板平面框架和相机框架之间的变换

                    // 检测成功时，将平面数据转换为网格（plane.extractMesh）
                    mesh = plane.extractMesh();

                    // 更新图像和平面网格的渲染
                    viewer.updateMesh(mesh, plane.type);
                }
            }

            user_action = viewer.updateImageAndState(image, pose.pose_data, tracking_state);
        }
    }

    // 8. 释放资源并退出
    image.free();
    mesh.clear();

    zed.disablePositionalTracking();
    zed.close();
    return EXIT_SUCCESS;
}

void parseArgs(int argc, char **argv, sl::InitParameters &param)
{
    if (argc > 1 && string(argv[1]).find(".svo") != string::npos)
    {
        // SVO input mode
        param.input.setFromSVOFile(argv[1]);
        cout << "[Sample] Using SVO File input: " << argv[1] << endl;
    }
    else if (argc > 1 && string(argv[1]).find(".svo") == string::npos)
    {
        string arg = string(argv[1]);
        unsigned int a, b, c, d, port;
        if (sscanf(arg.c_str(), "%u.%u.%u.%u:%d", &a, &b, &c, &d, &port) == 5)
        {
            // Stream input mode - IP + port
            string ip_adress = to_string(a) + "." + to_string(b) + "." + to_string(c) + "." + to_string(d);
            param.input.setFromStream(sl::String(ip_adress.c_str()), port);
            cout << "[Sample] Using Stream input, IP : " << ip_adress << ", port : " << port << endl;
        }
        else if (sscanf(arg.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4)
        {
            // Stream input mode - IP only
            param.input.setFromStream(sl::String(argv[1]));
            cout << "[Sample] Using Stream input, IP : " << argv[1] << endl;
        }
        else if (arg.find("HD2K") != string::npos)
        {
            param.camera_resolution = RESOLUTION::HD2K;
            cout << "[Sample] Using Camera in resolution HD2K" << endl;
        }
        else if (arg.find("HD1200") != string::npos)
        {
            param.camera_resolution = RESOLUTION::HD1200;
            cout << "[Sample] Using Camera in resolution HD1200" << endl;
        }
        else if (arg.find("HD1080") != string::npos)
        {
            param.camera_resolution = RESOLUTION::HD1080;
            cout << "[Sample] Using Camera in resolution HD1080" << endl;
        }
        else if (arg.find("HD720") != string::npos)
        {
            param.camera_resolution = RESOLUTION::HD720;
            cout << "[Sample] Using Camera in resolution HD720" << endl;
        }
        else if (arg.find("SVGA") != string::npos)
        {
            param.camera_resolution = RESOLUTION::SVGA;
            cout << "[Sample] Using Camera in resolution SVGA" << endl;
        }
        else if (arg.find("VGA") != string::npos)
        {
            param.camera_resolution = RESOLUTION::VGA;
            cout << "[Sample] Using Camera in resolution VGA" << endl;
        }
    }
    else
    {
        // Default
    }
}
