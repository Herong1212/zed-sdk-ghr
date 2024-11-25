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

/*************************************************************************
 ** This sample demonstrates how to use the ZED for positional tracking  **
 ** and display camera motion in an OpenGL window. 		                **
 **************************************************************************/

// ZED includes
#include <sl/Camera.hpp>

// Sample includes
// #include "GLViewer.hpp"
#include "../include/GLViewer.hpp"

// Using std namespace
using namespace std;
using namespace sl;

#define IMU_ONLY 0

inline std::string setTxt(sl::float3 value)
{
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << value;
    return stream.str();
}

std::string parseArgs(int argc, char **argv, sl::InitParameters &param);

int main(int argc, char **argv)
{
    // 1. 初始化相机
    Camera zed;
    // Set configuration parameters for the ZED
    InitParameters init_parameters;
    init_parameters.coordinate_units = UNIT::METER;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    init_parameters.sdk_verbose = true;
    auto mask_path = parseArgs(argc, argv, init_parameters); // 这里输入 mask 图像，遮罩图通常是用户预先生成的，用于指定特定区域。

    // 2. 打开相机并加载遮罩文件
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS)
    {
        // print("Camera Open", returned_state, "Exit program.");
        std::cout << "Camera Open: " << returned_state << "Exit program." << std::endl;
        return EXIT_FAILURE;
    }

    // TODO 加载 mask 图像并设置感兴趣区域（ROI），以排除图像中不相关的区域，考虑应用到视觉 SLAM 中
    // 作用：屏蔽掉图像中无关的部分，提高深度计算或位置追踪的效率。
    // Load optional region of interest to exclude irrelevant area of the image
    if (!mask_path.empty())
    {
        sl::Mat mask_roi;                            // 创建一个掩码对象
        auto err = mask_roi.read(mask_path.c_str()); // 从文件中加载遮罩
        if (err == sl::ERROR_CODE::SUCCESS)
            // zed.setRegionOfInterest(mask_roi, {MODULE::ALL});
            zed.setRegionOfInterest(mask_roi); // ps：设置感兴趣区域，遮罩生效后，只有图像中未被屏蔽的区域会被处理。
        else
            std::cout << "Error loading Region of Interest file: " << err << std::endl;
    }

    auto camera_model = zed.getCameraInformation().camera_model;

    // Create text for GUI
    std::string text_rotation, text_translation;

    // 3. 设置 Positional Tracking 参数
    PositionalTrackingParameters positional_tracking_param;
    positional_tracking_param.enable_imu_fusion = true;                      // 启用 IMU 融合，利用惯性数据提高追踪稳定性。
    positional_tracking_param.mode = sl::POSITIONAL_TRACKING_MODE::STANDARD; // 使用标准模式，适合大多数应用场景。
    // positional_tracking_param.enable_area_memory = true;

    // 启用 Positional Tracking 功能
    returned_state = zed.enablePositionalTracking(positional_tracking_param); // 调用 zed.enablePositionalTracking 激活位置追踪功能。
    if (returned_state != ERROR_CODE::SUCCESS)
    {
        // print("Enabling positional tracking failed: ", returned_state);
        std::cout << "Enabling positional tracking failed: " << returned_state << std::endl;
        zed.close();
        return EXIT_FAILURE;
    }

    // 4. 自动 ROI 检测
    // If there is a part of the image containing a static zone, the tracking accuracy will be significantly impacted
    // The region of interest auto detection is a feature that can be used to remove such zone by masking the irrelevant area of the image.
    // The region of interest can be loaded from a file :

    sl::Mat roi;
    sl::String roi_name = "roi_mask.jpg";
    // roi.read(roi_name);
    // zed.setRegionOfInterest(roi, {sl::MODULE::POSITIONAL_TRACKING});

    // or alternatively auto detected at runtime :
    sl::RegionOfInterestParameters roi_param;

    // 如果没有提供遮罩文件（mask_path.empty()），启用自动 ROI 检测。
    if (mask_path.empty())
    {
        // roi_param.auto_apply_module = {sl::MODULE::DEPTH, sl::MODULE::POSITIONAL_TRACKING};
        zed.startRegionOfInterestAutoDetection(roi_param);
        std::cout << "Region Of Interest auto detection is running." << std::endl;
    }

    Pose camera_path;
    POSITIONAL_TRACKING_STATE tracking_state;

#if IMU_ONLY
    SensorsData sensors_data;
#endif

    REGION_OF_INTEREST_AUTO_DETECTION_STATE roi_state = REGION_OF_INTEREST_AUTO_DETECTION_STATE::NOT_ENABLED;

    GLViewer viewer;

    // 初始化 OpenGL 查看器
    viewer.init(argc, argv, camera_model);

    // 5. 主循环：实时获取位置和可视化
    while (viewer.isAvailable())
    {
        sl::Pose camera_pose;
        if (zed.grab() == ERROR_CODE::SUCCESS)
        {
            // Get the position of the camera in a fixed reference frame (the World Frame)
            tracking_state = zed.getPosition(camera_path, REFERENCE_FRAME::WORLD);

            // sl::PositionalTrackingStatus PositionalTrackingStatus = zed.getPositionalTrackingStatus();
            sl::POSITIONAL_TRACKING_STATE PositionalTrackingState = zed.getPosition(camera_pose, sl::REFERENCE_FRAME::WORLD); // ! PositionalTrackingStatus 已经被弃用了！
            if (zed.getPosition(camera_pose, sl::REFERENCE_FRAME::WORLD) == sl::POSITIONAL_TRACKING_STATE::OK)
            {
                auto translation = camera_pose.getTranslation();
                auto rotation = camera_pose.getOrientation();
                std::cout << "Translation: " << translation << std::endl;
                std::cout << "Rotation: " << rotation << std::endl;
            }

// 这叫条件编译
#if IMU_ONLY
            PositionalTrackingStatus.odometry_status = sl::ODOMETRY_STATUS::OK;
            PositionalTrackingStatus.spatial_memory_status = sl::SPATIAL_MEMORY_STATUS::OK;
            PositionalTrackingStatus.tracking_fusion_status = sl::POSITIONAL_TRACKING_FUSION_STATUS::INERTIAL;
            if (zed.getSensorsData(sensors_data, TIME_REFERENCE::IMAGE) == sl::ERROR_CODE::SUCCESS)
            {
                text_rotation = setTxt(sensors_data.imu.pose.getEulerAngles()); // only rotation is computed for IMU
                viewer.updateData(sensors_data.imu.pose, text_translation, text_rotation, PositionalTrackingStatus);
            }
#else
            if (tracking_state == POSITIONAL_TRACKING_STATE::OK)
            {
                // Get rotation and translation and displays it
                text_rotation = setTxt(camera_path.getEulerAngles());
                text_translation = setTxt(camera_path.getTranslation());
            }
            else
            {
                text_translation = "Tracking lost.";
                text_rotation = "";
            }

            // Update rotation, translation and tracking state values in the OpenGL window
            // viewer.updateData(camera_path.pose_data, text_translation, text_rotation, PositionalTrackingStatus);
            viewer.updateData(camera_path.pose_data, text_translation, text_rotation, tracking_state);
#endif

            // 6. 如果是自动 ROI 检测，那么完成后保存遮罩
            // If the region of interest auto detection is running, the resulting mask can be saved and reloaded for later use
            if (mask_path.empty() && roi_state == sl::REGION_OF_INTEREST_AUTO_DETECTION_STATE::RUNNING && zed.getRegionOfInterestAutoDetectionStatus() == sl::REGION_OF_INTEREST_AUTO_DETECTION_STATE::READY)
            {
                std::cout << "Region Of Interest detection done! Saving into " << roi_name << std::endl;
                // zed.getRegionOfInterest(roi, sl::Resolution(0, 0), sl::MODULE::POSITIONAL_TRACKING);
                zed.getRegionOfInterest(roi, sl::Resolution(0, 0));
                roi.write(roi_name);
            }
            roi_state = zed.getRegionOfInterestAutoDetectionStatus();
        }
        else
            sleep_ms(1);
    }

    // 7. 清理资源
    zed.disablePositionalTracking();

    zed.close();
    return EXIT_SUCCESS;
}

inline int findImageExtension(int argc, char **argv)
{
    int arg_idx = -1;
    int arg_idx_search = 0;
    if (argc > 2)
        arg_idx_search = 2;
    else if (argc > 1)
        arg_idx_search = 1;

    if (arg_idx_search > 0 && (string(argv[arg_idx_search]).find(".png") != string::npos ||
                               string(argv[arg_idx_search]).find(".jpg") != string::npos))
        arg_idx = arg_idx_search;
    return arg_idx;
}

std::string parseArgs(int argc, char **argv, sl::InitParameters &param)
{
    int mask_arg = findImageExtension(argc, argv);
    std::string mask_path;

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

    if (mask_arg > 0)
    {
        mask_path = string(argv[mask_arg]);
        cout << "[Sample] Using Region of Interest from file : " << mask_path << endl;
    }

    return mask_path;
}
