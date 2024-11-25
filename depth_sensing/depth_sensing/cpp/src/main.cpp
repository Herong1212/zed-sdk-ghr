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

/*********************************************************************
 ** This sample demonstrates how to capture a live 3D point cloud   **
 ** with the ZED SDK and display the result in an OpenGL window.    **
 *********************************************************************/

// ZED includes
#include <sl/Camera.hpp>

// Sample includes
#include "GLViewer.hpp"

// Using std and sl namespaces
using namespace std;
using namespace sl;

std::string parseArgs(int argc, char **argv, sl::InitParameters &param);

int main(int argc, char **argv)
{
    // step1. 初始化 ZED 相机：设置相机的深度模式、坐标系等参数，
    Camera zed;
    // Set configuration parameters for the ZED
    InitParameters init_parameters;
    init_parameters.depth_mode = DEPTH_MODE::NEURAL;                          // 使用基于神经网络的深度感知模式。
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // 使用右手系（适用于 OpenGL）。
    init_parameters.sdk_verbose = 1;
    auto mask_path = parseArgs(argc, argv, init_parameters); // parseArgs：解析命令行参数，设置相机的额外初始化参数，并返回掩码文件路径（ROI 文件）。

    // step2. Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS)
    {
        // print("Camera Open", returned_state, "Exit program.");
        std::cout << "Camera Open failed!" << std::endl;
        return EXIT_FAILURE;
    }

    // step3. 可选区域屏蔽（ROI）：支持加载区域遮罩文件，用于剔除不相关的区域。
    // Load optional region of interest to exclude irrelevant area of the image
    if (!mask_path.empty())
    {
        sl::Mat mask_roi;
        auto err = mask_roi.read(mask_path.c_str());
        if (err == sl::ERROR_CODE::SUCCESS)
            // zed.setRegionOfInterest(mask_roi, {MODULE::ALL}); 应用到所有模块（如深度计算和点云）。
            zed.setRegionOfInterest(mask_roi);
        else
            std::cout << "Error loading Region of Interest file: " << err << std::endl;
    }

    // step4. 分辨率和 CUDA 流设置：优化点云处理的分辨率，并获取 GPU 流。
    auto camera_config = zed.getCameraInformation().camera_configuration;
    float image_aspect_ratio = camera_config.resolution.width / (1.f * camera_config.resolution.height); // 获取当前相机分辨率（宽度和高度），根据宽高比设置目标分辨率 res（宽度最多 720 像素）
    int requested_low_res_w = min(720, (int)camera_config.resolution.width);
    sl::Resolution res(requested_low_res_w, requested_low_res_w / image_aspect_ratio);

    auto stream = zed.getCUDAStream(); // 获取相机的 GPU 流，用于优化 GPU 数据处理（如点云计算）

    // step5. 初始化点云查看器：通过 OpenGL 渲染 3D 点云。
    GLViewer viewer;

    // 初始化点云查看器
    GLenum errgl = viewer.init(argc, argv, camera_config.calibration_parameters.left_cam, stream, res); // 调用 viewer.init 初始化查看器，传入相机参数（如左眼相机校准参数）和 CUDA 流。
    // viewer.init(argc, argv, camera_config.calibration_parameters.left_cam, stream, res); // 调用 viewer.init 初始化查看器，传入相机参数（如左眼相机校准参数）和 CUDA 流。
    if (errgl != GLEW_OK)
    {
        // print("Error OpenGL: " + std::string((char *)glewGetErrorString(errgl)));
        std::cout << "Error OpenGL:  + " << std::string((char *)glewGetErrorString(errgl)) << std::endl;
        return EXIT_FAILURE;
    }

    RuntimeParameters runParameters;
    // ps：Setting the depth confidence parameters -- 设置置信度参数
    // runParameters.confidence_threshold = 98;             // 删除边缘上的点以避免“链接”对象
    // runParameters.texture_confidence_threshold = 100;    // 从图像的均匀区域中删除点

    // step6. 点云存储和显示：
    // Allocation of 4 channels of float on GPU
    Mat point_cloud(res, MAT_TYPE::F32_C4, sl::MEM::GPU); // 使用 Mat point_cloud 在 GPU 上分配 4 通道的浮点数组（每个点存储 X、Y、Z 和颜色信息）。
    std::cout << "Press on 's' for saving current .ply file" << std::endl;

    // Main Loop
    while (viewer.isAvailable())
    {
        // Check that a new image is successfully acquired
        if (zed.grab(runParameters) == ERROR_CODE::SUCCESS)
        {
            // retrieve the current 3D coloread point cloud in GPU
            zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA, MEM::GPU, res); // 使用 retrieveMeasure 提取当前帧的 3D 点云（存储在 GPU）
            viewer.updatePointCloud(point_cloud);                              // 调用 viewer.updatePointCloud 将点云数据更新到 OpenGL 查看器
            std::cout << "FPS: " << zed.getCurrentFPS() << std::endl;

            // step7. 保存点云数据
            if (viewer.shouldSaveData())
            {
                sl::Mat point_cloud_to_save;
                zed.retrieveMeasure(point_cloud_to_save, MEASURE::XYZRGBA); // 提取点云到 CPU
                auto write_suceed = point_cloud_to_save.write("Pointcloud.ply");
                if (write_suceed == sl::ERROR_CODE::SUCCESS)
                    std::cout << "Current .ply file saving succeed" << std::endl;
                else
                    std::cout << "Current .ply file saving failed" << std::endl;
            }
        }
    }

    // step8. 释放资源
    // free allocated memory before closing the ZED
    point_cloud.free();

    // close the ZED
    zed.close();

    return EXIT_SUCCESS;
}

inline int findImageExtension(int argc, char **argv) // 输入示例：./app image.png
{
    int arg_idx = -1;       // 初始化为 -1，表示默认未找到包含图片扩展名的参数。
    int arg_idx_search = 0; // 初始化为 0，用于确定要开始搜索的参数索引。

    if (argc > 2)
        arg_idx_search = 2;
    else if (argc > 1)
        arg_idx_search = 1;

    if (arg_idx_search > 0 && (string(argv[arg_idx_search]).find(".png") != string::npos ||
                               string(argv[arg_idx_search]).find(".jpg") != string::npos))
        arg_idx = arg_idx_search;
    return arg_idx;
}

// TODO 解析命令行参数，并根据参数设置 ZED 相机的初始化参数 (InitParameters &param)。
//  通过分析命令行参数，函数可以设置输入模式（例如，SVO 文件、网络流、分辨率等），并打印相应的消息。
std::string parseArgs(int argc, char **argv, sl::InitParameters &param)
{
    int mask_arg = findImageExtension(argc, argv);
    std::string mask_path;

    if (argc > 1 && string(argv[1]).find(".svo") != string::npos)
    {
        // SVO 输入模式
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
