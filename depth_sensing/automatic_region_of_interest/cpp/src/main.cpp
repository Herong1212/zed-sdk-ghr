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

/****************************************************************************************************
 ** This sample demonstrates how to apply an exclusion ROI to all ZED SDK measures                 **
 ** This can be very useful to avoid noise from a vehicle bonnet or drone propellers for instance  **
 ***************************************************************************************************/

// Standard includes
#include <stdio.h>
#include <string.h>

// ZED include
#include <sl/Camera.hpp>

// OpenCV include (for display)
#include <opencv2/opencv.hpp>

// Using std and sl namespaces
using namespace std;
using namespace sl;

void print(string msg_prefix, ERROR_CODE err_code, string msg_suffix);
void parseArgs(int argc, char **argv, InitParameters &param);

int main(int argc, char **argv)
{
    // step1.初始化 ZED 相机：：根据命令行参数设置相机参数，打开相机。
    // Create a ZED Camera object
    Camera zed;

    InitParameters init_parameters;
    init_parameters.camera_resolution = RESOLUTION::AUTO; // 相机自动选择分辨率。
    init_parameters.depth_mode = sl::DEPTH_MODE::NEURAL;  // 启用深度感知（基于神经网络计算深度图像），利用 AI 技术将深度感应提升到更高的精度，即使在最具挑战性的情况下也能保持准确和流畅。
    parseArgs(argc, argv, init_parameters);               // 解析命令行参数（如分辨率、输入源），并设置到 init_parameters 中

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS)
    {
        print("Camera Open", returned_state, "Exit program.");
        return EXIT_FAILURE;
    }

    // step2.启用位置追踪：设置追踪模式以支持相机位置数据。
    PositionalTrackingParameters tracking_parameters;
    // tracking_parameters.mode = sl::POSITIONAL_TRACKING_MODE::GEN_2;
    tracking_parameters.mode = sl::POSITIONAL_TRACKING_MODE::STANDARD; // 标准模式，用于大多数应用场景。
    zed.enablePositionalTracking(tracking_parameters);                 // 启用位置追踪功能。

    // step3.配置窗口显示：使用 OpenCV 创建窗口，用于实时显示左眼图像、深度图像以及检测到的 ROI。
    cv::String imWndName = "Image";
    cv::String depthWndName = "Depth";
    cv::String ROIWndName = "ROI";
    cv::namedWindow(imWndName, cv::WINDOW_NORMAL);
    cv::namedWindow(ROIWndName, cv::WINDOW_NORMAL);
    cv::namedWindow(depthWndName, cv::WINDOW_NORMAL);

    std::cout << "Press 'a' to apply the ROI\n"
                 "Press 'r' to reset the ROI\n"
                 "Press 's' to save the ROI as image file to reload it later\n"
                 "Press 'l' to load the ROI from an image file"
              << std::endl;

    // step4. 初始化图像容器
    auto resolution = zed.getCameraInformation().camera_configuration.resolution;

    // Create a Mat to store images
    Mat zed_image(resolution, MAT_TYPE::U8_C4);
    cv::Mat cvImage(resolution.height, resolution.width, CV_8UC4, zed_image.getPtr<sl::uchar1>(MEM::CPU));

    Mat zed_depth_image(resolution, MAT_TYPE::U8_C4);
    cv::Mat cvDepthImage(resolution.height, resolution.width, CV_8UC4, zed_depth_image.getPtr<sl::uchar1>(MEM::CPU));

    std::string mask_name = "Mask.png";
    Mat mask_roi(resolution, MAT_TYPE::U8_C1);
    cv::Mat cvMaskROI(resolution.height, resolution.width, CV_8UC1, mask_roi.getPtr<sl::uchar1>(MEM::CPU));

    // step5.自动检测 ROI：实时检测感兴趣区域并在深度图上标记。
    bool roi_running = false;
    sl::RegionOfInterestParameters roi_param;
    // roi_param.auto_apply_module = {sl::MODULE::ALL};
    roi_param.depth_far_threshold_meters = 2.5;        // ROI 检测的深度范围，最大为 2.5 米。
    roi_param.image_height_ratio_cutoff = 0.5;         // 图像高度的分割比例，用于检测 ROI 的垂直范围。
    zed.startRegionOfInterestAutoDetection(roi_param); // 启动自动 ROI 检测

    // step5.主循环：实时捕获与显示
    // Capture new images until 'q' is pressed
    char key = ' ';
    while ((key != 'q') && (key != 27))
    {
        // Check that a new image is successfully acquired
        returned_state = zed.grab();
        if (returned_state == ERROR_CODE::SUCCESS)
        {
            // Retrieve left image
            zed.retrieveImage(zed_image, VIEW::LEFT);
            zed.retrieveImage(zed_depth_image, VIEW::DEPTH);

            auto status = zed.getRegionOfInterestAutoDetectionStatus();
            if (roi_running)
            {
                std::cout << "Region of interest auto detection is running\r" << std::flush;
                if (status == sl::REGION_OF_INTEREST_AUTO_DETECTION_STATE::READY)
                {
                    std::cout << "Region of interest auto detection is done!   " << std::endl;
                    zed.getRegionOfInterest(mask_roi);
                    cv::imshow(ROIWndName, cvMaskROI);
                }
            }
            roi_running = (status == sl::REGION_OF_INTEREST_AUTO_DETECTION_STATE::RUNNING);

            cv::imshow(imWndName, cvImage);
            cv::imshow(depthWndName, cvDepthImage);
        }

        key = cv::waitKey(15);

        // Apply Current ROI
        if (key == 'r') // 重置 ROI：清空当前 ROI，重新启动自动 ROI 检测
        {
            // Reset ROI
            if (!roi_running)
            {
                Mat emptyROI;
                zed.setRegionOfInterest(emptyROI);
            }
            std::cout << "Resetting Auto ROI detection" << std::endl;
            zed.startRegionOfInterestAutoDetection(roi_param);
        }
        else if (key == 's' && mask_roi.isInit()) // 保存 ROI：将当前 ROI 遮罩保存为 PNG 文件，便于以后加载。
        {
            std::cout << "Saving ROI to " << mask_name << std::endl;
            mask_roi.write(mask_name.c_str());
        }
        else if (key == 'l') // 加载 ROI：从保存的文件中加载 ROI，并设置到相机
        {
            // Load the mask from a previously saved file
            cv::Mat tmp = cv::imread(mask_name, cv::IMREAD_GRAYSCALE);
            if (!tmp.empty())
            {
                Mat slROI(sl::Resolution(tmp.cols, tmp.rows), MAT_TYPE::U8_C1, tmp.data, tmp.step);
                zed.setRegionOfInterest(slROI);
            }
            else
                std::cout << mask_name << " could not be found" << std::endl;
        }
    }

    // step6.退出程序
    zed.close();
    return EXIT_SUCCESS;
}

// TODO 格式化并打印消息
void print(string msg_prefix,   // 作为消息的前缀，表示当前操作的描述信息
           ERROR_CODE err_code, // 表示操作的错误码，用于判断操作是否成功
           string msg_suffix)   // 作为消息的后缀，表示额外的提示信息
{
    cout << "[Sample]";
    if (err_code != ERROR_CODE::SUCCESS)
        cout << "[Error] ";
    else
        cout << " ";
    cout << msg_prefix << " ";
    if (err_code != ERROR_CODE::SUCCESS)
    {
        cout << " | " << toString(err_code) << " : ";
        cout << toVerbose(err_code);
    }
    if (!msg_suffix.empty())
        cout << " " << msg_suffix;
    cout << endl;
}

// TODO 解析命令行参数，并根据参数设置 ZED 相机的初始化参数 (InitParameters &param)。
//  通过分析命令行参数，函数可以设置输入模式（例如，SVO 文件、网络流、分辨率等），并打印相应的消息。
void parseArgs(int argc, char **argv, InitParameters &param)
{
    // case1：检查输入是否为 SVO 文件。
    // ps：如果输入是 SVO 文件，表示离线模式，ZED 相机会直接读取该文件，而不是实时捕获图像。输入示例：“ ./app input.svo ”
    if (argc > 1 && string(argv[1]).find(".svo") != string::npos)
    {
        param.input.setFromSVOFile(argv[1]);
    }

    // ps：如果输入不是 SVO 文件
    else if (argc > 1 && string(argv[1]).find(".svo") == string::npos)
    {
        string arg = string(argv[1]);
        unsigned int a, b, c, d, port;
        if (sscanf(arg.c_str(), "%u.%u.%u.%u:%d", &a, &b, &c, &d, &port) == 5)
        {
            // 检查输入是否为网络流（IP 地址 + 端口），输入示例：“ ./app 192.168.1.10:3000 ”
            // Stream input mode - IP + port
            string ip_adress = to_string(a) + "." + to_string(b) + "." + to_string(c) + "." + to_string(d);
            param.input.setFromStream(String(ip_adress.c_str()), port);
            cout << "[Sample] Using Stream input, IP : " << ip_adress << ", port : " << port << endl;
        }
        else if (sscanf(arg.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4)
        {
            // 检查输入是否为纯 IP 地址，输入示例：“ ./app 192.168.1.10 ”
            // Stream input mode - IP only
            param.input.setFromStream(String(argv[1]));
            cout << "[Sample] Using Stream input, IP : " << argv[1] << endl;
        }
        // 如果输入是分辨率字符串（如 HD1080），设置相机分辨率。，输入示例：“ ./app HD1080 ”
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

    // ps：如果输入参数不符合任何已定义的模式（SVO 文件、网络流、分辨率设置等），将采用默认配置。
    else
    {
        // Default
    }
}