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

/***************************************************************************
 ** This sample demonstrates how to use previous GNSS recorded data and   **
 ** How to fused it with ZED camera                                       **
 **************************************************************************/

#include <iostream>
#include <future>
#include <sl/Camera.hpp>
#include <sl/Fusion.hpp>

// ps：将 GNSS 数据与 ZED 相机的位置数据进行融合，从而实现全局定位
/**
 * @brief Function used for getting GNSS data
 *
 * @return sl::GNSSData
 */
sl::GNSSData getGNSSData();

int main(int argc, char **argv)
{
    //////////////////////////////
    //                          //
    //      Setup camera        //
    //                          //
    //////////////////////////////

    // step1. ZED 相机初始化:
    sl::InitParameters init_params;
    init_params.depth_mode = sl::DEPTH_MODE::ULTRA;               // 使用高精度深度模式
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE; // 使用高精度深度模式
    init_params.coordinate_units = sl::UNIT::METER;               // 测量单位为米
    init_params.camera_resolution = sl::RESOLUTION::AUTO;         // 自动选择分辨率
    init_params.camera_fps = 60;                                  // 相机帧率为 60 FPS

    sl::Camera zed;
    sl::ERROR_CODE camera_open_error = zed.open(init_params);
    if (camera_open_error != sl::ERROR_CODE::SUCCESS)
    {
        std::cerr << "[ZED][ERROR] Can't open ZED camera" << std::endl;
        return EXIT_FAILURE;
    }

    // step2. 启用相机的位置追踪：
    sl::PositionalTrackingParameters ptp;
    ptp.initial_world_transform = sl::Transform::identity();
    ptp.enable_imu_fusion = true;                             // 启用 IMU 融合以获得重力方向
    ptp.set_gravity_as_origin = true;                         // 置重力为原点，便于 GNSS 初始化
    auto positional_init = zed.enablePositionalTracking(ptp); // 启用位置追踪
    if (positional_init != sl::ERROR_CODE::SUCCESS)
    {
        std::cerr << "[ZED][ERROR] Can't start tracking of camera" << std::endl;
        return EXIT_FAILURE;
    }

    /// Enable camera publishing for fusion:
    sl::CommunicationParameters communication_parameters;
    communication_parameters.setForSharedMemory();
    zed.startPublishing(communication_parameters);
    /// Run a first grab for starting sending data:
    while (zed.grab() != sl::ERROR_CODE::SUCCESS)
        ;

    //////////////////////////////
    //                          //
    //      Setup Fusion        //
    //                          //
    //////////////////////////////
    // step3. 初始化 Fusion SDK：
    // Fusion SDK 用于融合来自多台相机或 GNSS 的数据
    sl::InitFusionParameters init_multi_cam_parameters;
    init_multi_cam_parameters.coordinate_units = sl::UNIT::METER; // 融合的单位为米
    init_multi_cam_parameters.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
    init_multi_cam_parameters.output_performance_metrics = true; // 启用性能指标输出（便于调试）
    init_multi_cam_parameters.verbose = true;                    // 启用详细日志输出

    sl::Fusion fusion;
    sl::FUSION_ERROR_CODE fusion_init_code = fusion.init(init_multi_cam_parameters);
    if (fusion_init_code != sl::FUSION_ERROR_CODE::SUCCESS)
    {
        std::cerr << "[Fusion][ERROR] Failed to initialize fusion, error: " << fusion_init_code << std::endl;
        return EXIT_FAILURE;
    }

    // step4. 订阅相机：通过 Fusion SDK 订阅 ZED 相机
    sl::CameraIdentifier uuid(zed.getCameraInformation().serial_number);         // 使用 ZED 相机的序列号 (serial_number) 标识相机
    fusion.subscribe(uuid, communication_parameters, sl::Transform::identity()); // 调用 fusion.subscribe 将相机数据加入 Fusion SDK

    // Enable positional tracking:
    sl::PositionalTrackingFusionParameters ptfp;
    ptfp.enable_GNSS_fusion = true;
    fusion.enablePositionalTracking(ptfp);

    //////////////////////////////
    //                          //
    //      Grab data           //
    //                          //
    //////////////////////////////

    std::cout << "Start grabbing data ... " << std::endl;
    // Setup future callback:
    auto gnss_async = std::async(std::launch::async, getGNSSData);
    unsigned number_detection = 0;
    while (number_detection < 200)
    {
        // Grab camera:
        if (zed.grab() == sl::ERROR_CODE::SUCCESS)
        {
            sl::Pose zed_pose;
            // You can still use the classical getPosition for your application, just not that the position returned by this method
            // is the position without any GNSS/cameras fusion
            zed.getPosition(zed_pose, sl::REFERENCE_FRAME::WORLD);
        }

        // step6. 数据融合与处理
        //  Get GNSS data:
        if (gnss_async.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
            sl::GNSSData input_gnss = gnss_async.get();
            // Here we set the GNSS timestamp to the current timestamp of zed camera
            // This is because we use synthetic data and not real one.
            input_gnss.ts = zed.getTimestamp(sl::TIME_REFERENCE::IMAGE);
            if (input_gnss.ts != sl::Timestamp(0))
                fusion.ingestGNSSData(input_gnss);
            gnss_async = std::async(std::launch::async, getGNSSData);
        }

        // step7. 提取融合后的位姿与全球位置
        // 融合处理：调用 fusion.process() 处理当前输入的数据（如 ZED 相机的数据和 GNSS 数据），如果返回值是 SUCCESS，表示数据处理成功。
        //  Process fusion
        if (fusion.process() == sl::FUSION_ERROR_CODE::SUCCESS)
        {
            /// 1、获取全球位置信息（GeoPose）：
            sl::GeoPose current_geopose; // TODO sl::GeoPose：一个结构体，表示全球位置信息，包括经度、纬度、高度和方向（姿态）
            // sl::GNSS_FUSION_STATUS current_geopose_status = fusion.getGeoPose(current_geopose);
            sl::GNSS_CALIBRATION_STATE current_geopose_status = fusion.getGeoPose(current_geopose); // 调用 fusion.getGeoPose 获取当前相机的全球位置（经纬度、高度）

            /// 2、获取融合后的本地位姿（Pose）：
            sl::Pose fused_position; // TODO sl::Pose：一个结构体，表示本地位置信息，包括位置（平移）和姿态（旋转）。
            sl::POSITIONAL_TRACKING_STATE current_state = fusion.getPosition(fused_position);

            // 打印本地位姿（平移、旋转）
            std::string translation_message = std::to_string(fused_position.pose_data.getTranslation().tx) + ", " + std::to_string(fused_position.pose_data.getTranslation().ty) + ", " + std::to_string(fused_position.pose_data.getTranslation().tz);
            std::string rotation_message = std::to_string(fused_position.pose_data.getEulerAngles()[0]) + ", " + std::to_string(fused_position.pose_data.getEulerAngles()[1]) + ", " + std::to_string(fused_position.pose_data.getEulerAngles()[2]);

            // 检查位姿状态
            if (current_state == sl::POSITIONAL_TRACKING_STATE::OK)
            {
                std::cout << "get position translation  = " << translation_message << ", rotation_message = " << rotation_message << std::endl;
            }

            // 输出全球位置（经纬度、高度）
            // if (current_geopose_status == sl::GNSS_FUSION_STATUS::OK)
            if (current_geopose_status == sl::GNSS_CALIBRATION_STATE::CALIBRATED)
            {
                number_detection++;
                double latitude, longitude, altitude;
                current_geopose.latlng_coordinates.getCoordinates(latitude, longitude, altitude, false);
                std::cout << "get world map coordinates latitude = " << latitude << ", longitude = " << longitude << ", altitude = " << altitude << std::endl;
            }
            // 全球位置未初始化的情况
            else
            {
                // GNSS coordinate system to ZED coordinate system is not initialize yet
                // The initialisation between the coordinates system is basically an optimization problem that
                // Try to fit the ZED computed path with the GNSS computed path. In order to do it just move
                // your system by the distance you specified in positional_tracking_fusion_parameters.gnss_initialisation_distance
            }
        }
    }
    fusion.close();
    zed.close();
}

// step5. 模拟获取 GNSS 数据
// GNSS 数据通过异步方式获取并模拟：模拟生成 GNSS 数据：x 值每次递增一个小值，表示模拟的 GNSS 坐标变化。
sl::GNSSData getGNSSData()
{
    static double x = 0;
    x = x + 0.00000001;
    sl::GNSSData out;
    out.setCoordinates(x, 0, 0);

    // N.B. For illustrate how to use Global Localization API we generated "fake" GNSS data.
    // If you use a real GNSS sensor you must provide all GNSSData:
    // coordinates, ts, position_covariance, latitude_std, longitude_std, altitude_std
    return out;
}
