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

#include <sl/Camera.hpp>

using namespace std;
using namespace sl;

int main(int argc, char **argv)
{
    // step1：打开相机
    // Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_parameters;
    init_parameters.camera_resolution = RESOLUTION::AUTO;                     // Use HD720 or HD1200 video mode (default fps: 60)
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // Use a right-handed Y-up coordinate system
    init_parameters.coordinate_units = UNIT::METER;                           // Set units in meters
    init_parameters.sensors_required = true;

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS)
    {
        cout << "Error " << returned_state << ", exit program.\n";
        return EXIT_FAILURE;
    }

    // step2：启用位置 
    // 一旦打开相机，我们必须启用位置跟踪模块 enablePositionalTracking() 才能获取ZED的位置和方向。
    // Enable positional tracking with default parameters
    PositionalTrackingParameters tracking_parameters;
    returned_state = zed.enablePositionalTracking(tracking_parameters);
    if (returned_state != ERROR_CODE::SUCCESS)
    {
        cout << "Error " << returned_state << ", exit program.\n";
        return EXIT_FAILURE;
    }

    // step3：捕捉位姿
    // Track the camera position during 1000 frames
    int i = 0;
    Pose zed_pose; // Pose 类：包含相机的平移和方向，以及图像时间戳和跟踪置信度。

    // Check if the camera is a ZED M and therefore if an IMU is available
    bool zed_has_imu = zed.getCameraInformation().sensors_configuration.isSensorAvailable(sl::SENSOR_TYPE::GYROSCOPE);
    SensorsData sensor_data;

    while (i < 1000)
    {
        if (zed.grab() == ERROR_CODE::SUCCESS)
        {

            // Get the pose of the left eye of the camera with reference to the world frame
            zed.getPosition(zed_pose, REFERENCE_FRAME::WORLD);

            // get the translation information
            auto zed_translation = zed_pose.getTranslation();
            // get the orientation information
            auto zed_orientation = zed_pose.getOrientation();
            // get the timestamp
            auto ts = zed_pose.timestamp.getNanoseconds();

            // Display the translation and timestamp
            cout << "Camera Translation: {" << zed_translation << "}, Orientation: {" << zed_orientation << "}, timestamp: " << zed_pose.timestamp.getNanoseconds() << "ns\n";

            // step4：捕捉 IMU 数据
            //  Display IMU data
            if (zed_has_imu)
            {
                // Get IMU data at the time the image was captured
                zed.getSensorsData(sensor_data, TIME_REFERENCE::IMAGE);

                // get filtered orientation quaternion
                auto imu_orientation = sensor_data.imu.pose.getOrientation();
                // get raw acceleration
                auto acceleration = sensor_data.imu.linear_acceleration;

                cout << "IMU Orientation: {" << zed_orientation << "}, Acceleration: {" << acceleration << "}\n";
            }
            // 示例输出：
            // Camera Translation: {0.0821431 0.0631538 -0.152049}, Orientation: {0.103573 -0.140253 -0.102196 0.979366}, timestamp: 1732366114087053236ns
            // IMU Orientation: {0.103573 -0.140253 -0.102196 0.979366}, Acceleration: {-1.08144 9.71358 -0.752378}

            i++;
        }
    }

    // step5：关闭相机
    // Disable positional tracking and close the camera
    zed.disablePositionalTracking();
    zed.close();
    return EXIT_SUCCESS;
}
