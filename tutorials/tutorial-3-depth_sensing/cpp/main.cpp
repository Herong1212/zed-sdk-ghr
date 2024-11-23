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

    // step1：Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_parameters;
    init_parameters.sdk_verbose = true; // Enable verbose logging
    // ps：注意：默认情况下，深度感知处于ULTRA模式。如果您使用此模式，则无需在 InitParameters 中设置深度模式。
    // init_parameters.depth_mode = DEPTH_MODE::ULTRA;      // Use ULTRA depth mode
    init_parameters.depth_mode = DEPTH_MODE::PERFORMANCE; // Set the depth mode to performance (fastest)
    init_parameters.coordinate_units = UNIT::MILLIMETER;  // Use millimeter units (for depth measurements)

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS)
    {
        cout << "Error " << returned_state << ", exit program." << endl;
        return EXIT_FAILURE;
    }

    // step2：捕获图像和深度
    // Capture 50 images and depth, then stop
    int i = 0;
    sl::Mat image, depth, point_cloud;

    while (i < 50)
    {
        // A new image is available if grab() returns ERROR_CODE::SUCCESS
        if (zed.grab() == ERROR_CODE::SUCCESS)
        {
            zed.retrieveImage(image, VIEW::LEFT);               // Retrieve left image
            zed.retrieveMeasure(depth, MEASURE::DEPTH);         // Retrieve depth map. Depth is aligned on the left image
            zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA); // Retrieve colored point cloud. Point cloud is aligned on the left image.

            // step3：通过点云提取特定像素点的深度值
            // 现在我们已经检索到了点云，我们可以提取特定像素处的深度啦！这里是先获取点云数据，然后再获取特定像素处的深度值！
            // Get and print distance value in mm at the center of the image
            // We measure the distance camera - object using Euclidean distance
            int x = image.getWidth() / 2;
            int y = image.getHeight() / 2;
            sl::float4 point_cloud_value;
            point_cloud.getValue(x, y, &point_cloud_value);

            if (std::isfinite(point_cloud_value.z))
            {
                float distance = sqrt(point_cloud_value.x * point_cloud_value.x + point_cloud_value.y * point_cloud_value.y + point_cloud_value.z * point_cloud_value.z);
                cout << "Distance to Camera at {" << x << ";" << y << "}: " << distance << "mm" << endl;
            }
            else
                cout << "The Distance can not be computed at {" << x << ";" << y << "}" << endl;

            // step3：直接使用深度图来提取特定像素点的深度值
            int x1 = image.getWidth() / 2;
            int y1 = image.getHeight() / 2;

            float depth_value;
            depth.getValue(x1, y1, &depth_value);
            std::cout << "Distance to Camera at {" << x << ";" << y << "}: " << depth_value << "mm" << std::endl;

            std::cout << "-----------------------------------------" << std::endl;

            // Increment the loop
            i++;
        }
    }
    // step4：Close the camera
    zed.close();
    return EXIT_SUCCESS;
}
