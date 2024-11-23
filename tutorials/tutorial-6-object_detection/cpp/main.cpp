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

/*********************************************************************************
 ** This sample demonstrates how to use the objects detection module            **
 **      with the ZED SDK and display the result                                **
 *********************************************************************************/

// Standard includes
#include <iostream>
#include <fstream>

// ZED includes
#include <sl/Camera.hpp>

// Using std and sl namespaces
using namespace std;
using namespace sl;

int main(int argc, char **argv)
{
    // step1：打开相机
    // Create ZED objects
    Camera zed;
    InitParameters init_parameters;
    init_parameters.depth_mode = DEPTH_MODE::PERFORMANCE;
    init_parameters.coordinate_units = UNIT::METER;
    init_parameters.sdk_verbose = true;

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS)
    {
        cout << "Error " << returned_state << ", exit program.\n";
        return EXIT_FAILURE;
    }

    // step2：启用 3D 物体检测
    // Define the Objects detection module parameters
    ObjectDetectionParameters detection_parameters;
    // run detection for every Camera grab
    // image_sync确定对象检测是否针对每一帧运行或在单独的线程中异步运行。
    detection_parameters.image_sync = true;
    // track detects object accross time and space
    // enable_tracking 允许跨帧跟踪对象并尽可能长时间保持相同的 ID。位置跟踪必须处于活动状态，以便独立于相机运动跟踪对象的运动。
    detection_parameters.enable_tracking = true;
    // compute a binary mask for each object aligned on the left image
    // enable_mask_output 在检测到的对象上输出 2D 蒙版。由于它需要额外的处理，如果不使用，请禁用此选项。
    detection_parameters.enable_segmentation = true; // designed to give person pixel mask

    // If you want to have object tracking you need to enable positional tracking first
    if (detection_parameters.enable_tracking)
        zed.enablePositionalTracking();

    // 第一次使用该模块时，模型将针对硬件进行优化，这可能需要几分钟。模型优化操作仅执行一次。
    cout << "Object Detection: Loading Module..." << endl;
    returned_state = zed.enableObjectDetection(detection_parameters);
    if (returned_state != ERROR_CODE::SUCCESS)
    {
        cout << "Error " << returned_state << ", exit program.\n";
        zed.close();
        return EXIT_FAILURE;
    }

    // step3：检索对象数据
    // detection runtime parameters
    ObjectDetectionRuntimeParameters detection_parameters_rt;
    // detection output
    Objects objects;
    cout << setprecision(3);

    int nb_detection = 0;
    while (nb_detection < 100)
    {

        if (zed.grab() == ERROR_CODE::SUCCESS)
        {
            zed.retrieveObjects(objects, detection_parameters_rt);

            if (objects.is_new)
            {
                cout << objects.object_list.size() << " Object(s) detected\n\n";
                if (!objects.object_list.empty())
                {

                    auto first_object = objects.object_list.front();

                    cout << "First object attributes :\n";
                    cout << " Label '" << first_object.label << "' (conf. "
                         << first_object.confidence << "/100)\n";

                    if (detection_parameters.enable_tracking)
                        cout << " Tracking ID: " << first_object.id << " tracking state: " << first_object.tracking_state << " / " << first_object.action_state << "\n";

                    cout << " 3D position: " << first_object.position << " Velocity: " << first_object.velocity << "\n";

                    cout << " 3D dimensions: " << first_object.dimensions << "\n";

                    if (first_object.mask.isInit())
                        cout << " 2D mask available\n";

                    cout << " Bounding Box 2D \n";
                    for (auto it : first_object.bounding_box_2d)
                        cout << "    " << it << "\n";

                    cout << " Bounding Box 3D \n";
                    for (auto it : first_object.bounding_box)
                        cout << "    " << it << "\n";

                    cout << "\nPress 'Enter' to continue...\n";
                    cin.ignore();
                }
                nb_detection++;
            }
        }
    }

    // step4：禁用模块并退出
    // Disable object detection and close the camera
    zed.disableObjectDetection();
    zed.close(); // zed.close()也可以正确禁用所有活动模块。

    return EXIT_SUCCESS;
}
