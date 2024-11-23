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
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // ps：选择 Y 轴向上的右手坐标系，因为它是 3D 查看软件（例如 Meshlab）最常用的系统
    init_parameters.coordinate_units = UNIT::METER;                           // Set units in meters

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS)
    {
        cout << "Error " << returned_state << ", exit program.\n";
        return EXIT_FAILURE;
    }

    // step2：启用位置跟踪
    // Enable positional tracking with default parameters. Positional tracking needs to be enabled before using spatial mapping
    sl::PositionalTrackingParameters tracking_parameters;
    returned_state = zed.enablePositionalTracking(tracking_parameters);
    if (returned_state != ERROR_CODE::SUCCESS)
    {
        cout << "Error " << returned_state << ", exit program.\n";
        return EXIT_FAILURE;
    }

    // step3：启用空间映射
    // Enable spatial mapping
    sl::SpatialMappingParameters mapping_parameters;
    returned_state = zed.enableSpatialMapping(mapping_parameters);
    if (returned_state != ERROR_CODE::SUCCESS)
    {
        cout << "Error " << returned_state << ", exit program.\n";
        return EXIT_FAILURE;
    }

    // step4：实时运行3D重建
    // Grab data during 500 frames
    int i = 0;
    sl::Mesh mesh; // Create a mesh object
    while (i < 500)
    {
        // For each new grab, mesh data is updated
        if (zed.grab() == ERROR_CODE::SUCCESS)
        {
            // In the background, spatial mapping will use newly retrieved images, depth and pose to update the mesh
            sl::SPATIAL_MAPPING_STATE mapping_state = zed.getSpatialMappingState();

            // Print spatial mapping state
            cout << "\rImages captured: " << i << " / 500  ||  Spatial mapping state: " << mapping_state << "\t" << flush;
            i++;
        }
    }

    cout << endl;

    // step5：提取网格
    // Extract, filter and save the mesh in a obj file
    cout << "Extracting Mesh...\n";
    zed.extractWholeSpatialMap(mesh); // Extract the whole mesh

    cout << "Filtering Mesh...\n";
    mesh.filter(sl::MeshFilterParameters::MESH_FILTER::LOW); // Filter the mesh (remove unnecessary vertices and faces)

    cout << "Saving Mesh...\n";
    mesh.save("mesh.obj"); // ps：将该网格保存为 OBJ 文件以供外部使用

    // step6：禁用模块并退出
    // 提取并保存网格后，不要忘记在退出应用程序之前禁用映射和跟踪模块（按此顺序）并关闭相机。
    // Disable tracking and mapping and close the camera
    zed.disableSpatialMapping();
    zed.disablePositionalTracking();
    zed.close();

    return EXIT_SUCCESS;
}
