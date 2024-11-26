#include <sl/Camera.hpp>
#include <iostream>

// using namespace sl;

int main(int argc, char const *argv[])
{
	// step1. 创建相机对象
	sl::Camera zed;

	sl::InitParameters init_parameters;
	init_parameters.coordinate_units = sl::UNIT::METER;
	init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
	init_parameters.camera_resolution = sl::RESOLUTION::VGA;
	init_parameters.camera_fps = 30;
	init_parameters.sensors_required = true;

	// step2. 打开相机
	sl::ERROR_CODE returned_state = zed.open(init_parameters);
	if (returned_state != sl::ERROR_CODE::SUCCESS)
	{
		std::cout << "Error: " << sl::toString(returned_state) << ", exit programe!" << std::endl;
		return EXIT_FAILURE;
	}

	// step3. 获取相机信息
	auto camera_info = zed.getCameraInformation();

	// step4. 输出相机及 INU 参数信息
	std::cout << "\n===== Camera Intrinsic Parameters =====" << std::endl;

	std::cout << "Left Camera: " << std::endl;
	auto &left_camera_parameters = camera_info.camera_configuration.calibration_parameters.left_cam;

	std::cout << "Camera1.fx: " << left_camera_parameters.fx << std::endl;
	std::cout << "Camera1.fy: " << left_camera_parameters.fy << std::endl;
	std::cout << "Camera1.cx: " << left_camera_parameters.cx << std::endl;
	std::cout << "Camera1.cy: " << left_camera_parameters.cy << std::endl;

	std::cout << "Right Camera: " << std::endl;
	auto &right_camera_parameters = camera_info.camera_configuration.calibration_parameters.right_cam;

	std::cout << "Camera2.fx: " << right_camera_parameters.fx << std::endl;
	std::cout << "Camera2.fy: " << right_camera_parameters.fy << std::endl;
	std::cout << "Camera2.cx: " << right_camera_parameters.cx << std::endl;
	std::cout << "Camera2.cy: " << right_camera_parameters.cy << std::endl;

	std::cout << "\n===== IMU Parameters =====" << std::endl;
	auto &imu_parameters = camera_info.sensors_configuration;
	std::cout << "IMU.NoiseGyro: " << imu_parameters.gyroscope_parameters.noise_density << std::endl;
	std::cout << "IMU.NoiseAcc: " << imu_parameters.accelerometer_parameters.noise_density << std::endl;
	std::cout << "IMU.GyroWalk: " << imu_parameters.gyroscope_parameters.random_walk << std::endl;
	std::cout << "IMU.AccWalk: " << imu_parameters.accelerometer_parameters.random_walk << std::endl;
	std::cout << "IMU.Frequency: " << imu_parameters.accelerometer_parameters.sampling_rate << std::endl;
	std::cout << imu_parameters.gyroscope_parameters.sampling_rate << std::endl;

	// 打印相机和 IMU 的转换矩阵
	std::cout << "\n===== Camera-IMU Transformation =====" << std::endl;
	auto &imu_transform = camera_info.sensors_configuration.camera_imu_transform;
	// std::cout << "Rotation: " << imu_transform.getRotationMatrix() << std::endl;
	sl::Rotation rotation_matrix = camera_info.sensors_configuration.camera_imu_transform.getRotationMatrix();
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			std::cout << rotation_matrix(i, j) << " ";
		}
		std::cout << std::endl;
	}

	std::cout << "Translation: " << std::endl;
	std::cout << "x: " << imu_transform.getTranslation().tx << std::endl;
	std::cout << "y: " << imu_transform.getTranslation().ty << std::endl;
	std::cout << "z: " << imu_transform.getTranslation().tz << std::endl;

	// 打印其他信息
	std::cout << "\n===== other information ====" << std::endl;
	std::cout << "Resolution: " << camera_info.camera_configuration.resolution.width << " x "
			  << camera_info.camera_configuration.resolution.height << std::endl;

	std::cout << "Camera.fps: " << camera_info.camera_configuration.fps << std::endl;
	std::cout << "Camera serial " << camera_info.serial_number << std::endl;
	std::cout << "Camera input type: " << camera_info.input_type << std::endl;
	std::cout << "Camera model: " << camera_info.camera_model << std::endl;
	std::cout << "Camera baseline: " << camera_info.camera_configuration.calibration_parameters.getCameraBaseline() << std::endl;

	zed.close();
	return EXIT_SUCCESS;
}
