# ZED SDK - Camera Control

This sample shows how to capture images with the ZED SDK video module and adjust camera settings. You can find additional information on the Video module in our [Documentation](https://www.stereolabs.com/docs/video/camera-controls/) and [API Reference](https://www.stereolabs.com/docs/api/group__Video__group.html).

<p align="center">
  <img src="https://user-images.githubusercontent.com/32394882/230602616-6b57c351-09c4-4aba-bdec-842afcc3b2ea.gif" />
</p>


## Overview

This repository demonstrates how to change the **ZED camera's video settings** and display the camera's image using OpenCV windows. The sample provides examples of how to change the following settings:

- Brightness
- Contrast --- 对比度，控制图像对比度。
- Hue -- 色调，控制图像颜色。
- Saturation -- 饱和度，控制图像颜色强度。
- Sharpness --- 锐度，控制图像清晰度。
- Gamma --- 伽马，控制伽马校正。
- Gain --- 增益，控制来自摄像机传感器的信号的数字放大。
- Exposure --- 曝光
- AEC_AGC --- 控制增益和曝光是否处于自动模式。
- AEC_AGC_ROI --- 控制自动曝光/增益计算的感兴趣区域。
- WHITEBALANCE_TEMPERATURE --- 控制相机白平衡。
- WHITEBALANCE_AUTO --- 控制相机白平衡自动模式。
- Automatic GAIN_EXP behavior
- White balance --- 白平衡
- LED state --- LED 状态，控制摄像头前置 LED 的状态。设置为 0 表示禁用灯光，设置为 1 表示启用灯光。

