// Copyright 2022 Amadeusz Szymko
// Perception for Physical Interaction Laboratory at Poznan University of Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef PHOXI_CAMERA_ROS2__PHOXI_CAMERA_HPP_
#define PHOXI_CAMERA_ROS2__PHOXI_CAMERA_HPP_
#define PHOXI_PCL_SUPPORT
#define PHOXI_OPENCV_SUPPORT
#define PHO_IGNORE_CV_VERSION_RESTRICTION
#define OPENCV_TRAITS_ENABLE_DEPRECATED

#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "phoxi_camera_ros2/visibility_control.hpp"
#include "phoxi_camera_ros2/lifecycle_interface.hpp"
#include "PhoXi.h"


namespace phoxi_camera
{
const std::unordered_map<std::string, pho::api::PhoXiTriggerMode> kTriggerModes {
  {"freerun", pho::api::PhoXiTriggerMode::Freerun},
  {"software", pho::api::PhoXiTriggerMode::Software}
};

const std::unordered_map<std::string, pho::api::PhoXiOperationMode> kOperationModes {
  {"camera", pho::api::PhoXiOperationMode::Camera},
  {"scanner", pho::api::PhoXiOperationMode::Scanner},
  {"mode2d", pho::api::PhoXiOperationMode::Mode2D}
};

enum class PhoxiMat
{
  kColorCameraImage,
  kConfidenceMap,
  kDepthMap,
  kEventMap,
  kNormalMap,
  kTexture
};

using UsedOutputs = std::pair<bool, std::unordered_map<PhoxiMat, bool>>;

class PHOXI_CAMERA_ROS2_PUBLIC PhoxiCamera
{
public:
  PhoxiCamera();
  ~PhoxiCamera();
  void setParameters(
    std::string trigger_mode, std::string operation_mode, std::string hardware_id,
    std::string save_dir, std::vector<double> roi, UsedOutputs used_outputs);
  int getTriggerMode();
  int getOperationMode();
  UsedOutputs getUsedOutputs();  // boolean flags representing the camera functions used
  void phoxiActivate();
  bool phoxiConfigure();
  void phoxiStop();
  void phoxiDisconnect();
  void phoxiSavePcd(const std::string filename);
  void phoxiSaveImg(const std::string filename, cv::Mat & img);
  void updateFrame();

  pcl::PointCloud<pcl::PointXYZ>::Ptr getPCD();
  cv::Mat getConfidenceMap();
  cv::Mat getColorCameraImage();
  cv::Mat getDepthMap();
  cv::Mat getEventMap();
  cv::Mat getNormalMap();
  cv::Mat getTexture();
  cv::Mat getTextureRGB();
  std::unordered_map<std::string, int> getDelaysMS();
  std::unordered_map<PhoxiMat, std::function<cv::Mat()>> phoxi_mat;

private:
  pho::api::PhoXiFactory factory_;
  pho::api::PPhoXi device_;
  pho::api::PFrame frame_;
  pho::api::PhoXiTriggerMode trigger_mode_;
  pho::api::PhoXiOperationMode operation_mode_;
  std::string hardware_id_;
  std::string save_dir_;
  std::vector<double> roi_;
  std::unordered_map<std::string, int> delays_ms_;
  UsedOutputs used_outputs_;
  void printDeviceInfoList(const std::vector<pho::api::PhoXiDeviceInformation> & device_list);
  void printDeviceInfo(const pho::api::PhoXiDeviceInformation & device_info);
};

}  // namespace phoxi_camera

#endif  // PHOXI_CAMERA_ROS2__PHOXI_CAMERA_HPP_
