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

#include <string>
#include <unordered_map>
#include <vector>
#include "phoxi_camera_ros2/phoxi_camera.hpp"


namespace phoxi_camera
{

PhoxiCamera::PhoxiCamera()
{
  phoxi_mat = {
    {PhoxiMat::kColorCameraImage, std::bind(&getColorCameraImage, this)},
    {PhoxiMat::kConfidenceMap, std::bind(&getConfidenceMap, this)},
    {PhoxiMat::kDepthMap, std::bind(&getDepthMap, this)},
    {PhoxiMat::kEventMap, std::bind(&getEventMap, this)},
    {PhoxiMat::kNormalMap, std::bind(&getNormalMap, this)},
    {PhoxiMat::kTexture, std::bind(&getTexture, this)}
  };
}

PhoxiCamera::~PhoxiCamera()
{
  device_->StopAcquisition();
  device_->Disconnect();
}

bool PhoxiCamera::phoxiConfigure()
{
  // Check if the PhoXi Control Software is running
  if (!factory_.isPhoXiControlRunning()) {
    std::cout << "PhoXi Control Software is not running" << std::endl;
    return;
  }

  // Get List of available devices on the network
  std::vector<pho::api::PhoXiDeviceInformation> device_list = factory_.GetDeviceList();
  if (device_list.empty()) {
    std::cout << "PhoXi Factory has found 0 devices" << std::endl;
    return;
  }
  PhoxiCamera::printDeviceInfoList(device_list);

  // Try to connect device opened in PhoXi Control, if any
  device_ = factory_.CreateAndConnectFirstAttached();
  if (device_) {
    std::cout << "You have already PhoXi device opened in PhoXi Control, "
      "the API is connected to device: " <<
    (std::string) device_->HardwareIdentification <<
      std::endl;
    // Try to connect specific hardware ID device
  } else if (!hardware_id_.empty()) {
    std::cout << "API will try to connect to " << hardware_id_ << "..." << std::endl;
    device_ = factory_.CreateAndConnect(hardware_id_, 5000);
    if (device_) {
      std::cout << "Successfully connected to " << hardware_id_ << std::endl;
    } else {
      std::cout << "Can not connect to " << hardware_id_ << std::endl;
    }
  } else {
    std::cout <<
      "You have no PhoXi device opened in PhoXi Control, the API ";
    for (size_t i = 0; i < device_list.size(); i++) {
      std::cout << "API will try to connect to ..." <<
        device_list.at(i).HWIdentification << std::endl;
      // wait 5 second for scanner became ready
      device_ = factory_.CreateAndConnect(
        device_list.at(i).HWIdentification, 5000);
      if (device_) {
        std::cout << "Successfully connected" << std::endl;
        break;
      }
      if (i == device_list.size() - 1) {
        std::cout << "Can not connect to any device" << std::endl;
      }
    }
  }

  // Check if device was created
  if (!device_) {
    std::cout << "Your device was not created!" << std::endl;
    return;
  }

  // Check if device is connected
  if (!device_->isConnected()) {
    std::cout << "Your device is not connected" << std::endl;
    return;
  }

  // Set camera trigger mode
  device_->TriggerMode = trigger_mode_;
  device_->MotionCam->OperationMode = operation_mode_;
  std::cout << "Camera trigger and operation mode have been set" << std::endl;

  // Set camera ROI
  device_->ProcessingSettings->ROI3D.CameraSpace.min.x = roi_[0] * 1000;
  device_->ProcessingSettings->ROI3D.CameraSpace.min.y = roi_[1] * 1000;
  device_->ProcessingSettings->ROI3D.CameraSpace.min.z = roi_[2] * 1000;
  device_->ProcessingSettings->ROI3D.CameraSpace.max.x = roi_[3] * 1000;
  device_->ProcessingSettings->ROI3D.CameraSpace.max.y = roi_[4] * 1000;
  device_->ProcessingSettings->ROI3D.CameraSpace.max.z = roi_[5] * 1000;
  if (!device_->ProcessingSettings.isLastOperationSuccessful()) {
    std::cout << device_->ProcessingSettings.GetLastErrorMessage().c_str() << std::endl;
  } else {
    std::cout << "Camera ROI has been set" << std::endl;
  }

  // Validate camera color feature
  if (!device_->Info().CheckFeature("Color")) {
    used_outputs_.second.at(PhoxiMat::kColorCameraImage) = false;
  }

  // Set output settings
  auto [use_pointcloud, use_mats] = getUsedOutputs();
  pho::api::FrameOutputSettings current_output_settings = device_->OutputSettings;
  pho::api::FrameOutputSettings new_output_settings = current_output_settings;
  new_output_settings.SendPointCloud = use_pointcloud;
  new_output_settings.SendColorCameraImage = use_mats.at(PhoxiMat::kColorCameraImage);
  new_output_settings.SendConfidenceMap = use_mats.at(PhoxiMat::kConfidenceMap);
  new_output_settings.SendDepthMap = use_mats.at(PhoxiMat::kDepthMap);
  new_output_settings.SendEventMap = use_mats.at(PhoxiMat::kEventMap);
  new_output_settings.SendNormalMap = use_mats.at(PhoxiMat::kNormalMap);
  new_output_settings.SendTexture = use_mats.at(PhoxiMat::kTexture);
  device_->OutputSettings = new_output_settings;
  if (!device_->OutputSettings.isLastOperationSuccessful()) {
    std::cout << device_->OutputSettings.GetLastErrorMessage().c_str() << std::endl;
  } else {
    std::cout << "Output settings has been set" << std::endl;
  }

  return true;
}

void PhoxiCamera::phoxiStop()
{
  if (device_->isAcquiring()) {
    device_->StopAcquisition();
    std::cout << "Acquisition has been stopped" << std::endl;
  }
}

void PhoxiCamera::phoxiDisconnect()
{
  if (device_->isConnected()) {
    device_->Disconnect();
    std::cout << "Device has been disconnected" << std::endl;
  }
}

std::unordered_map<std::string, int> PhoxiCamera::getDelaysMS()
{
  return delays_ms_;
}

void PhoxiCamera::updateFrame()
{
  auto t1 = std::chrono::steady_clock::now();
  if (trigger_mode_ == phoxi_camera::kTriggerModes.at("software")) {
    device_->TriggerFrame();          // required in case of software mode
  }
  frame_ = device_->GetFrame();
  auto t2 = std::chrono::steady_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  delays_ms_["get_frame"] = ms;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PhoxiCamera::getPCD()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd(new pcl::PointCloud<pcl::PointXYZ>());
  auto t1 = std::chrono::steady_clock::now();
  frame_->ConvertTo(*pcd);
  auto t2 = std::chrono::steady_clock::now();
  for (auto & point: pcd->points) {
    point.x /= 1000;
    point.y /= 1000;
    point.z /= 1000;
  }
  auto t3 = std::chrono::steady_clock::now();
  auto ms_1 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  auto ms_2 = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count();
  delays_ms_["frame_to_pcl"] = ms_1;
  delays_ms_["pcl_loop"] = ms_2;
  return pcd;
}

cv::Mat PhoxiCamera::getColorCameraImage()
{
  cv::Mat color_camera_image;
  auto t1 = std::chrono::steady_clock::now();
  frame_->ColorCameraImage.ConvertTo(color_camera_image);
  auto t2 = std::chrono::steady_clock::now();
  auto ms_1 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  delays_ms_["color_camera_image"] = ms_1;
  return color_camera_image;
}

cv::Mat PhoxiCamera::getConfidenceMap()
{
  cv::Mat confidence_map;
  auto t1 = std::chrono::steady_clock::now();
  frame_->ConfidenceMap.ConvertTo(confidence_map);
  auto t2 = std::chrono::steady_clock::now();
  auto ms_1 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  delays_ms_["confidence_map"] = ms_1;
  return confidence_map;
}

cv::Mat PhoxiCamera::getDepthMap()
{
  cv::Mat depth_map;
  auto t1 = std::chrono::steady_clock::now();
  frame_->DepthMap.ConvertTo(depth_map);
  depth_map /= 1000.0;
  auto t2 = std::chrono::steady_clock::now();
  auto ms_1 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  delays_ms_["depth_map"] = ms_1;
  return depth_map;
}

cv::Mat PhoxiCamera::getEventMap()
{
  cv::Mat event_map;
  auto t1 = std::chrono::steady_clock::now();
  frame_->EventMap.ConvertTo(event_map);
  auto t2 = std::chrono::steady_clock::now();
  auto ms_1 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  delays_ms_["event_map"] = ms_1;
  return event_map;
}

cv::Mat PhoxiCamera::getNormalMap()
{
  cv::Mat normal_map;
  auto t1 = std::chrono::steady_clock::now();
  frame_->NormalMap.ConvertTo(normal_map);
  auto t2 = std::chrono::steady_clock::now();
  auto ms_1 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  delays_ms_["normal_map"] = ms_1;
  return normal_map;
}

cv::Mat PhoxiCamera::getTexture()
{
  cv::Mat texture;
  auto t1 = std::chrono::steady_clock::now();
  if (device_->Info().CheckFeature("Color")) {
    frame_->TextureRGB.ConvertTo(texture);
  } else {
    frame_->Texture.ConvertTo(texture);
  }
  auto t2 = std::chrono::steady_clock::now();
  auto ms_1 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  delays_ms_["texture"] = ms_1;
  return texture;
}

void PhoxiCamera::phoxiActivate()
{
  device_->ClearBuffer();
  device_->StartAcquisition();
  if (!device_->isAcquiring()) {
    std::cout << "Your device could not start acquisition!" << std::endl;
    return;
  }
}

void PhoxiCamera::printDeviceInfoList(
  const std::vector<pho::api::PhoXiDeviceInformation> & device_list)
{
  for (std::size_t i = 0; i < device_list.size(); ++i) {
    std::cout << "Device: " << i << std::endl;
    printDeviceInfo(device_list[i]);
  }
}

void PhoxiCamera::printDeviceInfo(const pho::api::PhoXiDeviceInformation & device_info)
{
  std::cout << "  Name:                    " << device_info.Name << std::endl;
  std::cout << "  Hardware Identification: " << device_info.HWIdentification << std::endl;
  std::cout << "  Type:                    " << static_cast<std::string>(device_info.Type) <<
    std::endl;
  std::cout << "  Firmware version:        " << device_info.FirmwareVersion << std::endl;
  std::cout << "  Variant:                 " << device_info.Variant << std::endl;
  std::cout << "  IsFileCamera:            " << (device_info.IsFileCamera ? "Yes" : "No") <<
    std::endl;
  std::cout << "  Status:                  " <<
  (device_info.Status.Attached ?
  "Attached to PhoXi Control. " :
  "Not Attached to PhoXi Control. ") <<
  (device_info.Status.Ready ? "Ready to connect" : "Occupied") <<
    std::endl <<
    std::endl;
}

void PhoxiCamera::setParameters(
  std::string trigger_mode, std::string operation_mode,
  std::string hardware_id, std::string save_dir,
  std::vector<double> roi, UsedOutputs used_outputs)
{
  auto trigger_mode_it = phoxi_camera::kTriggerModes.find(trigger_mode);
  auto operation_mode_it = phoxi_camera::kOperationModes.find(operation_mode);
  if (trigger_mode_it == phoxi_camera::kTriggerModes.end()) {
    throw std::runtime_error("Unsupported trigger camera mode.\n");
  } else if (operation_mode_it == phoxi_camera::kOperationModes.end()) {
    throw std::runtime_error("Unsupported operation camera mode.\n");
  } else {
    trigger_mode_ = trigger_mode_it->second;
  }
  operation_mode_ = operation_mode_it->second;
  hardware_id_ = hardware_id;
  if (save_dir.empty()) {
    save_dir_.append(std::getenv("HOME"));
    save_dir_.append("/.PhotoneoPhoXiControl/");
  } else {
    save_dir_ = save_dir;
    if (save_dir_.back() != '/') {
      save_dir_.append("/");
    }
  }
  if (roi.size() != 6) {
    roi_ = std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  } else {
    roi_ = roi;
  }
  used_outputs_ = used_outputs;

  if (operation_mode_ == pho::api::PhoXiOperationMode::Mode2D) {
    used_outputs_.first = false;
    for (auto & used_mat : used_outputs_.second) {
      if (used_mat.first != PhoxiMat::kTexture) {
        used_mat.second = false;
      }
    }
  }
}

UsedOutputs PhoxiCamera::getUsedOutputs()
{
  return used_outputs_;
}

int PhoxiCamera::getTriggerMode()
{
  return trigger_mode_;
}

int PhoxiCamera::getOperationMode()
{
  return operation_mode_;
}

void PhoxiCamera::phoxiSavePcd(const std::string filename)
{
  const std::string filepath = save_dir_ + filename + ".ply";
  device_->SaveLastOutput(filepath);
  std::cout << "File saved to " << filepath << std::endl;
}

void PhoxiCamera::phoxiSaveImg(const std::string filename, cv::Mat & img)
{
  std::string filepath = save_dir_ + filename + ".tiff";
  cv::imwrite(filepath, img);
  std::cout << "File saved to " << filepath << std::endl;
}

}  // namespace phoxi_camera
