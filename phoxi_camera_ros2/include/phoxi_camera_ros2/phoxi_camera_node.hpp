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

#ifndef PHOXI_CAMERA_ROS2__PHOXI_CAMERA_NODE_HPP_
#define PHOXI_CAMERA_ROS2__PHOXI_CAMERA_NODE_HPP_

#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include "phoxi_camera_ros2/phoxi_camera.hpp"
#include "phoxi_camera_ros2/lifecycle_interface.hpp"
#include "phoxi_camera_msgs/srv/phoxi_cloud.hpp"
#include "phoxi_camera_msgs/srv/phoxi_full.hpp"
#include "phoxi_camera_msgs/srv/phoxi_img.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/fill_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rmw/types.h"


namespace phoxi_camera
{
using PhoxiCameraPtr = std::unique_ptr<phoxi_camera::PhoxiCamera>;
using ImgMsgPtr = rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr;
using ImgSrvPtr = rclcpp::Service<phoxi_camera_msgs::srv::PhoxiImg>::SharedPtr;

const std::unordered_map<int, std::string> kMatType {
  {CV_8UC1, sensor_msgs::image_encodings::TYPE_8UC1},
  {CV_8UC3, sensor_msgs::image_encodings::TYPE_8UC3},
  {CV_16UC1, sensor_msgs::image_encodings::TYPE_16UC1},
  {CV_16UC3, sensor_msgs::image_encodings::TYPE_16UC3},
  {CV_32FC1, sensor_msgs::image_encodings::TYPE_32FC1},
  {CV_32FC3, sensor_msgs::image_encodings::TYPE_32FC3}
};

const std::unordered_map<PhoxiMat, std::string> kTopicName {
  {PhoxiMat::kColorCameraImage, "color_camera_image"},
  {PhoxiMat::kConfidenceMap, "confidence_map"},
  {PhoxiMat::kDepthMap, "depth_map"},
  {PhoxiMat::kEventMap, "event_map"},
  {PhoxiMat::kNormalMap, "normal_map"},
  {PhoxiMat::kTexture, "texture"},
};

class PHOXI_CAMERA_ROS2_PUBLIC PhoxiCameraNode : public lifecycle_interface::LifecycleInterface
{
public:
  explicit PhoxiCameraNode(const rclcpp::NodeOptions & options);
  bool onConfigure() override;
  void onActivate() override;
  void onDeactivate() override;
  void onError() override;
  void onShutdown() override;
  void onCleanup() override;

private:
  PhoxiCameraPtr phoxi_camera_{nullptr};
  rclcpp::TimerBase::SharedPtr process_timer_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_{};
  std::unordered_map<PhoxiMat, ImgMsgPtr> img_pubs_;
  std::unordered_map<PhoxiMat, ImgSrvPtr> img_srvs_;
  std::unordered_map<PhoxiMat,
    std::function<void(const phoxi_camera_msgs::srv::PhoxiImg::Request::SharedPtr,
    const phoxi_camera_msgs::srv::PhoxiImg::Response::SharedPtr)>> fun_img_srvs_;
  ImgMsgPtr color_camera_image_pub_{};
  ImgMsgPtr confidence_map_pub_{};
  ImgMsgPtr depth_map_pub_{};
  ImgMsgPtr event_map_pub_{};
  ImgMsgPtr normal_map_pub_{};
  ImgMsgPtr texture_pub_{};
  rclcpp::Service<phoxi_camera_msgs::srv::PhoxiCloud>::SharedPtr cloud_srv_;
  rclcpp::Service<phoxi_camera_msgs::srv::PhoxiFull>::SharedPtr full_srv_;
  ImgSrvPtr color_camera_image_srv_;
  ImgSrvPtr confidence_map_srv_;
  ImgSrvPtr depth_map_srv_;
  ImgSrvPtr event_map_srv_;
  ImgSrvPtr normal_map_srv_;
  ImgSrvPtr texture_srv_;
  void processFreeRun();
  void pcdPublish(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcd);
  void imgPublish(std::pair<PhoxiMat, ImgMsgPtr> img_pub);
  void processImgSrv(
    sensor_msgs::msg::Image::SharedPtr & msg, const rclcpp::Time & stamp,
    PhoxiMat phoxi_mat, bool save);
  void cloudService(
    const phoxi_camera_msgs::srv::PhoxiCloud::Request::SharedPtr request,
    phoxi_camera_msgs::srv::PhoxiCloud::Response::SharedPtr response);
  void fullService(
    const phoxi_camera_msgs::srv::PhoxiFull::Request::SharedPtr request,
    phoxi_camera_msgs::srv::PhoxiFull::Response::SharedPtr response);
  void colorCameraImageService(
    const phoxi_camera_msgs::srv::PhoxiImg::Request::SharedPtr request,
    phoxi_camera_msgs::srv::PhoxiImg::Response::SharedPtr response);
  void confidenceMapService(
    const phoxi_camera_msgs::srv::PhoxiImg::Request::SharedPtr request,
    phoxi_camera_msgs::srv::PhoxiImg::Response::SharedPtr response);
  void depthMapService(
    const phoxi_camera_msgs::srv::PhoxiImg::Request::SharedPtr request,
    phoxi_camera_msgs::srv::PhoxiImg::Response::SharedPtr response);
  void eventMapService(
    const phoxi_camera_msgs::srv::PhoxiImg::Request::SharedPtr request,
    phoxi_camera_msgs::srv::PhoxiImg::Response::SharedPtr response);
  void normalMapService(
    const phoxi_camera_msgs::srv::PhoxiImg::Request::SharedPtr request,
    phoxi_camera_msgs::srv::PhoxiImg::Response::SharedPtr response);
  void textureService(
    const phoxi_camera_msgs::srv::PhoxiImg::Request::SharedPtr request,
    phoxi_camera_msgs::srv::PhoxiImg::Response::SharedPtr response);
  std::string tf_prefix_;
  bool debug_;
};
}  // namespace phoxi_camera

#endif  // PHOXI_CAMERA_ROS2__PHOXI_CAMERA_NODE_HPP_
