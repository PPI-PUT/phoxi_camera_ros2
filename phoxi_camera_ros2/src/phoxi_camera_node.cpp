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

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include "phoxi_camera_ros2/phoxi_camera_node.hpp"

namespace phoxi_camera
{
using namespace std::chrono_literals;
auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

PhoxiCameraNode::PhoxiCameraNode(const rclcpp::NodeOptions & options)
: lifecycle_interface::LifecycleInterface("phoxi_camera", options)
{
  phoxi_camera_ = std::make_unique<phoxi_camera::PhoxiCamera>();
  const std::string trigger_mode = this->declare_parameter("trigger_mode", "freerun");
  const std::string operation_mode = this->declare_parameter("operation_mode", "camera");
  const std::string hardware_id = this->declare_parameter("hardware_id", "");
  const std::string save_dir = this->declare_parameter("save_dir", "");
  const bool use_pointcloud = this->declare_parameter("use_pointcloud", true);
  const bool use_color_camera_image = this->declare_parameter("use_color_camera_image", false);
  const bool use_confidence_map = this->declare_parameter("use_confidence_map", false);
  const bool use_depth_map = this->declare_parameter("use_depth_map", false);
  const bool use_event_map = this->declare_parameter("use_event_map", false);
  const bool use_normal_map = this->declare_parameter("use_normal_map", false);
  const bool use_texture = this->declare_parameter("use_texture", false);
  const std::vector<double> roi = this->declare_parameter(
    "roi", std::vector<double>{0.0, 0.0, 0.0,
      0.0, 0.0, 0.0});
  tf_prefix_ = this->declare_parameter("tf_prefix", "phoxi");
  tf_prefix_ = (tf_prefix_.empty()) ? "phoxi" : tf_prefix_;
  debug_ = this->declare_parameter("debug", false);
  auto used_outputs = UsedOutputs(
    use_pointcloud,
    std::unordered_map<PhoxiMat, bool> {
      {PhoxiMat::kColorCameraImage, use_color_camera_image},
      {PhoxiMat::kConfidenceMap, use_confidence_map},
      {PhoxiMat::kDepthMap, use_depth_map},
      {PhoxiMat::kEventMap, use_event_map},
      {PhoxiMat::kNormalMap, use_normal_map},
      {PhoxiMat::kTexture, use_texture},
    }
  );
  img_pubs_ = {
    {PhoxiMat::kColorCameraImage, color_camera_image_pub_},
    {PhoxiMat::kConfidenceMap, confidence_map_pub_},
    {PhoxiMat::kDepthMap, depth_map_pub_},
    {PhoxiMat::kEventMap, event_map_pub_},
    {PhoxiMat::kNormalMap, normal_map_pub_},
    {PhoxiMat::kTexture, texture_pub_},
  };
  img_srvs_ = {
    {PhoxiMat::kColorCameraImage, color_camera_image_srv_},
    {PhoxiMat::kConfidenceMap, confidence_map_srv_},
    {PhoxiMat::kDepthMap, depth_map_srv_},
    {PhoxiMat::kEventMap, event_map_srv_},
    {PhoxiMat::kNormalMap, normal_map_srv_},
    {PhoxiMat::kTexture, texture_srv_},
  };
  fun_img_srvs_ = {
    {PhoxiMat::kColorCameraImage, std::bind(
        &colorCameraImageService, this, std::placeholders::_1,
        std::placeholders::_2)},
    {PhoxiMat::kConfidenceMap, std::bind(
        &confidenceMapService, this, std::placeholders::_1,
        std::placeholders::_2)},
    {PhoxiMat::kDepthMap, std::bind(
        &depthMapService, this, std::placeholders::_1,
        std::placeholders::_2)},
    {PhoxiMat::kEventMap, std::bind(
        &eventMapService, this, std::placeholders::_1,
        std::placeholders::_2)},
    {PhoxiMat::kNormalMap, std::bind(
        &normalMapService, this, std::placeholders::_1,
        std::placeholders::_2)},
    {PhoxiMat::kTexture, std::bind(
        &textureService, this, std::placeholders::_1,
        std::placeholders::_2)},
  };
  phoxi_camera_->setParameters(
    trigger_mode,
    operation_mode,
    hardware_id,
    save_dir,
    roi,
    used_outputs);
  // Uncomment changestate commands if debug without launch file or
  // running driver as composable node - process' containers doesn't handle
  // lifecycle nodes yet because of ros2 core implementation
  // this->configure();
  // this->activate();
}

bool PhoxiCameraNode::onConfigure()
{
  auto success = phoxi_camera_->phoxiConfigure();
  auto [use_pointcloud, use_mats] = phoxi_camera_->getUsedOutputs();
  bool is_any_srv = false;
  switch (phoxi_camera_->getTriggerMode()) {
    case pho::api::PhoXiTriggerMode::Freerun:
      if (use_pointcloud) {
        pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/cloud", custom_qos);
      }
      for (auto & img_pub : img_pubs_) {
        if (use_mats.at(img_pub.first)) {
          img_pub.second =
            this->create_publisher<sensor_msgs::msg::Image>(
            "~/" + kTopicName.at(
              img_pub.first), custom_qos);
        }
      }
      process_timer_ =
        this->create_wall_timer(30ms, std::bind(&PhoxiCameraNode::processFreeRun, this));
      break;
    case pho::api::PhoXiTriggerMode::Software:
      if (use_pointcloud) {
        cloud_srv_ =
          this->create_service<phoxi_camera_msgs::srv::PhoxiCloud>(
          "~/cloud",
          std::bind(
            &PhoxiCameraNode::cloudService, this, std::placeholders::_1,
            std::placeholders::_2));
        is_any_srv = true;
      }
      for (auto & img_srv : img_srvs_) {
        if (use_mats.at(img_srv.first)) {
          img_srv.second = this->create_service<phoxi_camera_msgs::srv::PhoxiImg>(
            "~/" + kTopicName.at(
              img_srv.first), fun_img_srvs_.at(img_srv.first));
          is_any_srv = true;
        }
      }
      if (is_any_srv) {
        full_srv_ =
          this->create_service<phoxi_camera_msgs::srv::PhoxiFull>(
          "~/full",
          std::bind(
            &PhoxiCameraNode::fullService, this, std::placeholders::_1,
            std::placeholders::_2));
      }
      break;
  }
  return success;
}

void PhoxiCameraNode::onActivate()
{
  auto [use_pointcloud, use_mats] = phoxi_camera_->getUsedOutputs();
  switch (phoxi_camera_->getTriggerMode()) {
    case pho::api::PhoXiTriggerMode::Freerun:
      if (use_pointcloud) {
        pcd_pub_->on_activate();
      }
      for (auto & img_pub : img_pubs_) {
        if (use_mats.at(img_pub.first)) {
          img_pub.second->on_activate();
        }
      }
      break;
    case pho::api::PhoXiTriggerMode::Software:
      break;
  }
  phoxi_camera_->phoxiActivate();
}

void PhoxiCameraNode::onDeactivate()
{
  auto [use_pointcloud, use_mats] = phoxi_camera_->getUsedOutputs();
  if (use_pointcloud) {
    pcd_pub_->on_deactivate();
  }
  for (auto & img_pub : img_pubs_) {
    if (use_mats.at(img_pub.first)) {
      img_pub.second->on_deactivate();
    }
  }
  phoxi_camera_->phoxiStop();
}

void PhoxiCameraNode::onError()
{
}

void PhoxiCameraNode::onShutdown()
{
  if (this->isActive()) {
    PhoxiCameraNode::onDeactivate();
  }
  phoxi_camera_->phoxiDisconnect();
}

void PhoxiCameraNode::onCleanup()
{
}

void PhoxiCameraNode::processFreeRun()
{
  if (!this->isActive()) {
    return;
  }
  auto delays_ms = phoxi_camera_->getDelaysMS();
  auto [use_pointcloud, use_mats] = phoxi_camera_->getUsedOutputs();
  phoxi_camera_->updateFrame();

  if (use_pointcloud) {
    auto pcd = phoxi_camera_->getPCD();
    auto t1 = std::chrono::steady_clock::now();
    this->pcdPublish(pcd);
    auto t2 = std::chrono::steady_clock::now();
    auto ms_1 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    delays_ms["publish_pcd"] = ms_1;
  }

  for (auto & img_pub : img_pubs_) {
    if (use_mats.at(img_pub.first)) {
      auto t3 = std::chrono::steady_clock::now();
      imgPublish(img_pub);
      auto t4 = std::chrono::steady_clock::now();
      auto ms_2 = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();
      delays_ms["publish_" + kTopicName.at(img_pub.first)] = ms_2;
    }
  }

  if (debug_) {
    std::stringstream ss;
    ss << "\n";
    for (auto & delay : delays_ms) {
      ss << delay.first.c_str() << ": " << delay.second << " ms.\n";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }
}

void PhoxiCameraNode::pcdPublish(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcd)
{
  auto msg = sensor_msgs::msg::PointCloud2();
  pcl::toROSMsg(*pcd, msg);
  msg.header.frame_id = tf_prefix_ + "_camera_link";
  msg.header.stamp = this->get_clock()->now();
  pcd_pub_->publish(msg);
}

void PhoxiCameraNode::imgPublish(std::pair<PhoxiMat, ImgMsgPtr> img_pub)
{
  auto mat = phoxi_camera_->phoxi_mat.at(img_pub.first)();
  cv_bridge::CvImage msg;
  msg.header.frame_id = tf_prefix_ + "_camera_link";
  msg.header.stamp = this->get_clock()->now();
  msg.encoding = kMatType.at(mat.type());
  msg.image = mat;
  img_pub.second->publish(*msg.toImageMsg());
}

void PhoxiCameraNode::processImgSrv(
  sensor_msgs::msg::Image::SharedPtr & msg,
  const rclcpp::Time & stamp, PhoxiMat phoxi_mat, bool save)
{
  cv_bridge::CvImage img_cv;

  auto img = phoxi_camera_->phoxi_mat.at(phoxi_mat)();
  img_cv.header.frame_id = tf_prefix_ + "_camera_link";
  img_cv.header.stamp = this->get_clock()->now();
  img_cv.encoding = kMatType.at(img.type());
  img_cv.image = img;
  msg = img_cv.toImageMsg();
  if (img.empty()) {
    auto warn = kTopicName.at(phoxi_mat) + " is empty!";
    RCLCPP_WARN(this->get_logger(), warn.c_str());
    return;
  }
  if (save) {
    const std::string filename = kTopicName.at(phoxi_mat) + "_" + std::to_string(stamp.seconds());
    phoxi_camera_->phoxiSaveImg(filename, img);
  }
}

void PhoxiCameraNode::cloudService(
  const phoxi_camera_msgs::srv::PhoxiCloud::Request::SharedPtr request,
  phoxi_camera_msgs::srv::PhoxiCloud::Response::SharedPtr response)
{
  phoxi_camera_->updateFrame();
  auto stamp = this->get_clock()->now();
  auto pcd = phoxi_camera_->getPCD();
  auto pcd_msg = sensor_msgs::msg::PointCloud2();
  pcl::toROSMsg(*pcd, pcd_msg);
  pcd_msg.header.frame_id = tf_prefix_ + "_camera_link";
  pcd_msg.header.stamp = stamp;
  response->cloud = pcd_msg;

  if (request->save) {
    const std::string filename = "cloud_" + std::to_string(stamp.seconds());
    phoxi_camera_->phoxiSavePcd(filename);
  }
}

void PhoxiCameraNode::fullService(
  const phoxi_camera_msgs::srv::PhoxiFull::Request::SharedPtr request,
  phoxi_camera_msgs::srv::PhoxiFull::Response::SharedPtr response)
{
  phoxi_camera_->updateFrame();
  auto stamp = this->get_clock()->now();

  auto pcd = phoxi_camera_->getPCD();
  auto pcd_msg = sensor_msgs::msg::PointCloud2();
  pcl::toROSMsg(*pcd, pcd_msg);
  pcd_msg.header.frame_id = tf_prefix_ + "_camera_link";
  pcd_msg.header.stamp = stamp;
  response->cloud = pcd_msg;

  if (request->save) {
    const std::string filename = std::to_string(stamp.seconds());
    phoxi_camera_->phoxiSavePcd(filename);
  }

  sensor_msgs::msg::Image::SharedPtr color_camera_image_msg;
  processImgSrv(color_camera_image_msg, stamp, PhoxiMat::kColorCameraImage, request->save);
  response->color_camera_image = *color_camera_image_msg;

  sensor_msgs::msg::Image::SharedPtr confidence_map_msg;
  processImgSrv(confidence_map_msg, stamp, PhoxiMat::kConfidenceMap, request->save);
  response->confidence_map = *confidence_map_msg;

  sensor_msgs::msg::Image::SharedPtr depth_map_msg;
  processImgSrv(depth_map_msg, stamp, PhoxiMat::kDepthMap, request->save);
  response->depth_map = *depth_map_msg;

  sensor_msgs::msg::Image::SharedPtr event_map_msg;
  processImgSrv(event_map_msg, stamp, PhoxiMat::kEventMap, request->save);
  response->event_map = *event_map_msg;

  sensor_msgs::msg::Image::SharedPtr normal_map_msg;
  processImgSrv(normal_map_msg, stamp, PhoxiMat::kNormalMap, request->save);
  response->normal_map = *normal_map_msg;

  sensor_msgs::msg::Image::SharedPtr texture_msg;
  processImgSrv(texture_msg, stamp, PhoxiMat::kTexture, request->save);
  response->texture = *texture_msg;
}

void PhoxiCameraNode::colorCameraImageService(
  const phoxi_camera_msgs::srv::PhoxiImg::Request::SharedPtr request,
  phoxi_camera_msgs::srv::PhoxiImg::Response::SharedPtr response)
{
  phoxi_camera_->updateFrame();
  auto stamp = this->get_clock()->now();

  sensor_msgs::msg::Image::SharedPtr img_msg;
  processImgSrv(img_msg, stamp, PhoxiMat::kColorCameraImage, request->save);
  response->image = *img_msg;
}

void PhoxiCameraNode::confidenceMapService(
  const phoxi_camera_msgs::srv::PhoxiImg::Request::SharedPtr request,
  phoxi_camera_msgs::srv::PhoxiImg::Response::SharedPtr response)
{
  phoxi_camera_->updateFrame();
  auto stamp = this->get_clock()->now();

  sensor_msgs::msg::Image::SharedPtr img_msg;
  processImgSrv(img_msg, stamp, PhoxiMat::kConfidenceMap, request->save);
  response->image = *img_msg;
}

void PhoxiCameraNode::depthMapService(
  const phoxi_camera_msgs::srv::PhoxiImg::Request::SharedPtr request,
  phoxi_camera_msgs::srv::PhoxiImg::Response::SharedPtr response)
{
  phoxi_camera_->updateFrame();
  auto stamp = this->get_clock()->now();

  sensor_msgs::msg::Image::SharedPtr img_msg;
  processImgSrv(img_msg, stamp, PhoxiMat::kDepthMap, request->save);
  response->image = *img_msg;
}

void PhoxiCameraNode::eventMapService(
  const phoxi_camera_msgs::srv::PhoxiImg::Request::SharedPtr request,
  phoxi_camera_msgs::srv::PhoxiImg::Response::SharedPtr response)
{
  phoxi_camera_->updateFrame();
  auto stamp = this->get_clock()->now();

  sensor_msgs::msg::Image::SharedPtr img_msg;
  processImgSrv(img_msg, stamp, PhoxiMat::kEventMap, request->save);
  response->image = *img_msg;
}

void PhoxiCameraNode::normalMapService(
  const phoxi_camera_msgs::srv::PhoxiImg::Request::SharedPtr request,
  phoxi_camera_msgs::srv::PhoxiImg::Response::SharedPtr response)
{
  phoxi_camera_->updateFrame();
  auto stamp = this->get_clock()->now();

  sensor_msgs::msg::Image::SharedPtr img_msg;
  processImgSrv(img_msg, stamp, PhoxiMat::kNormalMap, request->save);
  response->image = *img_msg;
}

void PhoxiCameraNode::textureService(
  const phoxi_camera_msgs::srv::PhoxiImg::Request::SharedPtr request,
  phoxi_camera_msgs::srv::PhoxiImg::Response::SharedPtr response)
{
  phoxi_camera_->updateFrame();
  auto stamp = this->get_clock()->now();

  sensor_msgs::msg::Image::SharedPtr img_msg;
  processImgSrv(img_msg, stamp, PhoxiMat::kTexture, request->save);
  response->image = *img_msg;
}

}  // namespace phoxi_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(phoxi_camera::PhoxiCameraNode)
