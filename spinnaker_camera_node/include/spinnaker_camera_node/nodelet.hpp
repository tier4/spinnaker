/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <spinnaker_camera_driver/camera_settings.hpp>
#include <spinnaker_camera_driver/system_wrapper.hpp>

namespace autoware
{
namespace drivers
{
namespace camera
{
class SpinnakerCameraNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit() override;
  void connectCb(int index, spinnaker::CameraListWrapper & cameras);

private:
  spinnaker::CameraListWrapper & createCameras(spinnaker::SystemWrapper * spinnaker_wrapper);
  void publishImage(
    std::uint32_t camera_index, const std::string & frame_id, const Spinnaker::ImagePtr & image);
  std::unique_ptr<sensor_msgs::Image> convertToImageMsg(
    const Spinnaker::ImagePtr & image, const std::string & frame_id);
  std::string convertToPixelFormatString(Spinnaker::PixelFormatEnums pixel_format);

  using CameraInfoManagerPtr = std::shared_ptr<camera_info_manager::CameraInfoManager>;
  ros::NodeHandle nh_, pnh_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  std::mutex connect_mutex_;

  bool use_camera_timestamp_;
  std::vector<spinnaker::CameraSettings> settings_;
  std::vector<image_transport::CameraPublisher> image_publishers_;
  std::vector<CameraInfoManagerPtr> camera_info_managers_;
  std::unique_ptr<spinnaker::SystemWrapper> spinnaker_wrapper_;
};

}  // namespace camera

}  // namespace drivers

}  // namespace autoware
