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

#include "spinnaker_camera_node/nodelet.hpp"

namespace
{
static constexpr const char kDefaultCameraFrame[] = "camera";
static constexpr std::int64_t kDefaultDeviceThroughputLimit = 100000000L;
static constexpr std::uint64_t kNanoSecondsInSecond = 1000000000U;
}  // namespace

namespace autoware
{
namespace drivers
{
namespace camera
{
void SpinnakerCameraNodelet::onInit()
{
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();
  it_.reset(new image_transport::ImageTransport(pnh_));
  spinnaker_wrapper_ = std::make_unique<spinnaker::SystemWrapper>();
  auto & cameras{createCameras(spinnaker_wrapper_.get())};
  const auto number_of_cameras{cameras.get_number_of_cameras()};
  if (number_of_cameras < 1) {
    throw std::runtime_error("No cameras were found. Cannot start node.");
  }

  pnh_.param<bool>("use_camera_timestamp", use_camera_timestamp_, false);

  std::lock_guard<std::mutex> lock(connect_mutex_);
  image_transport::SubscriberStatusCallback itssc;
  ros::SubscriberStatusCallback rssc;

  for (int i = 0; i < number_of_cameras; ++i) {
    itssc = std::bind(&SpinnakerCameraNodelet::connectCb, this, i, std::ref(cameras));
    rssc = std::bind(&SpinnakerCameraNodelet::connectCb, this, i, std::ref(cameras));
    image_publishers_.emplace_back(it_->advertiseCamera(
      settings_.at(i).get_camera_name() + "/image_raw", 1, itssc, itssc, rssc, rssc));
    camera_info_managers_.emplace_back(std::make_shared<camera_info_manager::CameraInfoManager>(
      pnh_, settings_.at(i).get_camera_name(), settings_.at(i).get_camera_info_url()));
  }

  cameras.set_image_callback(std::bind(
    &SpinnakerCameraNodelet::publishImage, this, std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3));
}

void SpinnakerCameraNodelet::connectCb(int index, spinnaker::CameraListWrapper & cameras)
{
  std::lock_guard<std::mutex> lock(connect_mutex_);
  if (image_publishers_.at(index).getNumSubscribers() > 0) {
    cameras.start_capturing(index);
  } else {
    cameras.stop_capturing(index);
  }
}

spinnaker::CameraListWrapper & SpinnakerCameraNodelet::createCameras(
  spinnaker::SystemWrapper * spinnaker_wrapper)
{
  if (!spinnaker_wrapper_) {
    throw std::runtime_error("The Spinnaker Wrapper is not initialized.");
  }
  XmlRpc::XmlRpcValue cameras, camera_setting;
  pnh_.getParam("cameras", cameras);
  if (cameras.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    throw std::invalid_argument("Invalid parameters");
  }
  for (size_t i = 0; i < cameras.size(); ++i) {
    if (cameras[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
      NODELET_WARN("Camera name should be string.");
      continue;
    }
    const auto camera_name = static_cast<std::string>(cameras[i]);
    pnh_.getParam("camera_settings/" + camera_name, camera_setting);
    if (camera_setting.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      NODELET_WARN("Cannot find camera setting: %s\n", camera_name.c_str());
      continue;
    }
    int window_width = 0;
    if (camera_setting["window_width"].getType() == XmlRpc::XmlRpcValue::TypeInt)
      window_width = static_cast<int>(camera_setting["window_width"]);
    int window_height = 0;
    if (camera_setting["window_height"].getType() == XmlRpc::XmlRpcValue::TypeInt)
      window_height = static_cast<int>(camera_setting["window_height"]);
    double fps = 0.0;
    if (camera_setting["fps"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      fps = static_cast<double>(camera_setting["fps"]);
    std::string pixel_format = "";
    if (camera_setting["pixel_format"].getType() == XmlRpc::XmlRpcValue::TypeString)
      pixel_format = static_cast<std::string>(camera_setting["pixel_format"]);
    std::string frame_id = "";
    if (camera_setting["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString)
      frame_id = static_cast<std::string>(camera_setting["frame_id"]);
    int serial_number = 0;
    if (camera_setting["serial_number"].getType() == XmlRpc::XmlRpcValue::TypeInt)
      serial_number = static_cast<int>(camera_setting["serial_number"]);
    std::string camera_info_url = "";
    if (camera_setting["camera_info_url"].getType() == XmlRpc::XmlRpcValue::TypeString)
      camera_info_url = static_cast<std::string>(camera_setting["camera_info_url"]);
    settings_.emplace_back(
      camera_name, window_width, window_height, fps, pixel_format, frame_id, serial_number,
      camera_info_url);
  }
  NODELET_INFO("Configuring %lu cameras.", settings_.size());
  spinnaker_wrapper->create_cameras(settings_);
  return spinnaker_wrapper->get_cameras();
}

void SpinnakerCameraNodelet::publishImage(
  std::uint32_t camera_index, const std::string & frame_id, const Spinnaker::ImagePtr & image)
{
  auto image_msg = convertToImageMsg(image, frame_id);
  if (!image_msg) {
    NODELET_WARN_THROTTLE(1.0, "Image is incomplete.");
  } else {
    auto camera_info = camera_info_managers_.at(camera_index)->getCameraInfo();
    camera_info.header = image_msg->header;
    image_publishers_.at(camera_index).publish(*image_msg, camera_info);
  }
}

std::unique_ptr<sensor_msgs::Image> SpinnakerCameraNodelet::convertToImageMsg(
  const Spinnaker::ImagePtr & image, const std::string & frame_id)
{
  if (image->IsIncomplete()) {
    return nullptr;
  }

  const std::string encoding_pattern = convertToPixelFormatString(image->GetPixelFormat());

  std_msgs::Header header;
  if (use_camera_timestamp_) {
    auto acquisition_time = image->GetTimeStamp();
    const auto seconds = acquisition_time / kNanoSecondsInSecond;
    header.stamp.sec = static_cast<std::int32_t>(seconds);
    header.stamp.nsec = static_cast<std::uint32_t>(acquisition_time - seconds);
  } else {
    header.stamp = ros::Time::now();
  }
  header.frame_id = frame_id;

  auto msg{std::make_unique<sensor_msgs::Image>()};
  msg->header = header;
  msg->height = static_cast<std::uint32_t>(image->GetHeight());
  msg->width = static_cast<std::uint32_t>(image->GetWidth());
  msg->encoding = encoding_pattern;
  msg->step = static_cast<std::uint32_t>(image->GetStride());

  const size_t image_size = image->GetImageSize();
  msg->data.resize(static_cast<std::uint32_t>(image_size));
  std::copy_n(static_cast<std::uint8_t *>(image->GetData()), image_size, msg->data.data());
  return msg;
}

std::string SpinnakerCameraNodelet::convertToPixelFormatString(
  Spinnaker::PixelFormatEnums pixel_format)
{
  switch (pixel_format) {
    case Spinnaker::PixelFormatEnums::PixelFormat_BayerRG8:
      return spinnaker::CameraSettings::kPixelFormatStr_RGGB8;
      break;
    case Spinnaker::PixelFormatEnums::PixelFormat_BayerGR8:
      return spinnaker::CameraSettings::kPixelFormatStr_GRBG8;
      break;
    case Spinnaker::PixelFormatEnums::PixelFormat_BayerGB8:
      return spinnaker::CameraSettings::kPixelFormatStr_GBRG8;
      break;
    case Spinnaker::PixelFormatEnums::PixelFormat_BayerBG8:
      return spinnaker::CameraSettings::kPixelFormatStr_BGGR8;
      break;
    case Spinnaker::PixelFormatEnums::PixelFormat_RGB8:
      return spinnaker::CameraSettings::kPixelFormatStr_RGB8;
      break;
    case Spinnaker::PixelFormatEnums::PixelFormat_BGR8:
      return spinnaker::CameraSettings::kPixelFormatStr_BGR8;
      break;
    case Spinnaker::PixelFormatEnums::PixelFormat_Mono8:
      return spinnaker::CameraSettings::kPixelFormatStr_MONO8;
      break;
    default:
      throw std::invalid_argument("Unknown pixel format.");
  }
  return {};
}

}  // namespace camera

}  // namespace drivers

}  // namespace autoware

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware::drivers::camera::SpinnakerCameraNodelet, nodelet::Nodelet)
