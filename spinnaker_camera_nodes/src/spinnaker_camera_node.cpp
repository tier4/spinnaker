// Copyright 2020 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <spinnaker_camera_nodes/spinnaker_camera_node.hpp>

#include <rcutils/logging_macros.h>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{

/// Create topic name from the camera index.
std::string create_camera_topic_name(std::uint32_t camera_index)
{
  return "/pointgrey/camera_" + std::to_string(camera_index) + "/image_raw";
}

}  // namespace

namespace
{
static constexpr const char kDefaultCameraFrame[] = "camera";
static constexpr const char kCameraSerial[] = "";
static constexpr std::int64_t kDefaultDeviceThroughputLimit = 100000000L;
}  // namespace

namespace autoware
{
namespace drivers
{
namespace camera
{

SpinnakerCameraNode::SpinnakerCameraNode(
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node{"spinnaker_camera_node", node_options}
{
  m_spinnaker_wrapper =
    std::make_unique<spinnaker::SystemWrapper>();
  auto & cameras{create_cameras_from_params(m_spinnaker_wrapper.get())};
  const auto number_of_cameras{cameras.get_number_of_cameras()};
  if (number_of_cameras < 1) {
    // TODO(igor): this should really be a terminate. It is a post-condition violation.
    throw std::runtime_error("No cameras were found. Cannot start node.");
  }
  const std::string one_publisher_per_camera_param{"one_publisher_per_camera"};
  m_use_publisher_per_camera = declare_parameter(one_publisher_per_camera_param, false);
  m_use_camera_timestamp = declare_parameter("use_camera_timestamp", false);
  m_publishers = create_publishers(this, number_of_cameras, m_use_publisher_per_camera);
  if (m_publishers.empty()) {
    // TODO(igor): this should really be a terminate. It is a post-condition violation.
    throw std::runtime_error("No publishers created. Cannot start node.");
  }
  cameras.set_image_callback(
    std::bind(
      &SpinnakerCameraNode::publish_image, this, std::placeholders::_1, std::placeholders::_2));
  cameras.start_capturing();
}

spinnaker::CameraListWrapper & SpinnakerCameraNode::create_cameras_from_params(
  spinnaker::SystemWrapper * spinnaker_wrapper)
{
  if (!spinnaker_wrapper) {
    // TODO(igor): this should really be a terminate. It is a pre-condition violation.
    throw std::runtime_error("The Spinnaker Wrapper is not initialized.");
  }
  const auto settings_from_params =
    [this](const std::string & setting_name, const std::string & camera_name = "") {
      const auto prefix_dot = camera_name.empty() ? setting_name + '.' : setting_name + '.' +
        camera_name + '.';
      // Declare and init optional params.
      const auto camera_frame_id_param{
        declare_parameter(prefix_dot + "frame_id", std::string{kDefaultCameraFrame})};
      const auto camera_serial_number_param{
        declare_parameter(prefix_dot + "serial_number", std::string{kCameraSerial})};
      const auto camera_info_url_param{
        declare_parameter(prefix_dot + "camera_info_url", "")};
      const auto device_link_throughput_limit_param{
        declare_parameter(
          prefix_dot + "device_link_throughput_limit", kDefaultDeviceThroughputLimit)};
      const auto use_external_trigger_param{
        declare_parameter(prefix_dot + "use_external_trigger", false)};
      const auto trigger_line_source_param{
        static_cast<uint32_t>(declare_parameter(prefix_dot + "trigger_line_source", 0))};
      const auto gain_upper_limit_param{
        static_cast<float>(declare_parameter(prefix_dot + "gain_upper_limit", 18.0))};
      return spinnaker::CameraSettings{
      camera_name,
      static_cast<std::uint32_t>(
        declare_parameter<int64_t>(prefix_dot + "window_width")),
      static_cast<std::uint32_t>(
        declare_parameter<int64_t>(prefix_dot + "window_height")),
      static_cast<float>(declare_parameter<float64_t>(prefix_dot + "fps")),
      declare_parameter<std::string>(prefix_dot + "pixel_format"),
      camera_frame_id_param,
      camera_serial_number_param,
      camera_info_url_param,
      device_link_throughput_limit_param,
      use_external_trigger_param,
      trigger_line_source_param,
      gain_upper_limit_param};
    };

  const std::string camera_settings_param_name{"camera_settings"};
  const std::string cameras_param_name{"cameras"};
  const auto camera_names{declare_parameter(cameras_param_name, std::vector<std::string>{})};
  if (camera_names.empty()) {
    RCLCPP_INFO(
      get_logger(), "No '%s' param provided. Looking for a single camera setting.",
      cameras_param_name.c_str());
    spinnaker_wrapper->create_cameras(
      settings_from_params(camera_settings_param_name));
  } else {
    std::transform(
      camera_names.begin(), camera_names.end(), std::back_inserter(m_settings),
      [&camera_settings_param_name, &settings_from_params](
        const std::string & name) -> spinnaker::CameraSettings {
        return settings_from_params(camera_settings_param_name, name);
      });
    RCLCPP_INFO(get_logger(), "Configuring %lu cameras.", m_settings.size());
    spinnaker_wrapper->create_cameras(m_settings);
  }
  return spinnaker_wrapper->get_cameras();
}

std::vector<SpinnakerCameraNode::ProtectedPublisher>
SpinnakerCameraNode::create_publishers(
  ::rclcpp::Node * node,
  const size_t number_of_cameras,
  const bool use_publisher_per_camera)
{
  if (!node) {
    // TODO(igor): this should really be a terminate. It is a pre-condition violation.
    throw std::runtime_error("The node is not initialized. Cannot create publishers.");
  }
  const auto number_of_publishers = use_publisher_per_camera ? number_of_cameras : 1UL;
  // We must create a new vector here as mutexes are not copyable.
  std::vector<ProtectedPublisher> publishers(number_of_publishers);
  for (auto i = 0U; i < number_of_publishers; ++i) {
    publishers[i].set_publisher(
      std::make_shared<image_transport::CameraPublisher>(
        image_transport::create_camera_publisher(
          node, m_settings.at(i).get_camera_name() + "/image_raw", rmw_qos_profile_sensor_data)));
    const auto camera_info_manager = std::make_shared<camera_info_manager::CameraInfoManager>(node);
    camera_info_manager->loadCameraInfo(m_settings.at(i).get_camera_info_url());
    publishers[i].set_camera_info_manager(camera_info_manager);
  }
  return publishers;
}


void SpinnakerCameraNode::publish_image(
  std::uint32_t camera_index,
  std::unique_ptr<sensor_msgs::msg::Image> image)
{
  const auto publisher_index = m_use_publisher_per_camera ? camera_index : 0UL;
  if (image) {
    if (!m_use_camera_timestamp) {image->header.stamp = this->now();}
    m_publishers.at(publisher_index).publish(std::move(image));
  }
}

void SpinnakerCameraNode::ProtectedPublisher::set_publisher(std::shared_ptr<PublisherT> publisher)
{
  m_publisher = publisher;
}

void SpinnakerCameraNode::ProtectedPublisher::set_camera_info_manager(
  std::shared_ptr<InfoManagerT> camera_info_manager)
{
  m_camera_info_manager = camera_info_manager;
}

void SpinnakerCameraNode::ProtectedPublisher::publish(
  std::unique_ptr<sensor_msgs::msg::Image> image)
{
  if (m_publisher) {
    const std::lock_guard<std::mutex> lock{m_publish_mutex};
    if (image) {
      std::unique_ptr<sensor_msgs::msg::CameraInfo> camera_info(
        new sensor_msgs::msg::CameraInfo(m_camera_info_manager->getCameraInfo()));
      camera_info->header = image->header;
      m_publisher->publish(std::move(image), std::move(camera_info));
    }
  } else {
    throw std::runtime_error("Publisher is nullptr, cannot publish.");
  }
}

}  // namespace camera
}  // namespace drivers
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::drivers::camera::SpinnakerCameraNode)
