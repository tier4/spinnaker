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

#ifndef SPINNAKER_CAMERA_NODE__SPINNAKER_CAMERA_NODE_HPP_
#define SPINNAKER_CAMERA_NODE__SPINNAKER_CAMERA_NODE_HPP_

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

#include <spinnaker_camera_driver/camera_settings.hpp>
#include <spinnaker_camera_driver/system_wrapper.hpp>
#include <spinnaker_camera_node/visibility_control.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace autoware
{
namespace drivers
{
namespace camera
{
class SPINNAKER_CAMERA_NODE_PUBLIC SpinnakerCameraNode : public ::rclcpp::Node
{
public:
  /// ROS 2 parameter contructor.
  /// \param[in] node_options Node options for this node.
  explicit SpinnakerCameraNode(const rclcpp::NodeOptions & node_options);

private:
  /// A wrapper around a publisher that handles proper multithreading protection.
  class SPINNAKER_CAMERA_NODE_LOCAL ProtectedPublisher;

  /// This funciton is called to publish an image.
  SPINNAKER_CAMERA_NODE_LOCAL void publish_image(
    std::uint32_t camera_index, const std::string & frame_id, const Spinnaker::ImagePtr & image);

  SPINNAKER_CAMERA_NODE_LOCAL std::unique_ptr<sensor_msgs::msg::Image> convert_to_image_msg(
    const Spinnaker::ImagePtr & image, const std::string & frame_id);

  SPINNAKER_CAMERA_NODE_LOCAL std::string convert_to_pixel_format_string(
    Spinnaker::PixelFormatEnums pixel_format);

  /// Helper function to parse camera-related params and create cameras from them.
  SPINNAKER_CAMERA_NODE_LOCAL spinnaker::CameraListWrapper & create_cameras_from_params(
    spinnaker::SystemWrapper * spinnaker_wrapper);

  /// Helper function to create publishers.
  SPINNAKER_CAMERA_NODE_LOCAL std::vector<ProtectedPublisher> create_publishers(
    ::rclcpp::Node * node, size_t number_of_cameras, bool use_publisher_per_camera);

  std::unique_ptr<spinnaker::SystemWrapper> m_spinnaker_wrapper{};
  std::vector<spinnaker::CameraSettings> m_settings{};
  std::vector<ProtectedPublisher> m_publishers{};
  bool m_use_publisher_per_camera{};
};

class SpinnakerCameraNode::ProtectedPublisher
{
  using PublisherT = image_transport::CameraPublisher;
  using InfoManagerT = camera_info_manager::CameraInfoManager;

public:
  /// Co-share ownership of a rclcpp publisher.
  void set_publisher(std::shared_ptr<PublisherT> publisher);
  /// Set camera info manager.
  void set_camera_info_manager(std::shared_ptr<InfoManagerT> camera_info_manager);
  /// Publish an image.
  void publish(std::unique_ptr<sensor_msgs::msg::Image> image);

private:
  std::mutex m_publish_mutex{};
  std::shared_ptr<PublisherT> m_publisher{};
  std::shared_ptr<InfoManagerT> m_camera_info_manager{};
};

}  // namespace camera
}  // namespace drivers
}  // namespace autoware

#endif  // SPINNAKER_CAMERA_NODE__SPINNAKER_CAMERA_NODE_HPP_
