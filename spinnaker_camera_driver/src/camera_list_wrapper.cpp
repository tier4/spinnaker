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

#include <spinnaker_camera_driver/camera_list_wrapper.hpp>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

namespace autoware
{
namespace drivers
{
namespace camera
{
namespace spinnaker
{
CameraListWrapper::CameraListWrapper(
  Spinnaker::CameraList camera_list, const CameraSettings & camera_settings)
: m_camera_list{camera_list}
{
  for (std::uint32_t camera_index = 0; camera_index < m_camera_list.GetSize(); ++camera_index) {
    m_cameras.emplace_back(camera_index, m_camera_list.GetByIndex(camera_index), camera_settings);
  }
}

CameraListWrapper::CameraListWrapper(
  Spinnaker::CameraList camera_list, const std::vector<CameraSettings> & camera_settings)
: m_camera_list{camera_list}
{
  if (camera_settings.size() > m_camera_list.GetSize()) {
    throw std::logic_error("Number of settings is greater than the number of available cameras.");
  }
  auto fi = std::find_if(
      camera_settings.cbegin(), camera_settings.cend(),
      [](const CameraSettings & cs) { return (cs.get_serial_number() == 0); });
  bool use_serial = (fi == camera_settings.end());
  for (std::uint32_t camera_index = 0; camera_index < camera_settings.size(); ++camera_index) {
    auto camera_ptr =
        use_serial ? m_camera_list.GetBySerial(std::to_string(
                         camera_settings[camera_index].get_serial_number()))
                   : m_camera_list.GetByIndex(camera_index);
    if (!camera_ptr) {
      throw std::runtime_error(
        "Cannot find a camera whose serial number is " +
        std::to_string(camera_settings[camera_index].get_serial_number()));
    } else {
      m_cameras.emplace_back(camera_index, camera_ptr, camera_settings[camera_index]);
    }
  }
}

CameraListWrapper::~CameraListWrapper()
{
  // Stop all cameras.
  stop_capturing();
  // First destroy all the cameras using their destructors.
  m_cameras.clear();
  // Now clear the camera list that actually destoys the last instances of the cameras.
  m_camera_list.Clear();
}

void CameraListWrapper::start_capturing()
{
  for (auto & camera : m_cameras) {
    camera.start_capturing();
  }
}

void CameraListWrapper::start_capturing(const int index) { m_cameras.at(index).start_capturing(); }

void CameraListWrapper::stop_capturing()
{
  for (auto & camera : m_cameras) {
    camera.stop_capturing();
  }
}

void CameraListWrapper::stop_capturing(const int index) { m_cameras.at(index).stop_capturing(); }

void CameraListWrapper::set_image_callback(CameraWrapper::ImageCallbackFunction callback)
{
  for (auto & camera : m_cameras) {
    camera.set_on_image_callback(callback);
  }
}

}  //  namespace spinnaker
}  // namespace camera
}  // namespace drivers
}  // namespace autoware
