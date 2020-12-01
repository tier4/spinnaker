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

#include <spinnaker_camera_driver/camera_settings.hpp>

#include <set>
#include <stdexcept>
#include <string>

namespace autoware
{
namespace drivers
{
namespace camera
{
namespace spinnaker
{
const char * CameraSettings::kPixelFormatStr_RGGB8 = "bayer_rggb8";
const char * CameraSettings::kPixelFormatStr_GRBG8 = "bayer_grbg8";
const char * CameraSettings::kPixelFormatStr_GBRG8 = "bayer_gbrg8";
const char * CameraSettings::kPixelFormatStr_BGGR8 = "bayer_bggr8";
const char * CameraSettings::kPixelFormatStr_RGB8 = "rgb8";
const char * CameraSettings::kPixelFormatStr_BGR8 = "bgr8";
const char * CameraSettings::kPixelFormatStr_MONO8 = "mono8";

const std::set<std::string> CameraSettings::kValidPixelFormats{
  kPixelFormatStr_RGGB8, kPixelFormatStr_GRBG8, kPixelFormatStr_GBRG8, kPixelFormatStr_BGGR8,
  kPixelFormatStr_RGB8,  kPixelFormatStr_BGR8,  kPixelFormatStr_MONO8};

CameraSettings::CameraSettings(
  const std::string & camera_name, std::uint32_t window_width, std::uint32_t window_height,
  float fps, const std::string & pixel_format, const std::string & frame_id,
  std::uint32_t serial_number, const std::string & camera_info_url,
  std::int64_t device_link_throughput_limit, bool use_external_trigger, int trigger_line_source,
  float gain_upper_limit)
: m_camera_name{camera_name},
  m_window_width{window_width},
  m_window_height{window_height},
  m_pixel_format{pixel_format},
  m_frame_id{frame_id},
  m_fps{fps},
  m_serial_number{serial_number},
  m_camera_info_url{camera_info_url},
  m_device_link_throughput_limit{device_link_throughput_limit},
  m_use_external_trigger{use_external_trigger},
  m_trigger_line_source{trigger_line_source},
  m_gain_upper_limit{gain_upper_limit}
{
  if (m_window_width < 1 || m_window_height < 1) {
    throw std::invalid_argument("Window size setting cannot be 0.");
  }
  if (m_fps <= 0) {
    throw std::invalid_argument("FPS setting must be bigger than 0.");
  }
  if (kValidPixelFormats.count(m_pixel_format) < 1) {
    throw std::invalid_argument("Invalid pixel format provided.");
  }
}

}  //  namespace spinnaker
}  //  namespace camera
}  //  namespace drivers
}  //  namespace autoware
