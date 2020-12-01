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

#ifndef SPINNAKER_CAMERA_DRIVER__CAMERA_SETTINGS_HPP_
#define SPINNAKER_CAMERA_DRIVER__CAMERA_SETTINGS_HPP_

#include <spinnaker_camera_driver/visibility_control.hpp>

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
/// Encapsulate settings that make sense to pass to a camera.
class SPINNAKER_CAMERA_PUBLIC CameraSettings
{
public:
  static const char * kPixelFormatStr_RGGB8;
  static const char * kPixelFormatStr_GRBG8;
  static const char * kPixelFormatStr_GBRG8;
  static const char * kPixelFormatStr_BGGR8;
  static const char * kPixelFormatStr_RGB8;
  static const char * kPixelFormatStr_BGR8;
  static const char * kPixelFormatStr_MONO8;
  static const char * kPixelFormatStr_UNKNOWN;

  /// Instantiate settings and throw if they are not valid.
  explicit CameraSettings(
    const std::string & camera_name, std::uint32_t window_width, std::uint32_t window_height,
    float fps, const std::string & pixel_format, const std::string & frame_id = "camera",
    std::uint32_t serial_number = 0, const std::string & camera_info_url = "",
    std::int64_t device_link_throughput_limit = 100000000L, bool use_external_trigger = false,
    int trigger_line_source = 0, float gain_upper_limit = 18.0);

  inline const std::string & get_camera_name() const noexcept { return m_camera_name; }
  inline std::uint32_t get_window_width() const noexcept { return m_window_width; }
  inline std::uint32_t get_window_height() const noexcept { return m_window_height; }
  inline const std::string & get_pixel_format() const noexcept { return m_pixel_format; }
  inline const std::string & get_frame_id() const noexcept { return m_frame_id; }
  inline float get_fps() const noexcept { return m_fps; }
  inline std::uint32_t get_serial_number() const noexcept { return m_serial_number; }
  inline std::string get_camera_info_url() const noexcept { return m_camera_info_url; }
  inline std::int64_t get_device_link_throughput_limit() const noexcept
  {
    return m_device_link_throughput_limit;
  }
  inline bool get_use_external_trigger() const noexcept { return m_use_external_trigger; }
  inline int get_trigger_line_source() const noexcept { return m_trigger_line_source; }
  inline float get_gain_upper_limit() const noexcept { return m_gain_upper_limit; }

private:
  /// A set of all valid pixel formats.
  static const std::set<std::string> kValidPixelFormats;

  /// Camera name
  std::string m_camera_name;
  /// Width of the returned image.
  std::uint32_t m_window_width;
  /// Height of the returned image.
  std::uint32_t m_window_height;

  /// Format of pixels.
  std::string m_pixel_format;

  /// Camera frame id.
  std::string m_frame_id;

  /// Wanted fps.
  float m_fps;

  /// Camera serial number
  uint32_t m_serial_number;

  /// Camera info url
  std::string m_camera_info_url;

  /// Desired device link throuput limit.
  std::int64_t m_device_link_throughput_limit;

  bool m_use_external_trigger;

  int m_trigger_line_source;

  float m_gain_upper_limit;
};  // namespace spinnaker

}  // namespace spinnaker
}  // namespace camera
}  // namespace drivers
}  //  namespace autoware

#endif  // SPINNAKER_CAMERA_DRIVER__CAMERA_SETTINGS_HPP_
