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
#include <spinnaker_camera_driver/camera_wrapper.hpp>

#include <Spinnaker.h>

#include <memory>
#include <string>

namespace autoware
{
namespace drivers
{
namespace camera
{
namespace spinnaker
{
CameraWrapper::CameraWrapper(std::uint32_t camera_index, const Spinnaker::CameraPtr & camera)
: m_camera_index{camera_index}, m_camera{camera}
{
  m_camera->Init();
}

CameraWrapper::CameraWrapper(
  std::uint32_t camera_index, const Spinnaker::CameraPtr & camera,
  const CameraSettings & camera_settings)
: CameraWrapper{camera_index, camera}
{
  configure_camera(camera_settings);
}

CameraWrapper::~CameraWrapper()
{
  if (m_on_image_callback) {
    m_camera->UnregisterEventHandler(*this);
  }
  if (m_camera->IsStreaming()) {
    stop_capturing();
  }
  m_camera->DeInit();
}

void CameraWrapper::OnImageEvent(Spinnaker::ImagePtr image)
{
  if (m_on_image_callback) {
    m_on_image_callback(m_camera_index, m_frame_id, image);
    image->Release();
  }
}

void CameraWrapper::start_capturing()
{
  if (!m_is_camera_configured) {
    throw std::logic_error("Please configure the camera before capturing images.");
  }
  try {
    if (m_camera && !m_camera_is_capturing) {
      m_camera->BeginAcquisition();
      m_camera_is_capturing = true;
    }
  } catch (const Spinnaker::Exception & e) {
    throw std::runtime_error("Failed to start capture: " + std::string(e.what()));
  }
}

void CameraWrapper::stop_capturing()
{
  if (m_camera && m_camera_is_capturing) {
    try {
      m_camera->EndAcquisition();
      m_camera_is_capturing = false;
    } catch (const Spinnaker::Exception & e) {
      throw std::runtime_error("Failed to stop capture: " + std::string(e.what()));
    }
  }
}

void CameraWrapper::set_on_image_callback(ImageCallbackFunction callback)
{
  if (!m_on_image_callback) {
    // This is the first time we are setting a callback so we want to register
    // event handling for this camera.
    m_camera->RegisterEventHandler(*this);
  }
  m_on_image_callback = callback;
}

Spinnaker::PixelFormatEnums CameraWrapper::convert_to_pixel_format_enum(
  const std::string & pixel_format)
{
  if (pixel_format == CameraSettings::kPixelFormatStr_RGGB8) {
    return Spinnaker::PixelFormatEnums::PixelFormat_BayerRG8;
  }
  if (pixel_format == CameraSettings::kPixelFormatStr_GRBG8) {
    return Spinnaker::PixelFormatEnums::PixelFormat_BayerGR8;
  }
  if (pixel_format == CameraSettings::kPixelFormatStr_GBRG8) {
    return Spinnaker::PixelFormatEnums::PixelFormat_BayerGB8;
  }
  if (pixel_format == CameraSettings::kPixelFormatStr_BGGR8) {
    return Spinnaker::PixelFormatEnums::PixelFormat_BayerBG8;
  }
  if (pixel_format == CameraSettings::kPixelFormatStr_RGB8) {
    return Spinnaker::PixelFormatEnums::PixelFormat_RGB8;
  }
  if (pixel_format == CameraSettings::kPixelFormatStr_BGR8) {
    return Spinnaker::PixelFormatEnums::PixelFormat_BGR8;
  }
  if (pixel_format == CameraSettings::kPixelFormatStr_MONO8) {
    return Spinnaker::PixelFormatEnums::PixelFormat_Mono8;
  }
  throw std::invalid_argument("Unknown pixel format.");
}

Spinnaker::TriggerSourceEnums CameraWrapper::convert_to_trigger_source_enum(const int line_source)
{
  if (line_source == 0) {
    return Spinnaker::TriggerSource_Line0;
  }
  if (line_source == 1) {
    return Spinnaker::TriggerSource_Line1;
  }
  if (line_source == 2) {
    return Spinnaker::TriggerSource_Line2;
  }
  if (line_source == 3) {
    return Spinnaker::TriggerSource_Line3;
  }
  throw std::invalid_argument("Unknown line source.");
}

/// Configure a Spinnaker camera.
void CameraWrapper::configure_camera(const CameraSettings & camera_settings)
{
  // Make sure this camera does not send callbacks while it is being configured.
  bool camera_should_capture{m_camera_is_capturing};
  if (m_camera_is_capturing) {
    stop_capturing();
  }

  using Spinnaker::GenApi::IsAvailable;
  using Spinnaker::GenApi::IsWritable;

  m_frame_id = camera_settings.get_frame_id();

  m_camera->Width.SetValue(camera_settings.get_window_width());
  m_camera->Height.SetValue(camera_settings.get_window_height());

  auto IsAvailableAndWritable = [](const Spinnaker::GenApi::IBase & value) {
    return IsAvailable(value) && IsWritable(value);
  };

  if (IsAvailable(m_camera->DeviceType)) {
    if (m_camera->DeviceType.GetCurrentEntry()->GetSymbolic() == "GEV") {
      if (IsAvailableAndWritable(m_camera->DeviceLinkThroughputLimit)) {
        m_camera->DeviceLinkThroughputLimit.SetValue(
          camera_settings.get_device_link_throughput_limit());
      } else {
        throw std::invalid_argument("Cannot set throuput limit on the m_camera.");
      }
    }
  }
  if (IsAvailableAndWritable(m_camera->AcquisitionFrameRateEnable)) {
    m_camera->AcquisitionFrameRateEnable.SetValue(true);
    if (IsAvailableAndWritable(m_camera->AcquisitionFrameRate)) {
      m_camera->AcquisitionFrameRate.SetValue(camera_settings.get_fps());
    }
  }
  if (IsAvailableAndWritable(m_camera->PixelFormat)) {
    m_camera->PixelFormat.SetValue(
      convert_to_pixel_format_enum(camera_settings.get_pixel_format()));
  } else {
    throw std::invalid_argument("Cannot set pixel format.");
  }
  if (IsAvailableAndWritable(m_camera->AcquisitionMode)) {
    m_camera->AcquisitionMode.SetValue(Spinnaker::AcquisitionModeEnums::AcquisitionMode_Continuous);
  } else {
    throw std::invalid_argument("Cannot set continuous acquisition mode.");
  }

  if (m_camera->TriggerMode.GetAccessMode() != Spinnaker::GenApi::RW) {
    throw std::invalid_argument("Unable to disable trigger mode.");
  } else {
    m_camera->TriggerMode.SetValue(Spinnaker::TriggerMode_Off);
  }

  if (camera_settings.get_use_external_trigger()) {
    if (m_camera->TriggerSource.GetAccessMode() != Spinnaker::GenApi::RW) {
      throw std::invalid_argument("Unable to set trigger mode (node retrieval).");
    } else {
      m_camera->TriggerSource.SetValue(
        convert_to_trigger_source_enum(camera_settings.get_trigger_line_source()));
    }

    if (m_camera->TriggerMode.GetAccessMode() != Spinnaker::GenApi::RW) {
      throw std::invalid_argument("Unable to disable trigger mode.");
    } else {
      m_camera->TriggerMode.SetValue(Spinnaker::TriggerMode_On);
    }
  }

  if (m_camera->AutoExposureGainUpperLimit.GetAccessMode() != Spinnaker::GenApi::RW) {
    throw std::invalid_argument("Cannot set gain upper limit.");
  } else {
    m_camera->AutoExposureGainUpperLimit.SetValue(camera_settings.get_gain_upper_limit());
  }

  m_is_camera_configured = true;

  if (camera_should_capture) {
    start_capturing();
  }
}

}  //  namespace spinnaker
}  //  namespace camera
}  //  namespace drivers
}  //  namespace autoware
