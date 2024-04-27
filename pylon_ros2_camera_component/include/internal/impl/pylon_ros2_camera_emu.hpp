/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2022, Basler AG. All rights reserved.
 * Copyright (C) 2024, KM Robotics s.r.o. (CZ). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * No contributors' name may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#pragma once

#include <rclcpp/logger.hpp>

#include "internal/pylon_ros2_camera_impl.hpp"
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/EnumParameter.h>

#include <string>
#include <vector>

namespace pylon_ros2_camera
{

namespace
{
    static const rclcpp::Logger LOGGER_CAM_EMU = rclcpp::get_logger("basler.pylon.ros2.pylon_ros2_emulated_camera");
}

struct EmuCameraTrait
{
    using CBaslerInstantCameraT = Pylon::CBaslerUniversalInstantCamera;
    using ExposureAutoEnums = Basler_UniversalCameraParams::ExposureAutoEnums;
    using GainAutoEnums = Basler_UniversalCameraParams::GainAutoEnums;
    using PixelFormatEnums = Basler_UniversalCameraParams::PixelFormatEnums;
    using PixelSizeEnums = Basler_UniversalCameraParams::PixelSizeEnums;
    using AutoTargetBrightnessType = GenApi::IFloat;
    using GainType = GenApi::IFloat;
    using AutoTargetBrightnessValueType = double;
    using ShutterModeEnums = Basler_UniversalCameraParams::ShutterModeEnums;
    using UserOutputSelectorEnums = Basler_UniversalCameraParams::UserOutputSelectorEnums;
    using AcquisitionStatusSelectorEnums = Basler_UniversalCameraParams::AcquisitionStatusSelectorEnums;
    using SensorReadoutModeEnums = Basler_UniversalCameraParams::SensorReadoutModeEnums;
    using TriggerSelectorEnums = Basler_UniversalCameraParams::TriggerSelectorEnums;
    using TriggerModeEnums = Basler_UniversalCameraParams::TriggerModeEnums;
    using TriggerSourceEnums = Basler_UniversalCameraParams::TriggerSourceEnums;
    using TriggerActivationEnums = Basler_UniversalCameraParams::TriggerActivationEnums;
    using LineSelectorEnums = Basler_UniversalCameraParams::LineSelectorEnums;
    using LineModeEnums = Basler_UniversalCameraParams::LineModeEnums;
    using DeviceLinkThroughputLimitModeEnums = Basler_UniversalCameraParams::DeviceLinkThroughputLimitModeEnums;
    using AutoFunctionROISelectorEnums = Basler_UniversalCameraParams::AutoFunctionROISelectorEnums;
    using BalanceWhiteAutoEnums = Basler_UniversalCameraParams::BalanceWhiteAutoEnums;
    using LightSourcePresetEnums = Basler_UniversalCameraParams::LightSourcePresetEnums;
    using LineSourceEnums = Basler_UniversalCameraParams::LineSourceEnums;
    using DemosaicingModeEnums = Basler_UniversalCameraParams::DemosaicingModeEnums;
    using PgiModeEnums = Basler_UniversalCameraParams::PgiModeEnums;
    using UserSetSelectorEnums = Basler_UniversalCameraParams::UserSetSelectorEnums;
    using UserSetDefaultSelectorEnums = Basler_UniversalCameraParams::UserSetDefaultEnums;
    using LineFormatEnums = Basler_UniversalCameraParams::LineFormatEnums;
    using BalanceRatioSelectorEnums = Basler_UniversalCameraParams::BalanceRatioSelectorEnums;
    using TimerSelectorEnums = Basler_UniversalCameraParams::TimerSelectorEnums;
    using TimerTriggerSourceEnums = Basler_UniversalCameraParams::TimerTriggerSourceEnums;

    static inline AutoTargetBrightnessValueType convertBrightness(const int& value)
    {
        return value / 255.0;
    }
    double resulting_frame_rate{100.0};
};

using PylonROS2EmuCamera = PylonROS2CameraImpl<EmuCameraTrait>;

template <>
bool PylonROS2EmuCamera::applyCamSpecificStartupSettings(const PylonROS2CameraParameter& parameters)
{
    try
    {
        //cam_->StartGrabbing();
        grabbingStarting();
        cam_->StopGrabbing();

        RCLCPP_INFO_STREAM(LOGGER_CAM_EMU, "Startup user profile set to \"" << parameters.startup_user_set_ << "\"");
        if (parameters.startup_user_set_ == "Default")
        {
            // Remove all previous settings (sequencer etc.)
            // Default Setting = Free-Running
            cam_->UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_Default);
            cam_->UserSetLoad.Execute();
            // UserSetSelector_Default overrides Software Trigger Mode !!
            cam_->TriggerSource.SetValue(Basler_UniversalCameraParams::TriggerSource_Software);
            cam_->TriggerMode.SetValue(Basler_UniversalCameraParams::TriggerMode_On);

             /* Thresholds for the AutoExposure Functions:
              *  - lower limit can be used to get rid of changing light conditions
              *    due to 50Hz lamps (-> 20ms cycle duration)
              *  - upper limit is to prevent motion blur
              */
            double upper_lim = std::min(parameters.auto_exposure_upper_limit_,
                                        cam_->ExposureTime.GetMax());
            cam_->AutoExposureTimeLowerLimit.SetValue(cam_->ExposureTime.GetMin());
            cam_->AutoExposureTimeUpperLimit.SetValue(upper_lim);
            RCLCPP_INFO_STREAM(LOGGER_CAM_EMU, "Cam has upper exposure value limit range: ["
                    << cam_->ExposureTimeAbs.GetMin()
                    << " - " << upper_lim << " (max possible value from cam is " << cam_->ExposureTimeAbs.GetMax() << ")"
                    << "].");

            cam_->AutoGainLowerLimit.SetValue(cam_->Gain.GetMin());
            cam_->AutoGainUpperLimit.SetValue(cam_->Gain.GetMax());

            // The gain auto function and the exposure auto function can be used at the same time. In this case,
            // however, you must also set the Auto Function Profile feature.
            //  cam_->AutoFunctionProfile.SetValue(Basler_UniversalCameraParams::AutoFunctionProfile_MinimizeGain);

            if ( GenApi::IsAvailable(cam_->BinningHorizontal) &&
                 GenApi::IsAvailable(cam_->BinningVertical) )
            {
                RCLCPP_INFO_STREAM(LOGGER_CAM_EMU, "Cam has binning range: x(hz) = ["
                        << cam_->BinningHorizontal.GetMin() << " - "
                        << cam_->BinningHorizontal.GetMax() << "], y(vt) = ["
                        << cam_->BinningVertical.GetMin() << " - "
                        << cam_->BinningVertical.GetMax() << "].");
            }
            else
            {
                RCLCPP_INFO_STREAM(LOGGER_CAM_EMU, "Cam does not support binning.");
            }

            RCLCPP_INFO_STREAM(LOGGER_CAM_EMU, "Cam has exposure time range: [" << cam_->ExposureTime.GetMin()
                    << " - " << cam_->ExposureTime.GetMax()
                    << "] measured in microseconds.");
            RCLCPP_INFO_STREAM(LOGGER_CAM_EMU, "Cam has gain range: [" << cam_->Gain.GetMin()
                    << " - " << cam_->Gain.GetMax()
                    << "] measured in dB.");
            RCLCPP_INFO_STREAM(LOGGER_CAM_EMU, "Cam has gammma range: ["
                    << cam_->Gamma.GetMin() << " - "
                    << cam_->Gamma.GetMax() << "].");
            RCLCPP_INFO_STREAM(LOGGER_CAM_EMU, "Cam has pylon auto brightness range: ["
                    << cam_->AutoTargetBrightness.GetMin() * 255 << " - "
                    << cam_->AutoTargetBrightness.GetMax() * 255
                    << "] which is the average pixel intensity.");
        }
        else if (parameters.startup_user_set_ == "UserSet1")
        {
            cam_->UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet1);
            cam_->UserSetLoad.Execute();
            RCLCPP_INFO(LOGGER_CAM_EMU, "User Set 1 Loaded");
        }
        else if (parameters.startup_user_set_ == "UserSet2")
        {
            cam_->UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet2);
            cam_->UserSetLoad.Execute();
            RCLCPP_INFO(LOGGER_CAM_EMU, "User Set 2 Loaded");
        }
        else if (parameters.startup_user_set_ == "UserSet3")
        {
            cam_->UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet3);
            cam_->UserSetLoad.Execute();
            RCLCPP_INFO(LOGGER_CAM_EMU, "User Set 3 Loaded");
        }
        else if (parameters.startup_user_set_ == "CurrentSetting")
        {
            RCLCPP_INFO(LOGGER_CAM_EMU, "No user set is provided -> Camera current setting will be applied");
        }
        else
        {
            RCLCPP_WARN_STREAM(LOGGER_CAM_EMU, "Unsupported startup user profile \"" << parameters.startup_user_set_ << "\", ignoring");
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_CAM_EMU, "Error applying camera specific startup setting for emulated camera: "
                << e.GetDescription());
        return false;
    }
    return true;
}

template <>
bool PylonROS2EmuCamera::setupSequencer(const std::vector<float>& exposure_times,
                                    std::vector<float>& exposure_times_set)
{
    try
    {
        // Runtime Sequencer: cam_->IsGrabbing() ? cam_->StopGrabbing(); //10ms
        if ( GenApi::IsWritable(cam_->SequencerMode) )
        {
            cam_->SequencerMode.SetValue(Basler_UniversalCameraParams::SequencerMode_Off);
        }
        else
        {
            RCLCPP_ERROR(LOGGER_CAM_EMU, "Sequencer Mode not writable");
        }

        cam_->SequencerConfigurationMode.SetValue(Basler_UniversalCameraParams::SequencerConfigurationMode_On);

        // **** valid for all sets: reset on software signal 1 ****
        int64_t initial_set = cam_->SequencerSetSelector.GetMin();

        cam_->SequencerSetSelector.SetValue(initial_set);
        cam_->SequencerPathSelector.SetValue(0);
        cam_->SequencerSetNext.SetValue(initial_set);
        cam_->SequencerTriggerSource.SetValue(Basler_UniversalCameraParams::SequencerTriggerSource_SoftwareSignal1);
        // advance on Frame Start
        cam_->SequencerPathSelector.SetValue(1);
        cam_->SequencerTriggerSource.SetValue(Basler_UniversalCameraParams::SequencerTriggerSource_FrameStart);
        // ********************************************************

        for ( std::size_t i = 0; i < exposure_times.size(); ++i )
        {
            if ( i > 0 )
            {
                cam_->SequencerSetSelector.SetValue(i);
            }

            if ( i == exposure_times.size() - 1 )  // last frame
            {
                cam_->SequencerSetNext.SetValue(0);
            }
            else
            {
                cam_->SequencerSetNext.SetValue(i + 1);
            }
            float reached_exposure;
            setExposure(exposure_times.at(i), reached_exposure);
            exposure_times_set.push_back(reached_exposure / 1000000.);
            cam_->SequencerSetSave.Execute();
        }

        // config finished
        cam_->SequencerConfigurationMode.SetValue(Basler_UniversalCameraParams::SequencerConfigurationMode_Off);
        cam_->SequencerMode.SetValue(Basler_UniversalCameraParams::SequencerMode_On);
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_CAM_EMU, "ERROR while initializing pylon sequencer: "
                << e.GetDescription());
        return false;
    }
    return true;
}

template <>
GenApi::IFloat& PylonROS2EmuCamera::exposureTime()
{
    if ( !GenApi::IsAvailable(cam_->ExposureTime) )
    {
        throw std::runtime_error("Error while accessing ExposureTime in PylonROS2EmuCamera");
    }
    return cam_->ExposureTime;
}

template <>
EmuCameraTrait::GainType& PylonROS2EmuCamera::gain()
{
    if ( !GenApi::IsAvailable(cam_->Gain) )
    {
        throw std::runtime_error("Error while accessing Gain in PylonROS2EmuCamera");
    }
    return cam_->Gain;
}

template <>
bool PylonROS2EmuCamera::setGamma(const float& target_gamma, float& reached_gamma)
{
    if ( !GenApi::IsAvailable(cam_->Gamma) )
    {
        RCLCPP_ERROR_STREAM(LOGGER_CAM_EMU, "Error while trying to set gamma: cam.Gamma NodeMap is"
               << " not available!");
        return false;
    }

    try
    {
        float gamma_to_set = target_gamma;
        if ( gamma().GetMin() > gamma_to_set )
        {
            gamma_to_set = gamma().GetMin();
            RCLCPP_WARN_STREAM(LOGGER_CAM_EMU, "Desired gamma unreachable! Setting to lower limit: "
                                  << gamma_to_set);
        }
        else if ( gamma().GetMax() < gamma_to_set )
        {
            gamma_to_set = gamma().GetMax();
            RCLCPP_WARN_STREAM(LOGGER_CAM_EMU, "Desired gamma unreachable! Setting to upper limit: "
                                  << gamma_to_set);
        }
        gamma().SetValue(gamma_to_set);
        reached_gamma = currentGamma();
    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_CAM_EMU, "An exception while setting target gamma to "
                << target_gamma << " occurred: " << e.GetDescription());
        return false;
    }
    return true;
}

template <>
GenApi::IFloat& PylonROS2EmuCamera::autoExposureTimeLowerLimit()
{
    if ( !GenApi::IsAvailable(cam_->AutoExposureTimeLowerLimit) )
    {
        throw std::runtime_error("Error while accessing AutoExposureTimeLowerLimit in PylonROS2EmuCamera");
    }
    return cam_->AutoExposureTimeLowerLimit;
}

template <>
GenApi::IFloat& PylonROS2EmuCamera::autoExposureTimeUpperLimit()
{
    if ( !GenApi::IsAvailable(cam_->AutoExposureTimeUpperLimit) )
    {
        throw std::runtime_error("Error while accessing AutoExposureTimeUpperLimit in PylonROS2EmuCamera");
    }
    return cam_->AutoExposureTimeUpperLimit;
}

template <>
EmuCameraTrait::GainType& PylonROS2EmuCamera::autoGainLowerLimit()
{
    if ( !GenApi::IsAvailable(cam_->AutoGainLowerLimit) )
    {
        throw std::runtime_error("Error while accessing AutoGainLowerLimit in PylonROS2EmuCamera");
    }
    return cam_->AutoGainLowerLimit;
}

template <>
EmuCameraTrait::GainType& PylonROS2EmuCamera::autoGainUpperLimit()
{
    if ( !GenApi::IsAvailable(cam_->AutoGainUpperLimit) )
    {
        throw std::runtime_error("Error while accessing AutoGainUpperLimit in PylonROS2EmuCamera");
    }
    return cam_->AutoGainUpperLimit;
}

template <>
GenApi::IFloat& PylonROS2EmuCamera::resultingFrameRate()
{
    if ( !GenApi::IsAvailable(cam_->ResultingFrameRateAbs) )
    {
        throw std::runtime_error("Error while accessing ResultingFrameRateAbs in PylonROS2EmuCamera");
    }
    return cam_->ResultingFrameRateAbs;
}

template <>
EmuCameraTrait::AutoTargetBrightnessType& PylonROS2EmuCamera::autoTargetBrightness()
{
    if ( !GenApi::IsAvailable(cam_->AutoTargetBrightness) )
    {
        throw std::runtime_error("Error while accessing AutoTargetBrightness in PylonROS2EmuCamera");
    }
    return cam_->AutoTargetBrightness;
}

template <>
std::string PylonROS2EmuCamera::typeName() const
{
    return "EMU";
}

template <>
std::string PylonROS2EmuCamera::setAcquisitionFrameCount(const int& frameCount)
{
    try
    {
        if ( GenApi::IsAvailable(cam_->AcquisitionBurstFrameCount) )
        {
            cam_->AcquisitionBurstFrameCount.SetValue(frameCount);
            return "done";
        }
        else if ( GenApi::IsAvailable(cam_->AcquisitionFrameRate) )
        {
            cam_->AcquisitionFrameRate.SetValue(frameCount);
            return "done";
        }
        else
        {
             RCLCPP_ERROR_STREAM(LOGGER_CAM_EMU, "Error while trying to change the Acquisition frame count. The connected Camera not supporting this feature");
             return "The connected Camera not supporting this feature";
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        RCLCPP_ERROR_STREAM(LOGGER_CAM_EMU, "An exception while changing Acquisition frame count occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}

template <>
int PylonROS2EmuCamera::getAcquisitionFrameCount()
{
    try
    {
        if ( GenApi::IsAvailable(cam_->AcquisitionBurstFrameCount) )
        {
            return static_cast<int>(cam_->AcquisitionBurstFrameCount.GetValue());
        }
        else if ( GenApi::IsAvailable(cam_->AcquisitionFrameRate) )
        {
            return static_cast<int>(cam_->AcquisitionFrameRate.GetValue());
        }
        else
        {
             return -10000;
        }

    }
    catch ( const GenICam::GenericException &e )
    {
        return -20000;
    }
}

template <>
std::string PylonROS2EmuCamera::setGammaSelector(const int& gammaSelector __attribute__((unused)))
{
    RCLCPP_ERROR_STREAM(LOGGER_CAM_EMU, "Error while trying to change the gamma selector. The connected camera does not support this feature");
    return "Error, the connected camera does not support this feature";
}

template <>
std::string PylonROS2EmuCamera::gammaEnable(const bool& enable __attribute__((unused)))
{
    RCLCPP_ERROR_STREAM(LOGGER_CAM_EMU, "Error while trying to enable/disable gamma. The connected camera does not support this feature");
    return "Error, the connected camera does not support this feature";
}

template <>
float PylonROS2EmuCamera::getTemperature(){
    try
    {
        if ( GenApi::IsAvailable(cam_->DeviceTemperature) )
        {
            return static_cast<float>(cam_->DeviceTemperature.GetValue());
        }
        else
        {
             return 0.0;
        }
    }
    catch ( const GenICam::GenericException &e )
    {
        return 0.0;
    }
}

template <>
std::string PylonROS2EmuCamera::setTimerSelector(const int& selector)
{
    try
    {
        if (GenApi::IsAvailable(cam_->TimerSelector))
        {
            switch (selector)
            {
                case 1:
                    cam_->TimerSelector.SetValue(TimerSelectorEnums::TimerSelector_Timer1);
                    return "done";
                case 2:
                    cam_->TimerSelector.SetValue(TimerSelectorEnums::TimerSelector_Timer2);
                    return "done";
                default:
                    RCLCPP_ERROR_STREAM(LOGGER_CAM_EMU, "Timer selector value is invalid! Please choose between 1 -> Timer 1 / 2 -> Timer 2");
                    return "Error: unknown value for timer selector";
            }
        }
        RCLCPP_ERROR_STREAM(LOGGER_CAM_EMU, "Error while trying to change the timer selector. The connected camera does not support this feature");
        return "The connected camera does not support this feature";
    }
    catch (const GenICam::GenericException &e)
    {
        RCLCPP_ERROR_STREAM(LOGGER_CAM_EMU, "An exception while setting the timer selector occurred:" << e.GetDescription());
        return e.GetDescription();
    }
}


}  // namespace pylon_ros2_camera
