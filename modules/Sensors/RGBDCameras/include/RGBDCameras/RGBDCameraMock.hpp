/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Camarero Vera CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>
#include <utility>
#include <vector>

#include "RGBDCameras/IRGBDCamera.hpp"

namespace crf {
namespace sensors {
namespace rgbdcameras {

class RGBDCameraMock : public IRGBDCamera {
 public:
    MOCK_METHOD0(initialize,
        bool());
    MOCK_METHOD0(deinitialize,
        bool());
    MOCK_METHOD0(getFrame,
        cv::Mat());
    MOCK_METHOD1(setResolution,
        bool(const cv::Size&));
    MOCK_METHOD1(setFramerate,
        bool(int fps));
    MOCK_METHOD1(setZoom,
        bool(float zoom));
    MOCK_METHOD2(setPosition,
        bool(float pan, float tilt));
    MOCK_METHOD1(setExposure,
        bool(float exposure));
    MOCK_METHOD1(setShutterSpeed,
        bool(float speed));
    MOCK_METHOD1(setFocusMode,
        bool(ICamera::FocusModes mode));
    MOCK_METHOD1(setFocus,
        bool(float focus));
    MOCK_METHOD1(setISO,
        bool(int iso));
    MOCK_METHOD0(getResolution,
        boost::optional<cv::Size>());
    MOCK_METHOD0(getFramerate,
        boost::optional<int>());
    MOCK_METHOD0(getZoom,
        boost::optional<float>());
    MOCK_METHOD0(getPan,
        boost::optional<float>());
    MOCK_METHOD0(getTilt,
        boost::optional<float>());
    MOCK_METHOD0(getExposure,
        boost::optional<float>());
    MOCK_METHOD0(getShutterSpeed,
        boost::optional<float>());
    MOCK_METHOD0(getFocusMode,
        boost::optional<ICamera::FocusModes>());
    MOCK_METHOD0(getFocus,
        boost::optional<float>());
    MOCK_METHOD0(getISO,
        boost::optional<int>());
    MOCK_METHOD0(getAvailableResolutions,
        std::vector<cv::Size>());
    MOCK_METHOD1(getAvailableFramerates,
        std::vector<int>(const cv::Size&));
    MOCK_METHOD0(getRgbdFrame,
        cv::rgbd::RgbdFrame());
    MOCK_METHOD1(setDepthResolution,
        bool(const cv::Size&));
    MOCK_METHOD1(setDepthFramerate,
        bool(int fps));
    MOCK_METHOD0(getDepthResolution,
        boost::optional<cv::Size>());
    MOCK_METHOD0(getDepthFramerate,
        boost::optional<int>());
    MOCK_METHOD0(getAvailableDepthResolutions,
        std::vector<cv::Size>());
    MOCK_METHOD1(getAvailableDepthFramerates,
        std::vector<int>(const cv::Size&));
    MOCK_METHOD0(getColorCameraMatrix,
        boost::optional<cv::Mat>());
    MOCK_METHOD0(getColorDistortionMatrix,
        boost::optional<cv::Mat>());
    MOCK_METHOD0(getDepthCameraMatrix,
        boost::optional<cv::Mat>());
    MOCK_METHOD0(getDepthDistortionMatrix,
        boost::optional<cv::Mat>());
    MOCK_METHOD0(getDepth2ColorExtrinsics,
        boost::optional<cv::Mat>());
    MOCK_METHOD1(getPointCloud,
        boost::optional<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
        cv::rgbd::RgbdFrame>>(bool));
    MOCK_METHOD1(setDeviceSpecificParameters,
        bool(const nlohmann::json& configData));
};

}  // namespace rgbdcameras
}  // namespace sensors
}  // namespace crf
