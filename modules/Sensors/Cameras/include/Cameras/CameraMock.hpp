/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Camarero Vera CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <gmock/gmock.h>

#include <vector>
#include <nlohmann/json.hpp>

#include "Cameras/ICamera.hpp"

namespace crf {
namespace sensors {
namespace cameras {

class CameraMock : public ICamera {
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
        bool(FocusModes mode));
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
        boost::optional<FocusModes>());
    MOCK_METHOD0(getFocus,
        boost::optional<float>());
    MOCK_METHOD0(getISO,
        boost::optional<int>());
    MOCK_METHOD0(getAvailableResolutions,
        std::vector<cv::Size>());
    MOCK_METHOD1(getAvailableFramerates,
        std::vector<int>(const cv::Size& resolution));
    MOCK_METHOD1(setDeviceSpecificParameters,
        bool(const nlohmann::json& configData));
};

}  // namespace cameras
}  // namespace sensors
}  // namespace crf
