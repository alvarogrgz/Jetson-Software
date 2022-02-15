#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Camarero Vera, Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <boost/optional.hpp>
#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>

#include "CommonInterfaces/IInitializable.hpp"

namespace crf {
namespace sensors {
namespace cameras {

class ICamera : public utility::commoninterfaces::IInitializable {
 public:
    enum FocusModes {Manual = 0, Auto = 1};

    virtual ~ICamera() = default;
    bool initialize() override = 0;
    bool deinitialize() override = 0;

    virtual cv::Mat getFrame() = 0;

    virtual bool setResolution(const cv::Size&) = 0;
    virtual bool setFramerate(int fps) = 0;
    virtual bool setZoom(float zoom) = 0;
    virtual bool setPosition(float pan, float tilt) = 0;
    virtual bool setExposure(float exposure) = 0;
    virtual bool setShutterSpeed(float speed) = 0;
    virtual bool setFocusMode(FocusModes mode) = 0;
    virtual bool setFocus(float focus) = 0;
    virtual bool setISO(int iso) = 0;

    virtual boost::optional<cv::Size> getResolution() = 0;
    virtual boost::optional<int> getFramerate() = 0;
    virtual boost::optional<float> getZoom() = 0;
    virtual boost::optional<float> getPan() = 0;
    virtual boost::optional<float> getTilt() = 0;
    virtual boost::optional<float> getExposure() = 0;
    virtual boost::optional<float> getShutterSpeed() = 0;
    virtual boost::optional<FocusModes> getFocusMode() = 0;
    virtual boost::optional<float> getFocus() = 0;
    virtual boost::optional<int> getISO() = 0;

    virtual std::vector<cv::Size> getAvailableResolutions() = 0;
    virtual std::vector<int> getAvailableFramerates(const cv::Size& resolution) = 0;
    virtual bool setDeviceSpecificParameters(const nlohmann::json& configData) = 0;
};

}  // namespace cameras
}  // namespace sensors
}  // namespace crf
