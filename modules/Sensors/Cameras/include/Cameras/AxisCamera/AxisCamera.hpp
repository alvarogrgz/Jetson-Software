/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author:  Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <condition_variable>
#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <utility>
#include <vector>
#include <thread>
#include <nlohmann/json.hpp>

#include "Cameras/ICamera.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace cameras {

class AxisCamera : public ICamera {
 public:
    AxisCamera() = delete;
    AxisCamera(const std::string& http_address,
        const std::string& username = "",
        const std::string& password = "");
    AxisCamera(const AxisCamera&) = delete;
    AxisCamera(AxisCamera&&) = delete;
    ~AxisCamera() override = default;

    bool initialize() override;
    bool deinitialize() override;

    bool setResolution(const cv::Size&) override;
    bool setFramerate(int fps) override;
    bool setZoom(float zoom) override;
    bool setPosition(float pan, float tilt);
    bool setExposure(float exposure) override;
    bool setShutterSpeed(float) override;
    bool setFocusMode(FocusModes) override;
    bool setFocus(float) override;
    bool setISO(int) override;

    boost::optional<cv::Size> getResolution() override;
    boost::optional<int> getFramerate() override;
    boost::optional<float> getZoom() override;
    boost::optional<float> getPan() override;
    boost::optional<float> getTilt() override;
    boost::optional<float> getExposure() override;
    boost::optional<float> getShutterSpeed() override;
    boost::optional<FocusModes> getFocusMode() override;
    boost::optional<float> getFocus() override;
    boost::optional<int> getISO() override;
    cv::Mat getFrame() override;

    std::vector<cv::Size> getAvailableResolutions() override;
    std::vector<int> getAvailableFramerates(const cv::Size& resolution) override;
    bool setDeviceSpecificParameters(const nlohmann::json& configData) override;

 private:
    utility::logger::EventLogger logger_;

    std::string cameraAddress_;
    std::string username_;
    std::string password_;
    std::string cameraCompleteAddress_;

    bool streamOn_;
    bool initialized_;

    std::mutex frameMutex_;
    std::condition_variable frameCv_;
    std::chrono::time_point<std::chrono::high_resolution_clock> lastFrameTime_;
    size_t latestFrameSize_;
    cv::Mat latestFrame_;
    cv::Size currentResolution_;
    int currentFramerate_;

    std::atomic<bool> stopStreamRequest_;
    std::thread frameGrabberThread_;
    bool frameGrabberLoop();

    std::string httpRequest(const std::string& request);
    std::vector<std::string> split(const std::string& s, const char seperator);

    bool startFrameStream();
    bool stopFrameStream();

    bool parseFrame(std::string& buffer);  // NOLINT
};

}  // namespace cameras
}  // namespace sensors
}  // namespace crf
