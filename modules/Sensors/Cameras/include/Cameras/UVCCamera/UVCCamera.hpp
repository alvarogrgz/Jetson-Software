/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author:  Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <linux/videodev2.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <sys/mman.h>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>

#include "Cameras/ICamera.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace cameras {

class UVCCamera : public ICamera {
 public:
    UVCCamera() = delete;
    explicit UVCCamera(const std::string& device_name);
    explicit UVCCamera(const std::string& device_name, unsigned int encoding);
    UVCCamera(const UVCCamera&) = delete;
    UVCCamera(UVCCamera&&) = delete;
    ~UVCCamera() override;
    bool initialize() override;
    bool deinitialize() override;

    bool setResolution(const cv::Size&) override;
    bool setFramerate(int fps) override;
    bool setZoom(float zoom) override;
    bool setPosition(float pan, float tilt) override;
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
    class mmap_deleter {
        std::size_t m_size;
     public:
        mmap_deleter() : m_size(0) {}
        explicit mmap_deleter(std::size_t size) : m_size{size} {}
        void operator()(void *ptr) const {munmap(ptr, m_size);}
    };

    utility::logger::EventLogger logger_;

    std::string devName_;
    bool streamOn_;
    bool initialized_;
    bool isZoomSupported_;

    int cameraFd_;
    int requestedFps_;
    unsigned int pixelFormat_;
    float digitalZoom_;

    cv::Size resolution_;
    cv::Size requestedResolution_;

    std::vector<std::unique_ptr<char, mmap_deleter> > buffers_;
    std::vector<struct v4l2_buffer> buffersinfo_;
    int currentBuffer_;
    std::mutex tryIoctlMutex_;
    const int tryIoctlTimeoutS_ = 10;

    bool isZoomSupported();
    cv::Mat applyZoom(const cv::Mat &frame, float zoom);
    std::vector<unsigned int> getAvailableFormats();

    bool setMmapVideo();
    void unmmap(char*);
    bool startFrameStream();
    bool stopFrameStream();
    bool tryIoctl(uint64_t ioctlCode, void *parameter, bool failIfBusy = true, int attempts = 10);
};

}  // namespace cameras
}  // namespace sensors
}  // namespace crf
