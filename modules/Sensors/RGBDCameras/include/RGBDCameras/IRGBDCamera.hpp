/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <boost/optional.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <utility>
#include <vector>

#include "Cameras/ICamera.hpp"

namespace crf {
namespace sensors {
namespace rgbdcameras {

class IRGBDCamera : public cameras::ICamera {
 public:
    virtual ~IRGBDCamera() = default;
    bool initialize() override = 0;
    bool deinitialize() override = 0;

    cv::Mat getFrame() override = 0;

    bool setResolution(const cv::Size&) override = 0;
    bool setFramerate(int fps) override = 0;
    bool setZoom(float zoom) override = 0;
    bool setPosition(float pan, float tilt) override = 0;
    bool setExposure(float exposure) override = 0;
    bool setShutterSpeed(float speed) override = 0;
    bool setFocusMode(ICamera::FocusModes mode) override = 0;
    bool setFocus(float focus) override = 0;
    bool setISO(int iso) override = 0;

    boost::optional<cv::Size> getResolution() override = 0;
    boost::optional<int> getFramerate() override = 0;
    boost::optional<float> getZoom() override = 0;
    boost::optional<float> getPan() override = 0;
    boost::optional<float> getTilt() override = 0;
    boost::optional<float> getExposure() override = 0;
    boost::optional<float> getShutterSpeed() override = 0;
    boost::optional<ICamera::FocusModes> getFocusMode() override = 0;
    boost::optional<float> getFocus() override = 0;
    boost::optional<int> getISO() override = 0;

    std::vector<cv::Size> getAvailableResolutions() override = 0;
    std::vector<int> getAvailableFramerates(const cv::Size& resolution) override = 0;

    virtual cv::rgbd::RgbdFrame getRgbdFrame() = 0;
    virtual bool setDepthResolution(const cv::Size&) = 0;
    virtual bool setDepthFramerate(int fps) = 0;
    virtual boost::optional<cv::Size> getDepthResolution() = 0;
    virtual boost::optional<int> getDepthFramerate() = 0;
    virtual std::vector<cv::Size> getAvailableDepthResolutions() = 0;
    virtual std::vector<int> getAvailableDepthFramerates(const cv::Size& resolution) = 0;
    virtual boost::optional<cv::Mat> getColorCameraMatrix() = 0;
    virtual boost::optional<cv::Mat> getColorDistortionMatrix() = 0;
    virtual boost::optional<cv::Mat> getDepthCameraMatrix() = 0;
    virtual boost::optional<cv::Mat> getDepthDistortionMatrix() = 0;
    virtual boost::optional<cv::Mat> getDepth2ColorExtrinsics() = 0;

    virtual boost::optional<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
        cv::rgbd::RgbdFrame>> getPointCloud(bool alignFrames) = 0;
    virtual bool setDeviceSpecificParameters(const nlohmann::json& configData) = 0;
};

}  // namespace rgbdcameras
}  // namespace sensors
}  // namespace crf
