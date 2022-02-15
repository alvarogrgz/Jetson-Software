/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *         Carlos Prados Sesmero CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <boost/optional.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/rs.hpp>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd.hpp>
#include <string>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>

#include "RGBDCameras/IRGBDCamera.hpp"
#include "EventLogger/EventLogger.hpp"

#define WIDTH_RGB_FRAME_D400 640
#define HEIGHT_RGB_FRAME_D400 480
#define WIDTH_DEPTH_FRAME_D400 640
#define HEIGHT_DEPTH_FRAME_D400 480

#define WIDTH_RGB_FRAME_L500 1920
#define HEIGHT_RGB_FRAME_L500 1080
#define WIDTH_DEPTH_FRAME_L500 1024
#define HEIGHT_DEPTH_FRAME_L500 768

namespace crf {
namespace sensors {
namespace rgbdcameras {

class RealSenseCamera : public IRGBDCamera {
 public:
    RealSenseCamera() = delete;
    explicit RealSenseCamera(const std::string& serialNumber);
    RealSenseCamera(const RealSenseCamera&) = delete;
    RealSenseCamera(RealSenseCamera&&) = delete;
    ~RealSenseCamera() override;

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

    cv::rgbd::RgbdFrame getRgbdFrame() override;
    bool setDepthResolution(const cv::Size&) override;
    bool setDepthFramerate(int fps) override;
    boost::optional<cv::Size> getDepthResolution() override;
    boost::optional<int> getDepthFramerate() override;
    std::vector<cv::Size> getAvailableDepthResolutions() override;
    std::vector<int> getAvailableDepthFramerates(const cv::Size& resolution) override;

    boost::optional<cv::Mat> getColorCameraMatrix() override;
    boost::optional<cv::Mat> getColorDistortionMatrix() override;
    boost::optional<cv::Mat> getDepthCameraMatrix() override;
    boost::optional<cv::Mat> getDepthDistortionMatrix() override;
    boost::optional<cv::Mat> getDepth2ColorExtrinsics() override;

    boost::optional<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
        cv::rgbd::RgbdFrame>> getPointCloud(bool alignFrames) override;
    bool setDeviceSpecificParameters(const nlohmann::json& configData) override;

 private:
    utility::logger::EventLogger logger_;

    std::string serialNumber_;
    bool initialized_;

    std::mutex pipeMutex_;

    rs2::config cfg_;
    rs2::pipeline pipe_;
    rs2::pipeline_profile profile_;
    rs2::device device_;
    rs2_extrinsics extrinsic_;
    std::unique_ptr<rs2::color_sensor> colorSensor_;
    std::unique_ptr<rs2::depth_sensor> depthSensor_;
    std::unique_ptr<rs2::motion_sensor> motionSensor_;

    std::unique_ptr<rs2::video_stream_profile> activeColorStream_;
    std::unique_ptr<rs2::video_stream_profile> activeDepthStream_;
    std::unique_ptr<rs2::align> alignToColor_;

    /**
     * @brief Sensor resolution definition
     * @details First pair determines the details of the D400 series, while second one the L500.
     *          For the following pairs, firsly it is detailed the color resolution (W, H) and
     *          second pair determines the depth
     */
    std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> defaultColorResolution {
        std::make_pair(std::make_pair(WIDTH_RGB_FRAME_D400, HEIGHT_RGB_FRAME_D400),
                       std::make_pair(WIDTH_DEPTH_FRAME_D400, HEIGHT_DEPTH_FRAME_D400)),
        std::make_pair(std::make_pair(WIDTH_RGB_FRAME_L500, HEIGHT_RGB_FRAME_L500),
                       std::make_pair(WIDTH_DEPTH_FRAME_L500, HEIGHT_DEPTH_FRAME_L500))
    };
};

}  // namespace rgbdcameras
}  // namespace sensors
}  // namespace crf
