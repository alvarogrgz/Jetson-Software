/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <shared_mutex>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>

#include "RGBDCameras/IRGBDCamera.hpp"
#include "Cameras/CameraCommunicationPoint/CameraManager.hpp"
#include "CommonInterfaces/IInitializable.hpp"
#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace sensors {
namespace rgbdcameras {

/*
 * @brief RGBD Camera manager class for camera communication point. The camera manager class
 *        ensures thread safe access to the camera. It is also responsible to optimize the
 *        resources. If no communication point is requesting frames the camera is deinitialized to
 *        release resources on the running controller (e.g. USB bandwidth). The communication point
 *        requests a frame stream to the camera manager with a certain resolution. It is also
 *        responsible of changing the camera parameters.
 */
class RGBDCameraManager: public cameras::CameraManager {
 public:
    RGBDCameraManager() = delete;
    RGBDCameraManager(std::shared_ptr<IRGBDCamera> camera,
        const std::chrono::milliseconds& grabberTimeout = std::chrono::seconds(5),
        const std::chrono::milliseconds& inizializationTimeout = std::chrono::seconds(10));
    RGBDCameraManager(const RGBDCameraManager& other) = delete;
    RGBDCameraManager(RGBDCameraManager&& other) = delete;
    ~RGBDCameraManager() override;

    bool startFrameStream(int streamID, const cv::Size& resolution, int framerate);
    bool startFrameStream(int streamID, bool RGBDStream, bool PointCloudStream, const cv::Size& resolution, int framerate, const cv::Size& depthResolution, int depthFramerate);
    void stopFrameStream(int streamID);

    nlohmann::json getStatus() override;

    cv::Mat getFrame(int streamID,
        const std::chrono::milliseconds& timeout = std::chrono::seconds(2));
    cv::rgbd::RgbdFrame getRGBDFrame(int streamID,
        const std::chrono::milliseconds& timeout = std::chrono::seconds(1));
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getPointCloud(int streamID,
        bool alignFrames,
        const std::chrono::milliseconds& timeout = std::chrono::seconds(1));


    cv::Size getMaxRequestedDepthResolution();
    int getMaxRequestedDepthFramerate();
    bool requestPointCloud(int streamID, bool requested_pointcloud);
    bool requestRGBDFrame(int streamID, bool requested_video);

 protected:
    std::atomic<cv::Size> maxRequestedDepthResolution_;
    std::atomic<int> maxRequestedDepthFramerate_;
    std::map<int, std::pair<cv::Size, int>> requestedDepthParametersMap_;
    void frameGrabber() override;
    //bool setClosestHigherResolution(const cv::Size& resolution) override;

 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<IRGBDCamera> rgbdCamera_;

    cv::rgbd::RgbdFrame latestRGBDFrame_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr latestPointCloud_;
    std::shared_timed_mutex requestedRGBDFramesMapMutex_;
    std::map<int, bool> requestedRGBDFramesMap_;
    std::shared_timed_mutex requestedPointCloudsMapMutex_;
    std::map<int, bool> requestedPointCloudsMap_;

    // cv::Size getClosestHigherDepthResolution(const cv::Size& resolution);

    bool isPointCloudRequested();
    bool isRGBDFrameRequested();
};

}  // namespace rgbdcameras
}  // namespace sensors
}  // namespace crf
