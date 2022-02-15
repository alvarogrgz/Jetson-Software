/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Camarero Vera CERN EN/SMM/MRO 2019
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

#include "Cameras/ICamera.hpp"
#include "CommonInterfaces/IInitializable.hpp"
#include "EventLogger/EventLogger.hpp"
#include "DeviceManager/DeviceManagerWithAutoInitialization/DeviceManagerWithAutoInitialization.hpp"
namespace crf {
namespace sensors {
namespace cameras {

/**
 * @brief Camera manager class for camera communication point
 * The camera manager class ensures thread safe access to the camera.
 * It is also responsible to optimize the resources. If no communication point is requesting frames
 * the camera is deinitialized to release resources on the running controller (e.g. USB bandwidth)
 * The communication point requests a frame stream to the camera manager with a certain resolution.
 * It is also responsible of changing the camera parameters.
 */
class CameraManager : public utility::devicemanager::DeviceManagerWithAutoInitialization {
 public:
    CameraManager() = delete;
    /**
     * @brief Construct a new Camera Manager object
     * 
     * @param camera the pointer to the ICamera object. It must not be initialized
     */
    CameraManager(std::shared_ptr<ICamera> camera,
        const std::chrono::milliseconds& grabberTimeout = std::chrono::seconds(10),
        const std::chrono::milliseconds& inizializationTimeout = std::chrono::seconds(10));
    CameraManager(const CameraManager& other) = delete;
    CameraManager(CameraManager&& other) = delete;

    /**
     * @brief Destroy the Camera Manager object
     * It deinitializes the camera.
     */
    virtual ~CameraManager();

    /**
     * @brief Get the current camera parameters
     * It includes the resolution
     * @return nlohmann::json the current camera parameters
     */

    nlohmann::json getStatus() override;
    /**
     * @brief Set camera parameters
     * Set the camera parameters such as focus, zoom etc.
     * The resolution will not be affected
     * @param parameters the parameters to change
     */

    bool setStatus(const nlohmann::json& parameters);

    /**
     * @brief Requests a stream of frames with a specific resolution
     * Requests a stream of frames with a specific resolution.
     * The streamID is the identifier of the communication point. If the streamID has already an active stream
     * the new resolution is updated.
     * @param streamID the id of the communication point
     * @param resolution the desired resolution
     * @param famerate the desired framerate
     * @return true if the stream correctly started
     * @return false if the camera stream did not start
     */

    bool startFrameStream(int streamID, const cv::Size& resolution, int framerate);
    /**
     * @brief Stops a stream.
     * If there are no streams left, deinitialize camera.
     * @param streamID the id of the communication point
     */
    void stopFrameStream(int streamID);


    /**
     * @brief Get the latest camera frame
     * Get the latest camera frame. The lastFrame parameter indicates that the requested frame
     * is the last of the stream and the requested stream can be interrupted.
     * If the requesting communication point is not requesting frames for more than 5 seconds
     * its stream is automatically closed.
     * In case of error, an empty cv::Mat is returned
     * @param streamID the id of the communication point
     * @param closeStream the closeStream flag to close the stream
     * @param timeout timeout in milliseconds after which the function return an empty frame if
     *              a new frame is not received 
     * @return cv::Mat the frame.
     */
    cv::Mat getFrame(int streamID,
        const std::chrono::milliseconds& timeout = std::chrono::seconds(2));
    cv::Mat getSingleFrame(const std::chrono::milliseconds& timeout = std::chrono::seconds(2));
    
 protected:
    cv::Mat latestFrame_;
    std::atomic<cv::Size> maxRequestedResolution_;
    std::atomic<int> maxRequestedFramerate_;
    
    // This variable maps each streamID to the requested stream parameters
    // The first parameter is the resolution and the second one is the framerate.
    std::map<int, std::pair<cv::Size, int>> requestedParametersMap_;

    std::map<int,
        std::chrono::time_point<std::chrono::high_resolution_clock> > lastRequestedFrameMap_;
    const std::chrono::milliseconds streamTimeout_;

    std::atomic<bool> stopGrabber_;
    std::atomic<bool> frameGrabberRunning_;
    std::mutex threadStartedMutex_;
    std::shared_timed_mutex parametersMutex_;
    std::shared_timed_mutex frameMutex_;
    std::condition_variable_any frameCv_;
    std::condition_variable startedGrabberThreadCv_;
    std::thread grabberThread_;

    virtual void frameGrabber();
    void cleanup();
    cv::Size getMaxRequestedResolution();
    int getMaxRequestedFramerate();


 private:
    utility::logger::EventLogger logger_;
    std::shared_ptr<ICamera> camera_;
    void checkLatestRequestTime();
};

}  // namespace cameras
}  // namespace sensors
}  // namespace crf
