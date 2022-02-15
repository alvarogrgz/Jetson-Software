/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Camarero Vera CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>

#include <opencv2/core.hpp>

#include "Cameras/CameraCommunicationPoint/CameraManager.hpp"

namespace crf {
namespace sensors {
namespace cameras {

CameraManager::CameraManager(std::shared_ptr<ICamera> camera,
    const std::chrono::milliseconds& streamTimeout, const std::chrono::milliseconds& inizializationTimeout) :
        DeviceManagerWithAutoInitialization(camera, inizializationTimeout),
        latestFrame_(),
        maxRequestedResolution_(cv::Size()),
        maxRequestedFramerate_(),        
        requestedParametersMap_(),
        lastRequestedFrameMap_(),
        streamTimeout_(streamTimeout),
        stopGrabber_(false),
        frameGrabberRunning_(false),
        threadStartedMutex_(),        
        parametersMutex_(),
        frameMutex_(),
        frameCv_(),
        startedGrabberThreadCv_(),
        grabberThread_(),
        logger_("CameraManager"),
        camera_(camera){
            logger_->debug("CTor");
}

CameraManager::~CameraManager() {
    logger_->debug("DTor");
    // Check if the frame grabber is still running, then stop it.
    if (frameGrabberRunning_) {
        stopGrabber_ = true;
    }
    // Join the grabber thread. Avoids segmentation fault
    if (grabberThread_.joinable()) {
        grabberThread_.join();
    }
}

nlohmann::json CameraManager::getStatus() {
    logger_->debug("getStatus");
    nlohmann::json statusJSON;
    lastRequestTime_ = std::chrono::high_resolution_clock::now();

    boost::optional<cv::Size> resolution;
    boost::optional<int> fps;
    boost::optional<float> zoom;
    boost::optional<float> pan;
    boost::optional<float> tilt;
    boost::optional<float> exposure;
    boost::optional<float> shutter;
    boost::optional<crf::sensors::cameras::ICamera::FocusModes> focusmode;
    boost::optional<float> focus;
    boost::optional<int> iso;

    { // request parameters
        std::scoped_lock<std::mutex> lock(accessMutex_);
        if (initializeDevice()) {
            statusJSON["status"] = "initialize";
        } else {
            statusJSON["status"] = "deinitialize";
            return statusJSON;
        }

        resolution = camera_->getResolution();
        fps = camera_->getFramerate();
        zoom = camera_->getZoom();
        pan = camera_->getPan();
        tilt = camera_->getTilt();
        exposure = camera_->getExposure();
        shutter = camera_->getShutterSpeed();
        focusmode = camera_->getFocusMode();
        focus = camera_->getFocus();
        iso = camera_->getISO();

        auto availableResolutions = camera_->getAvailableResolutions();
        int index = 0;
        for (auto &res : availableResolutions) {
            auto availableFramerates = camera_->getAvailableFramerates(res);
            nlohmann::json profile;
            profile["resolution"] = { res.width, res.height };
            profile["framerates"] = availableFramerates;
            statusJSON["profiles"][std::to_string(index)] = profile;
            index++;
        }
    }

    if (resolution) {
        statusJSON["resolution"]["width"] = resolution.get().width;
        statusJSON["resolution"]["height"] = resolution.get().height;
    }
    if (fps) {
        statusJSON["fps"] = fps.get();
    }
    if (zoom) {
        statusJSON["zoom"] = zoom.get();
    }
    if (pan && tilt) {
        std::vector<float> position({pan.get(), tilt.get()});
        statusJSON["position"] = position;
    }
    if (exposure) {
        statusJSON["exposure"] = exposure.get();
    }
    if (shutter) {
        statusJSON["shutter"] = shutter.get();
    }
    if (focusmode) {
        statusJSON["focusmode"] = focusmode.get() == ICamera::FocusModes::Auto ? "auto" : "manual";
    }
    if (focus) {
        statusJSON["focus"] = focus.get();
    }
    if (iso) {
        statusJSON["iso"] = iso.get();
    }

    return statusJSON;
}

bool CameraManager::setStatus(const nlohmann::json& parameters) {
    logger_->debug("setStatus()");
    lastRequestTime_ = std::chrono::high_resolution_clock::now();

    std::scoped_lock<std::mutex> lock(accessMutex_);
    if (!initializeDevice()) {
        logger_->error("Cannot initialize the camera");
        return false;
    }

    bool allSuccess = true;
    try {
        if (parameters.contains("zoom")){
            auto value = parameters["zoom"].get<float>();
            allSuccess = allSuccess && camera_->setZoom(value);
        }
        if (parameters.contains("position")) {
            auto value = parameters["position"].get<std::vector<float> >();
            if (value.size() != 2) {
                logger_->warn("Requested position vector size wrong");
                allSuccess = false;
            }
            allSuccess = allSuccess && camera_->setPosition(value[0], value[1]);
        }
        if (parameters.contains("exposure")) {
            auto value = parameters["exposure"].get<float>();
            allSuccess = allSuccess && camera_->setExposure(value);
        }
        if (parameters.contains("shutter")) {
            auto value = parameters["shutter"].get<float>();
            allSuccess = allSuccess && camera_->setShutterSpeed(value);
        }
        if (parameters.contains("focusmode")) {
            auto value = parameters["focusmode"].get<std::string>();
            if (value == "auto") {
                allSuccess = allSuccess && camera_->setFocusMode(ICamera::FocusModes::Auto);
            } else if (value == "manual") {
                allSuccess = allSuccess && camera_->setFocusMode(ICamera::FocusModes::Manual);
            }
        }
        if (parameters.contains("shutter")) {
            auto value = parameters["shutter"].get<float>();
            allSuccess = allSuccess && camera_->setShutterSpeed(value);
        }
        if (parameters.contains("focusmode")) {
            auto value = parameters["focusmode"].get<std::string>();
            if (value == "auto") {
                allSuccess = allSuccess && camera_->setFocusMode(ICamera::FocusModes::Auto);
            } else if (value == "manual") {
                allSuccess = allSuccess && camera_->setFocusMode(ICamera::FocusModes::Manual);
            }
        }
        if (parameters.contains("focus")) {
            auto value = parameters["focus"].get<float>();
            allSuccess = allSuccess && camera_->setFocus(value);
        }
        if (parameters.contains("iso")) {
            auto value = parameters["iso"].get<float>();
            allSuccess = allSuccess && camera_->setISO(value);
        }
    } catch (const std::exception& ex) {
        logger_->warn("Failed to parse request: {}", ex.what());
        allSuccess = false;
    }
    return allSuccess;
}

bool CameraManager::startFrameStream(int streamID, const cv::Size& resolution, int framerate) {
    logger_->debug("startFrameStream()");
    lastRequestTime_ = std::chrono::high_resolution_clock::now();

    if ((resolution.width == 0) || (resolution.height == 0)) {
        logger_->error("Can't request a resolution with a dimension equal to 0");
        throw std::invalid_argument( "Can't request a resolution with a dimension equal to 0" );
    }

    if (framerate<=0){
        logger_->error("Can't request a framerate lower or equal to 0");
        throw std::invalid_argument("Can't request a framerate lower or equal to 0");
    }
    
    {
        std::scoped_lock<std::mutex> lock(accessMutex_);
        if (!initializeDevice()) {
                logger_->error("Cannot initialize the camera");
                throw std::runtime_error("Cannot initialize the camera");
        }

        // Check that the resolution and framerate is supported by the camera
        std::vector<cv::Size> availableResolutions = camera_->getAvailableResolutions();
        if(std::find(availableResolutions.begin(), availableResolutions.end(), resolution)
            == availableResolutions.end()) {
            logger_->error("Requested resolution is not among available resolutions");
            throw std::invalid_argument( "Requested resolution is not among available resolutions" );
        }

        std::vector<int> availableFramerates = camera_->getAvailableFramerates(resolution);
        if(std::find(availableFramerates.begin(), availableFramerates.end(), framerate)
            == availableFramerates.end()) {
            logger_->error("Requested framerate is not valid for the specified resolution");
            throw std::invalid_argument("Requested framerate is not valid for the specified resolution");
            }
    } //end device mutex
    
    { // Update the request parameters map (resolution, framerate)
        std::scoped_lock<std::shared_timed_mutex> lock(parametersMutex_);
        if (requestedParametersMap_.find(streamID) == requestedParametersMap_.end()) {
            requestedParametersMap_.insert({streamID, std::pair(resolution, framerate)});
        } else {
            requestedParametersMap_[streamID] = std::pair(resolution, framerate);
        }
        lastRequestedFrameMap_[streamID] = std::chrono::high_resolution_clock::now();
    }
    maxRequestedResolution_ = getMaxRequestedResolution();
    maxRequestedFramerate_= getMaxRequestedFramerate();

    //If frameGrabber is not running start it
    std::unique_lock<std::mutex> lock(threadStartedMutex_);
    if(!frameGrabberRunning_){
        if (grabberThread_.joinable()) {
            grabberThread_.join();
        }
        frameGrabberRunning_ = true;
        stopGrabber_ = false;
        logger_->debug("Starting new frame grabber thread");
        grabberThread_ = std::thread(&CameraManager::frameGrabber, this);
        if (streamTimeout_.count() == 0) {
            startedGrabberThreadCv_.wait(lock);
        } else { // Wait for timeout: If thread has started return true, exception otherwise.
            if (!startedGrabberThreadCv_.wait_for(lock, streamTimeout_, [this]() {return true;})) {
                frameGrabberRunning_ = false;
                logger_->error("Failed to start grabber thread");
                throw std::runtime_error("Failed to start grabber thread");
            }
        }
    }
    return true;
}

cv::Mat CameraManager::getFrame(
    int streamID, const std::chrono::milliseconds& timeout) {
        lastRequestTime_ = std::chrono::high_resolution_clock::now();
        cv::Size resolution;
        cv::Mat frame;

        if (!frameGrabberRunning_) {
            logger_->error("Frame grabber is not running");
            throw std::runtime_error( "Frame grabber is not running" );
        }

        { // Get resolution and check if Stream is registered.
            std::shared_lock<std::shared_timed_mutex> lock(parametersMutex_);
            if (requestedParametersMap_.find(streamID) == requestedParametersMap_.end()) {
                logger_->error("StreamID {} not registered", streamID);
                throw std::runtime_error( "Stream is not registered in manager" );
            }
            resolution = requestedParametersMap_[streamID].first;
        }


        { // Waits for the next frame coming from the grabber.
            std::shared_lock<std::shared_timed_mutex> lock(frameMutex_);
            if (frameCv_.wait_for(lock, std::chrono::milliseconds(timeout)) == std::cv_status::timeout) { // NOLINT
                logger_->error("Timeout while waiting for frame on socket {}", streamID);
                return cv::Mat();
            }
            frame = latestFrame_;
            lastRequestedFrameMap_[streamID] = std::chrono::high_resolution_clock::now();
        }

        // Resize frame if needed.
        if ((frame.cols != resolution.width) || (frame.rows != resolution.height)) {
            cv::resize(frame, frame, resolution);
        }
        return frame;
}

cv::Mat CameraManager::getSingleFrame(const std::chrono::milliseconds& timeout){
    logger_->debug("getSingleFrame()");
    lastRequestTime_ = std::chrono::high_resolution_clock::now();

    if(frameGrabberRunning_){
        std::shared_lock<std::shared_timed_mutex> lock(frameMutex_);
        if (frameCv_.wait_for(lock, std::chrono::milliseconds(timeout)) == std::cv_status::timeout) { // NOLINT
            logger_->error("Timeout while waiting for frame");
            return cv::Mat();
        }
        return latestFrame_;
    }

    std::scoped_lock<std::mutex> lock(accessMutex_);
    if (!initializeDevice()) {
        logger_->error("Cannot initialize the camera");
        return cv::Mat();
    }

    return camera_->getFrame();
}

void CameraManager::frameGrabber() {
    logger_->debug("frameGrabber()");
    cv::Size currentResolution;
    int currentFramerate;

    boost::optional<cv::Size> resolution;
    boost::optional<int> framerate;
    // Sets the resolution with requested camera resolution and framerate then notifies that the startup was completed.
    {
        std::scoped_lock<std::mutex> lock(accessMutex_);
        if (!camera_->setResolution(maxRequestedResolution_)) {
            logger_->error("Set resolution failed");
            frameGrabberRunning_ = false;
            startedGrabberThreadCv_.notify_all();
            return;
        }
        resolution = camera_->getResolution();

        if (!camera_->setFramerate(maxRequestedFramerate_)) {
            logger_->error("Set framerate failed");
            frameGrabberRunning_ = false;
            startedGrabberThreadCv_.notify_all();
            return;
        }
        framerate = camera_->getFramerate();
    }

    if (!resolution || !framerate) {
        logger_->error("Error getting resolution or framerate");
        frameGrabberRunning_ = false;
        startedGrabberThreadCv_.notify_all();
        return;
    }

    currentResolution = resolution.get();
    currentFramerate = framerate.get();
    startedGrabberThreadCv_.notify_all();

    cv::Mat frame;
    while (!stopGrabber_) {
        {
            std::scoped_lock<std::mutex> lock(accessMutex_);
            if (!initializeDevice()) {
                logger_->error("frameGrabber(): Cannot initialize the camera");
                break;
            }

            // If the requested resolution has changed check if there is a better resolution for the
            // camera to satisfy all the requests
            if (currentResolution != maxRequestedResolution_.load()) {
                if (!camera_->setResolution(maxRequestedResolution_.load())) {
                    logger_->warn("Set resolution failed");
                    break;
                }
                auto resolution = camera_->getResolution();
                if (!resolution) {
                    logger_->warn("Failed to get resolution");
                    break;
                }
                currentResolution = resolution.get();
            }

            // Same for the framerate
            if (currentFramerate != maxRequestedFramerate_.load()) {
                if(!camera_->setFramerate(maxRequestedFramerate_.load())){
                        logger_->warn("Set framerate failed");
                        break;
                }
                auto framerate = camera_->getFramerate();
                if (!framerate) {
                    logger_->warn("Failed to get framerate");
                    break;
                }
                currentFramerate = framerate.get();
            }

            // Gets the camera frame but it updates it only if it's a good one. The getFrame method is
            // a blocking call until a new frame arrives.
        frame = camera_->getFrame();
        } //device mutex

        if (!frame.empty()) {
            std::scoped_lock<std::shared_timed_mutex> lock(frameMutex_);
            latestFrame_ = frame;
            frameCv_.notify_all();
        } 

    if (streamTimeout_.count() != 0)
        cleanup();
    } //end while

    logger_->debug("Stopping frameGrabber");
    frameGrabberRunning_ = false;
    //Delete all entries from the map
    std::unique_lock<std::shared_timed_mutex> lock(parametersMutex_);
    requestedParametersMap_.clear();
    lastRequestedFrameMap_.clear();
}


void CameraManager::cleanup() {
    std::vector<int> timeoutStreams;
    {
        std::scoped_lock<std::shared_timed_mutex> lock(parametersMutex_);
        auto now = std::chrono::high_resolution_clock::now();
        for (auto it = lastRequestedFrameMap_.begin(); it != lastRequestedFrameMap_.end(); ++it) {
            if ((now - it->second) > streamTimeout_) {
                logger_->info("Timeout of socket {}, going to close it", it->first);
                timeoutStreams.push_back(it->first);
            }
        }
    }
    for(int i: timeoutStreams)
        stopFrameStream(i);
}

void CameraManager::stopFrameStream(int streamID) {
    logger_->debug("stopStream()");
    {
        // Removes the stream from the requested streams.
        std::unique_lock<std::shared_timed_mutex> lock(parametersMutex_);
        requestedParametersMap_.erase(streamID);
        lastRequestedFrameMap_.erase(streamID);
        
        // If it's the last stream left, it also closes the grabber.
        if (requestedParametersMap_.size() == 0) {
            logger_->info("No more requested frames, going to close the grabber");
            frameGrabberRunning_ = false;
            stopGrabber_ = true;
            return;
        }
    }
    // Otherwise it checks which is the maximum resolution requested and it changes it.
    maxRequestedResolution_ = getMaxRequestedResolution();
    maxRequestedFramerate_= getMaxRequestedFramerate();
}

cv::Size CameraManager::getMaxRequestedResolution() {
    cv::Size maxResolutionRequested(0, 0);
    std::shared_lock<std::shared_timed_mutex> lock(parametersMutex_);
    for (auto it = requestedParametersMap_.begin(); it != requestedParametersMap_.end(); ++it) {
        cv::Size resolution = it->second.first;
        if ((resolution.width * resolution.height) >
            (maxResolutionRequested.width * maxResolutionRequested.height)) {
                maxResolutionRequested = resolution;
        }
    }
    return maxResolutionRequested;
}

int CameraManager::getMaxRequestedFramerate() {
    int maxFramerate = 0;
    std::shared_lock<std::shared_timed_mutex> lock(parametersMutex_);
    for (auto it = requestedParametersMap_.begin(); it != requestedParametersMap_.end(); ++it) {
        int framerate = it->second.second;
        if (framerate > maxFramerate)
            maxFramerate = framerate;
    }
    return maxFramerate;
}

}   // namespace cameras
}   // namespace sensors
}   // namespace crf
