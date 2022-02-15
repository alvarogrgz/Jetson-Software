/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "RGBDCameras/RGBDCameraCommunicationPoint/RGBDCameraManager.hpp"
#include "VisionUtility/Image/ImageJSONConverter.hpp"

namespace crf {
namespace sensors {
namespace rgbdcameras {

RGBDCameraManager::RGBDCameraManager(std::shared_ptr<IRGBDCamera> camera,
    const std::chrono::milliseconds& grabberTimeout, const std::chrono::milliseconds& inizializationTimeout):
    CameraManager(camera, grabberTimeout, inizializationTimeout),
    maxRequestedDepthResolution_(cv::Size()),
    maxRequestedDepthFramerate_(),        
    requestedDepthParametersMap_(),
    logger_("RGBDCameraManager"),
    rgbdCamera_(camera),
    latestRGBDFrame_(),
    latestPointCloud_(new pcl::PointCloud<pcl::PointXYZRGBA>()),
    requestedPointCloudsMap_() {
    logger_->debug("CTor");
}

RGBDCameraManager::~RGBDCameraManager() {
    logger_->debug("DTor");
}

nlohmann::json RGBDCameraManager::getStatus() {
    logger_->debug("getStatus");
    lastRequestTime_ = std::chrono::high_resolution_clock::now();

    
    auto response = CameraManager::getStatus();

    //TODO: FIX THSI TO ADD THE RECURSIVE MUTEX
    std::scoped_lock<std::mutex> lock(accessMutex_);
    if (!initializeDevice()) {
        logger_->error("Cannot initialize the camera");
        throw std::runtime_error("Cannot initialize the camera");
    }

    auto colorIntrinsic = rgbdCamera_->getColorCameraMatrix();
    if (colorIntrinsic) {
        response["color_intrinsic"] = colorIntrinsic.get();
    }
    auto colorDirstortion = rgbdCamera_->getColorDistortionMatrix();
    if (colorDirstortion) {
        response["color_distortion"] = colorDirstortion.get();
    }

    auto depthIntrinsic = rgbdCamera_->getDepthCameraMatrix();
    if (depthIntrinsic) {
        response["depth_intrinsic"] = depthIntrinsic.get();
    }

    auto depthDistortion = rgbdCamera_->getDepthDistortionMatrix();
    if (depthDistortion) {
        response["depth_distortion"] = depthDistortion.get();
    }

    auto extrinsic = rgbdCamera_->getDepth2ColorExtrinsics();
    if (extrinsic) {
        response["extrinsic"] = extrinsic.get();
    }

    auto availableDepthResolutions = rgbdCamera_->getAvailableResolutions();
    int index = 0;
    for (auto &res : availableDepthResolutions) {
        auto availableFramerates = rgbdCamera_->getAvailableDepthFramerates(res);
        nlohmann::json profile;
        profile["resolution"] = { res.width, res.height };
        profile["framerates"] = availableFramerates;
        response["depth_profiles"][std::to_string(index)] = profile;
        index++;
    }
    return response;
}

cv::Mat RGBDCameraManager::getFrame(int streamID, const std::chrono::milliseconds& timeout) {
    lastRequestTime_ = std::chrono::high_resolution_clock::now();
    cv::Mat frame;
    cv::Size resolution;

    if (!frameGrabberRunning_) {
        logger_->warn("Frame grabber is not running");
        throw std::runtime_error( "Frame grabber is not running" );
    }
    {
        // Thread safe access to the resolution map the shared_lock allows multiple readers to
        // access at the same time.
        std::shared_lock<std::shared_timed_mutex> lock(parametersMutex_);
        if (requestedParametersMap_.find(streamID) == requestedParametersMap_.end()) {
            logger_->warn("streamID {} not registered", streamID);
            throw std::runtime_error( "Stream is not registered in manager" );
        }
        resolution = requestedParametersMap_[streamID].first;
    }

    {
        // Waits for the next frame coming from the grabber. It waits for 5 time the framerate.
        std::shared_lock<std::shared_timed_mutex> lock(frameMutex_);
        if (frameCv_.wait_for(lock, std::chrono::milliseconds(timeout)) == std::cv_status::timeout) { // NOLINT
            logger_->warn("Timeout while waiting for frame on socket {}", streamID);
                return cv::Mat();
        }
        frame = latestRGBDFrame_.image;
        lastRequestedFrameMap_[streamID] = std::chrono::high_resolution_clock::now();
    }

    // Resize frame if needed.
    if ((frame.cols != resolution.width) || (frame.rows != resolution.height)) {
        cv::resize(frame, frame, resolution);
    }

    return frame;
}

cv::rgbd::RgbdFrame RGBDCameraManager::getRGBDFrame(int streamID, const std::chrono::milliseconds& timeout) {
    lastRequestTime_ = std::chrono::high_resolution_clock::now();

    if (!frameGrabberRunning_) {
        logger_->warn("Frame grabber is not running");
        return cv::rgbd::RgbdFrame();
    }
    cv::Size resolution;
    {
        // Thread safe access to the resolution map the shared_lock allows multiple readers to
        // access at the same time.
        std::shared_lock<std::shared_timed_mutex> lock(parametersMutex_);
        if (requestedParametersMap_.find(streamID) == requestedParametersMap_.end()) {
            logger_->warn("streamID {} not registered", streamID);
            return cv::rgbd::RgbdFrame();
        }
        resolution = requestedParametersMap_[streamID].first;
    }

    cv::rgbd::RgbdFrame frame;
    {
        // Waits for the next frame coming from the grabber. It waits for 5 time the framerate.
        std::shared_lock<std::shared_timed_mutex> lock(frameMutex_);
        if (frameCv_.wait_for(lock, std::chrono::milliseconds(timeout)) == std::cv_status::timeout) { // NOLINT
            logger_->warn("Timeout while waiting for frame on socket {}", streamID);
            return cv::rgbd::RgbdFrame();
        }
        frame = latestRGBDFrame_;
        lastRequestedFrameMap_[streamID] = std::chrono::high_resolution_clock::now();
    }
    return frame;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr RGBDCameraManager::getPointCloud(int streamID, bool alignFrames,  const std::chrono::milliseconds& timeout) {
    lastRequestTime_ = std::chrono::high_resolution_clock::now();
    std::shared_lock<std::shared_timed_mutex> lock(frameMutex_);
    if (frameCv_.wait_for(lock, std::chrono::milliseconds(timeout)) == std::cv_status::timeout) { // NOLINT
        logger_->warn("Timeout while waiting for frame on socket {}", streamID);
        return pcl::PointCloud<pcl::PointXYZRGBA>::Ptr();
    }
    lastRequestedFrameMap_[streamID] = std::chrono::high_resolution_clock::now();
    return latestPointCloud_;
}

bool RGBDCameraManager::startFrameStream(int streamID, const cv::Size& resolution, int framerate){
    throw std::runtime_error( "RGBDCameraManager::startFrameStream(int streamID, const cv::Size& resolution, int framerate) is unsupported\n\n Use: RGBDCameraManager::startFrameStream(int streamID, bool RGBDStream, bool PointCloudStream, const cv::Size& resolution, int framerate, const cv::Size& depthResolution, int depthFramerate) instead" );
}

bool RGBDCameraManager::startFrameStream(int streamID, bool RGBDStream, bool PointCloudStream, const cv::Size& resolution, int framerate, const cv::Size& depthResolution, int depthFramerate) {
    logger_->debug("startFrameStream()");
    lastRequestTime_ = std::chrono::high_resolution_clock::now();

    if(RGBDStream){
        if ((resolution.width == 0) || (resolution.height == 0) ||
            (depthResolution.width == 0) || (depthResolution.height == 0) ) {
        logger_->error("Can't request a resolution with a dimension equal to 0");
        throw std::invalid_argument( "Can't request a resolution with a dimension equal to 0" );
        }
        if (framerate<=0 || depthFramerate<=0){
        logger_->error("Can't request a framerate lower or equal to 0");
        throw std::invalid_argument("Can't request a framerate lower or equal to 0");
        }
    }

    {
        std::scoped_lock<std::mutex> lock(accessMutex_);
        if (!initializeDevice()) {
                logger_->error("Cannot initialize the camera");
                throw std::runtime_error("Cannot initialize the camera");
        }

        // Check that the resolution and framerate is supported by the camera
        std::vector<cv::Size> availableResolutions = rgbdCamera_->getAvailableResolutions();
        if(std::find(availableResolutions.begin(), availableResolutions.end(), resolution)
            == availableResolutions.end()) {
            logger_->error("Requested resolution is not among available resolutions");
            throw std::invalid_argument( "Requested resolution is not among available resolutions" );
        }

        std::vector<int> availableFramerates = rgbdCamera_->getAvailableFramerates(resolution);
        if(std::find(availableFramerates.begin(), availableFramerates.end(), framerate)
            == availableFramerates.end()) {
            logger_->error("Requested framerate is not valid for the specified resolution");
            throw std::invalid_argument("Requested framerate is not valid for the specified resolution");
        }

        std::vector<cv::Size> availableDepthResolutions = rgbdCamera_->getAvailableDepthResolutions();
        if(std::find(availableDepthResolutions.begin(), availableDepthResolutions.end(), depthResolution)
            == availableDepthResolutions.end()) {
            logger_->error("Requested depth resolution is not among available resolutions");
            throw std::invalid_argument( "Requested depth resolution is not among available resolutions" );
        }

        std::vector<int> availabledepthFramerates = rgbdCamera_->getAvailableDepthFramerates(depthResolution);
        if(std::find(availabledepthFramerates.begin(), availabledepthFramerates.end(), depthFramerate)
            == availabledepthFramerates.end()) {
            logger_->error("Requested framerate is not valid for the specified resolution");
            throw std::invalid_argument("Requested framerate is not valid for the specified resolution");
        }
    } //end device mutex

    {
        //Update the request parameters map (resolution, framerate, depthresolution, depthframerate)
        std::unique_lock<std::shared_timed_mutex> lock(parametersMutex_);
        if (requestedParametersMap_.find(streamID) == requestedParametersMap_.end()) {
            requestedParametersMap_.insert({streamID, std::pair(resolution, framerate)});
            requestedDepthParametersMap_.insert({streamID, std::pair(depthResolution, depthFramerate)});
        } else {
            requestedParametersMap_[streamID] = std::pair(resolution, framerate);
            requestedDepthParametersMap_[streamID] = std::pair(depthResolution, depthFramerate);
        }
        lastRequestedFrameMap_[streamID] = std::chrono::high_resolution_clock::now();
    }

    //Add the Streaming request for RGBD
    {
        std::scoped_lock<std::shared_timed_mutex> lock(requestedRGBDFramesMapMutex_);
        if (requestedRGBDFramesMap_.find(streamID) == requestedRGBDFramesMap_.end()) {
            requestedRGBDFramesMap_.insert({streamID, RGBDStream});
        } else {
            requestedRGBDFramesMap_[streamID] = RGBDStream;
        }
    }
    
    //Add the Streaming request for PointCloud
    {
        std::scoped_lock<std::shared_timed_mutex> lock(requestedPointCloudsMapMutex_);
        if (requestedPointCloudsMap_.find(streamID) == requestedPointCloudsMap_.end()) {
            requestedPointCloudsMap_.insert({streamID, PointCloudStream});
        } else {
            requestedPointCloudsMap_[streamID] = PointCloudStream;
        }
    }

    //Get the max streaming paramaters
    maxRequestedResolution_ = getMaxRequestedResolution();
    maxRequestedFramerate_= getMaxRequestedFramerate();
    maxRequestedDepthResolution_ = getMaxRequestedDepthResolution();
    maxRequestedDepthFramerate_= getMaxRequestedDepthFramerate();

    //If frameGrabber is not running start it
    std::unique_lock<std::mutex> lock(threadStartedMutex_);
    if(!frameGrabberRunning_){
        if (grabberThread_.joinable()) {
            grabberThread_.join();
        }
        logger_->debug("Starting new frame grabber thread");
        frameGrabberRunning_ = true;
        stopGrabber_ = false;
        grabberThread_ = std::thread(&RGBDCameraManager::frameGrabber, this);
        if (streamTimeout_.count() == 0) {
            startedGrabberThreadCv_.wait(lock);
        } else {
            if (!startedGrabberThreadCv_.wait_for(lock, streamTimeout_, [this]() {
                return true;})) {
                frameGrabberRunning_ = false;
                logger_->error("Failed to start grabber thread");
                throw std::runtime_error("Failed to start grabber thread");
                return false;
            }
        }
    }
    return true;
}

void RGBDCameraManager::frameGrabber() {
    logger_->debug("frameGrabber()");
    cv::Size currentResolution;
    cv::Size currentDepthResolution;
    int currentFramerate;
    int currentDepthFramerate;
    frameGrabberRunning_ = true;

    // Sets the resolution with requested camera resolution and framerate
    // then notifies that the startup was completed.
    if (!rgbdCamera_->setResolution(maxRequestedResolution_)) {
        logger_->error("Set resolution failed");
        startedGrabberThreadCv_.notify_all();
        frameGrabberRunning_ = false;
        return;
    }
    auto resolution = rgbdCamera_->getResolution();

    if (!rgbdCamera_->setFramerate(maxRequestedFramerate_)) {
        logger_->error("Set framerate failed");
        startedGrabberThreadCv_.notify_all();
        frameGrabberRunning_ = false;
        return;
    }
    auto framerate = rgbdCamera_->getFramerate();

    if (!rgbdCamera_->setDepthResolution(maxRequestedDepthResolution_)) {
        logger_->error("Set resolution failed");
        startedGrabberThreadCv_.notify_all();
        frameGrabberRunning_ = false;
        return;
    }
    auto depthResolution = rgbdCamera_->getDepthResolution();

    if (!rgbdCamera_->setDepthFramerate(maxRequestedDepthFramerate_)) {
        logger_->error("Set framerate failed");
        startedGrabberThreadCv_.notify_all();
        frameGrabberRunning_ = false;
        return;
    }
    auto depthFramerate = rgbdCamera_->getDepthFramerate();

    if (!resolution || !depthResolution || !framerate || !depthFramerate) {
        logger_->error("Error getting resolution or framerate");
        startedGrabberThreadCv_.notify_all();
        frameGrabberRunning_ = false;
        return;
    }

    currentResolution = resolution.get();
    currentFramerate = framerate.get();
    currentDepthResolution = depthResolution.get();
    currentDepthFramerate = depthFramerate.get();
    startedGrabberThreadCv_.notify_all();

    while (!stopGrabber_) {
        {
            std::scoped_lock<std::mutex> lock(accessMutex_);
                if (!initializeDevice()) {
                    logger_->error("frameGrabber(): Cannot initialize the camera");
                    break;
                }

            // If the requested resolution has changed check if there is a better resolution for the
            // camera to satisfy all the requests.
            if (currentResolution != maxRequestedResolution_.load()) {
                if (!rgbdCamera_->setResolution(maxRequestedResolution_.load())) {
                    logger_->warn("Set resolution failed");
                    break;
                }
                auto resolution = rgbdCamera_->getResolution();
                if (!resolution) {
                    logger_->warn("Failed to get resolution");
                    break;
                }
                currentResolution = resolution.get();
            }

            if (currentDepthResolution != maxRequestedDepthResolution_.load()) {
                if (!rgbdCamera_->setDepthResolution(maxRequestedDepthResolution_.load())) {
                    logger_->warn("Set depth resolution failed");
                    break;
                }
                auto depthResolution = rgbdCamera_->getDepthResolution();
                if (!depthResolution) {
                    logger_->warn("Failed to get depth resolution");
                    break;
                }
                currentDepthResolution = depthResolution.get();
            }

            // Same for the framerate
            if (currentFramerate != maxRequestedFramerate_.load()) {
                if(!rgbdCamera_->setFramerate(maxRequestedFramerate_.load())){
                        logger_->warn("Set framerate failed");
                        break;
                }
                auto framerate = rgbdCamera_->getFramerate();
                if (!framerate) {
                    logger_->warn("Failed to get framerate");
                    break;
                }
                currentFramerate = framerate.get();
            }

            if (currentDepthFramerate != maxRequestedDepthFramerate_.load()) {
                if(!rgbdCamera_->setDepthFramerate(maxRequestedDepthFramerate_.load())){
                        logger_->warn("Set depth framerate failed");
                        break;
                }
                auto depthFramerate = rgbdCamera_->getDepthFramerate();
                if (!depthFramerate) {
                    logger_->warn("Failed to get depth framerate");
                    break;
                }
                currentDepthFramerate = depthFramerate.get();
            }
            // Gets the camera frame but it updates it only if it's a good one. The getFrame method is
            // a blocking call until a new frame arrives.
            if (isPointCloudRequested()) {
                auto frame = rgbdCamera_->getPointCloud(true);
                if (frame) {
                    std::scoped_lock<std::shared_timed_mutex> lock(frameMutex_);
                    latestRGBDFrame_ = frame->second;
                    latestPointCloud_ = frame->first;
                    frameCv_.notify_all();
                }
            }
            else if (isRGBDFrameRequested()) {
                auto frame = rgbdCamera_->getRgbdFrame();
                if (!frame.image.empty()) {
                    std::scoped_lock<std::shared_timed_mutex> lock(frameMutex_);
                    latestRGBDFrame_ = frame;
                    latestPointCloud_->clear();
                    frameCv_.notify_all();
                }
            }
            else {
                logger_->warn("No RGBDFrame or Pointcloud requested");
                latestRGBDFrame_.release();
                latestPointCloud_->clear();
                frameCv_.notify_all();
                continue;
            }
        } //device mutex

        // Cleanup stale streams
        if (streamTimeout_.count() != 0) {
            cleanup();
        }
    }
    
    logger_->warn("Stopping frameGrabber");
    frameGrabberRunning_ = false;
    rgbdCamera_->deinitialize();
}

void RGBDCameraManager::stopFrameStream(int streamID) {
    logger_->debug("stopFrameStream()");
    {
        // Removes the stream from the requested streams.
        std::scoped_lock<std::shared_timed_mutex> lock(parametersMutex_);
        requestedParametersMap_.erase(streamID);
        requestedDepthParametersMap_.erase(streamID);
        lastRequestedFrameMap_.erase(streamID);
    }

    {
        std::scoped_lock<std::shared_timed_mutex> lock(requestedPointCloudsMapMutex_);
        if (requestedPointCloudsMap_.find(streamID) != requestedPointCloudsMap_.end()) {
            requestedPointCloudsMap_.erase(streamID);
        }
    }
    {
        std::scoped_lock<std::shared_timed_mutex> lock(requestedRGBDFramesMapMutex_);
            if (requestedRGBDFramesMap_.find(streamID) != requestedRGBDFramesMap_.end()) {
        requestedRGBDFramesMap_.erase(streamID);
        }
    }

    {
        std::scoped_lock<std::shared_timed_mutex> lock(parametersMutex_);
        // If it's the last stream left, it also closes the grabber.
        if (requestedParametersMap_.size() == 0) {
            logger_->info("No more requested frames, going to close the grabber");
            frameGrabberRunning_ = false;
            stopGrabber_ = true;
            return;
        }

        // Otherwise it checks which is the maximum resolution requested and it changes it
        maxRequestedResolution_ = getMaxRequestedResolution();
        maxRequestedFramerate_= getMaxRequestedFramerate();
        maxRequestedDepthResolution_ = getMaxRequestedDepthResolution();
        maxRequestedDepthFramerate_= getMaxRequestedDepthFramerate();
    }
}

cv::Size RGBDCameraManager::getMaxRequestedDepthResolution() {
    cv::Size maxResolutionRequested(0, 0);
    {
        std::shared_lock<std::shared_timed_mutex> lock(parametersMutex_);
        for (auto it = requestedDepthParametersMap_.begin(); it != requestedDepthParametersMap_.end(); ++it) {
            cv::Size resolution = it->second.first;
            if ((resolution.width * resolution.height) >
                (maxResolutionRequested.width * maxResolutionRequested.height)) {
                    maxResolutionRequested = resolution;
            }
        }
    }
    return maxResolutionRequested;
}

int RGBDCameraManager::getMaxRequestedDepthFramerate() {
    int maxFramerate = 0;
    {
        std::shared_lock<std::shared_timed_mutex> lock(parametersMutex_);
        for (auto it = requestedDepthParametersMap_.begin(); it != requestedDepthParametersMap_.end(); ++it) {
            int framerate = it->second.second;
            if (framerate > maxFramerate)
                maxFramerate = framerate;
            }
        }
    return maxFramerate;
}


bool RGBDCameraManager::isRGBDFrameRequested() {
    std::scoped_lock<std::shared_timed_mutex> lock(requestedRGBDFramesMapMutex_);
    for (auto streamRGBDRequest : requestedRGBDFramesMap_) {
        if (streamRGBDRequest.second) {
            return true;
        }
    }
    return false;
}

bool RGBDCameraManager::isPointCloudRequested() {
    std::scoped_lock<std::shared_timed_mutex> lock(requestedPointCloudsMapMutex_);
    for (auto streamPointCloudRequest : requestedPointCloudsMap_) {
        if (streamPointCloudRequest.second) {
            return true;
        }
    }
    return false;
}

}  // namespace rgbdcameras
}  // namespace sensors
}  // namespace crf
