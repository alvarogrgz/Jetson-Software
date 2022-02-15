/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author:  Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <condition_variable>
#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>

#include "Cameras/AxisCamera/AxisCamera.hpp"

#define TIMEOUT_FRAMERATE_FACTOR 10

namespace crf {
namespace sensors {
namespace cameras {

AxisCamera::AxisCamera(const std::string& http_address,
    const std::string& username,
    const std::string& password) :
        logger_("AxisCamera"),
        cameraAddress_(http_address),
        username_(username),
        password_(password),
        streamOn_(false),
        initialized_(false),
        frameMutex_(),
        frameCv_(),
        lastFrameTime_(),
        latestFrameSize_(0),
        latestFrame_(),
        currentResolution_(0, 0),
        currentFramerate_(15) {
            logger_->debug("CTor()");
            cameraCompleteAddress_ = "http://";
            if ((username_ != "") && (password_ != "")) {
                cameraCompleteAddress_.append(username_);
                cameraCompleteAddress_.append(":");
                cameraCompleteAddress_.append(password_);
                cameraCompleteAddress_.append("@");
            }
            cameraCompleteAddress_.append(cameraAddress_);
}

bool AxisCamera::initialize() {
    logger_->debug("initialize()");

    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }

    curlpp::initialize();
    std::string response;
    try {
        response =
            httpRequest("/axis-cgi/param.cgi?action=list&group=root.Properties.API.HTTP.Version");
    } catch (const std::exception& ex) {
        logger_->error("Could not connect to camera: {}", ex.what());
        return false;
    }

    if (response.find("root.Properties.API.HTTP.Version=3") == std::string::npos) {
        logger_->error("Requested camera is not of the correct version: {}", response);
        return false;
    }

    try {
        response =
            httpRequest("/axis-cgi/imagesize.cgi?camera=1");
    } catch (const std::exception& ex) {
        logger_->error("Could not connect to camera: {}", ex.what());
        return false;
    }
    auto resVec = split(response, '\n');
    currentResolution_ = cv::Size(
        std::stoi(resVec[0].substr(resVec[0].find('=')+1)),
        std::stoi(resVec[1].substr(resVec[1].find('=')+1)));

    initialized_ = true;

    if (!startFrameStream()) {
        initialized_ = false;
        return false;
    }

    logger_->info("Initialized");
    return true;
}

bool AxisCamera::deinitialize() {
    logger_->debug("deinitialize()");

    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    curlpp::terminate();
    if (streamOn_) {
        stopFrameStream();
    }
    initialized_ = false;

    return true;
}

bool AxisCamera::parseFrame(std::string& buffer) {
    // This is a sort of optimization.
    // If we see that the buffer has little data, it's useless to continue in the parsing
    // but it's better to wait for more data
    if (buffer.size() < latestFrameSize_) {
        return false;
    }
    std::string headerField = "Content-Length:";
    // Look for the Content-Length header which contains the size in bytes of the jpeg image
    auto contentLengthPos = buffer.find(headerField);
    if (contentLengthPos == std::string::npos) {
        return false;
    }
    auto lineEndPos = buffer.find('\n', contentLengthPos);
    if (lineEndPos == std::string::npos) {
        return false;
    }

    // Extract the string containing the length and parse it
    std::string lenStr = buffer.substr(contentLengthPos + headerField.length(),
        lineEndPos - contentLengthPos - headerField.length());

    size_t jpegBytesSize = 0;
    try {
        jpegBytesSize = std::stoi(lenStr);
    } catch (const std::exception&) {
        return false;
    }

    // Skip another line of bullshit, then the jpeg bytes start
    auto jpegStartPos = buffer.find('\n', lineEndPos + 1);
    if (jpegStartPos == std::string::npos) {
        return false;
    }

    if (buffer.size() < jpegStartPos + 1 + jpegBytesSize) {
        return false;
    }

    std::string jpegBytes = buffer.substr(jpegStartPos + 1, jpegBytesSize);

    // remove everything that has been read from the buffer
    buffer = buffer.substr(jpegStartPos + jpegBytesSize + 1);

    // Create cv::Mat from bytes
    std::vector<char> vectordata(jpegBytes.begin(), jpegBytes.end());
    std::unique_lock<std::mutex> lock(frameMutex_);
    latestFrame_ = cv::imdecode(cv::Mat(vectordata, true), 1);
    lastFrameTime_ = std::chrono::high_resolution_clock::now();
    latestFrameSize_ = jpegBytesSize;
    frameCv_.notify_all();

    return true;
}

bool AxisCamera::frameGrabberLoop() {
    std::string buffer;
    std::string requestAddress = cameraCompleteAddress_ +
        "/mjpg/video.mjpg?resolution=" +
        std::to_string(currentResolution_.width) + "x" +
        std::to_string(currentResolution_.height) + "&fps=" +
        std::to_string(currentFramerate_);

    curlpp::Easy activeHandle;
    activeHandle.setOpt(new curlpp::options::Url(requestAddress));
    activeHandle.setOpt(new curlpp::options::WriteFunction([this, &buffer](char* ptr, size_t size, size_t nmemb) {  // NOLINT
        if (stopStreamRequest_) {
            return (size_t)0;
        }
        std::string receivedBytes(ptr, size*nmemb);
        buffer.append(receivedBytes);
        parseFrame(buffer);
        return size*nmemb;
    }));
    activeHandle.setOpt(new curlpp::options::BufferSize(512000));
    activeHandle.setOpt(new curlpp::options::ConnectTimeout(1));
    activeHandle.setOpt(new curlpp::options::Timeout(1));

    try {
        activeHandle.perform();
    } catch(const curlpp::LibcurlRuntimeError&) {
        logger_->warn("Exception {}", stopStreamRequest_);
        return stopStreamRequest_;
    }
    logger_->warn("Exit grabber");
    return true;
}

bool AxisCamera::startFrameStream() {
    logger_->debug("startFrameStream()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    stopStreamRequest_ = false;
    frameGrabberThread_ = std::thread(&AxisCamera::frameGrabberLoop, this);
    streamOn_ = true;
    return true;
}

bool AxisCamera::stopFrameStream() {
    logger_->debug("stopFrameStream()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    stopStreamRequest_ = true;
    frameGrabberThread_.join();

    streamOn_ = false;
    return true;
}

bool AxisCamera::setResolution(const cv::Size& resolution) {
    logger_->debug("setResolution()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    auto resolutions = getAvailableResolutions();
    bool resolutionExists = false;
    for (unsigned int i=0; i < resolutions.size(); i++) {
        if (resolutions[i] == resolution) {
            currentResolution_ = resolution;
            resolutionExists = true;
        }
    }

    if (!resolutionExists) {
        logger_->warn("Requested resolution is not available");
        return false;
    }

     if (streamOn_) {
        if (!stopFrameStream()) {
            logger_->warn("Could not stop stream to change resolution");
            return false;
        }

        if (!startFrameStream()) {
            logger_->warn("Could not restart the stream after changing resolution");
            return false;
        }
    }

    return true;
}

bool AxisCamera::setFramerate(int fps) {
    logger_->debug("setFramerate()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    currentFramerate_ = fps;

    if (streamOn_) {
        if (!stopFrameStream()) {
            logger_->warn("Could not stop stream to change resolution");
            return false;
        }

        if (!startFrameStream()) {
            logger_->warn("Could not restart the stream after changing resolution");
            return false;
        }
    }
    return true;
}

bool AxisCamera::setZoom(float zoom) {
    logger_->debug("setZoom()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    std::string request = cameraCompleteAddress_ +
        "axis-cgi/com/ptz.cgi?zoom="+
        std::to_string(static_cast<int>(zoom*9999));

    try {
        httpRequest(request);
    } catch (const std::exception& ex) {
        return false;
    }

    return true;
}

bool AxisCamera::setPosition(float pan, float tilt) {
    logger_->debug("setPosition()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    std::string request = cameraCompleteAddress_ +
        "axis-cgi/com/ptz.cgi?pan="+
        std::to_string(pan) + "&tilt=" +
        std::to_string(tilt);

    try {
        httpRequest(request);
    } catch (const std::exception& ex) {
        return false;
    }

    return true;
}

bool AxisCamera::setExposure(float exposure) {
    logger_->debug("setExposure()");
    logger_->warn("Not available");
    return false;
}

bool AxisCamera::setShutterSpeed(float) {
    logger_->debug("setShutterSpeed()");
    logger_->warn("Not available");
    return false;
}

bool AxisCamera::setFocusMode(FocusModes mode) {
    logger_->debug("setFocusMode()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    std::string request = cameraCompleteAddress_ +
        "axis-cgi/com/ptz.cgi?autofocus=";
    if (mode == FocusModes::Auto) {
        request += "on";
    } else {
        request += "off";
    }

    try {
        httpRequest(request);
    } catch (const std::exception& ex) {
        return false;
    }

    return true;
}

bool AxisCamera::setFocus(float focus) {
    logger_->debug("setFocus()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    if (getFocusMode() != FocusModes::Manual) {
        logger_->warn("Not in manual focus mode");
        return false;
    }

    std::string request = cameraCompleteAddress_ +
        "axis-cgi/com/ptz.cgi?focus="+
        std::to_string(static_cast<int>(focus*9999));

    try {
        httpRequest(request);
    } catch (const std::exception& ex) {
        return false;
    }

    return true;
}

bool AxisCamera::setISO(int) {
    logger_->debug("setISO()");
    logger_->warn("Not available");
    return false;
}

boost::optional<cv::Size> AxisCamera::getResolution() {
    logger_->debug("getResolution()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }

    return currentResolution_;
}

boost::optional<int> AxisCamera::getFramerate() {
    logger_->debug("getFramerate()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }

    return currentFramerate_;
}

boost::optional<float> AxisCamera::getZoom() {
    logger_->debug("getZoom()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }
    std::string response;
    try {
        response =
            httpRequest("/axis-cgi/com/ptz.cgi?query=position");
    } catch (const std::exception& ex) {
        return boost::none;
    }

    auto vecPositions = split(response, '\n');
    auto zoomStr = vecPositions[2].substr(vecPositions[2].find('=') + 1);
    float zoom = std::stof(zoomStr)/9999;
    return zoom;
}

boost::optional<float> AxisCamera::getPan() {
    logger_->debug("getPan()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }

    std::string response;
    try {
        response =
            httpRequest("/axis-cgi/com/ptz.cgi?query=position");
    } catch (const std::exception& ex) {
        return boost::none;
    }

    auto vecPositions = split(response, '\n');
    auto panStr = vecPositions[0].substr(vecPositions[0].find('=') + 1);
    return std::stof(panStr);
}

boost::optional<float> AxisCamera::getTilt() {
    logger_->debug("getPan()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }

    std::string response;
    try {
        response =
            httpRequest("/axis-cgi/com/ptz.cgi?query=position");
    } catch (const std::exception& ex) {
        return boost::none;
    }

    auto vecPositions = split(response, '\n');
    auto tiltStr = vecPositions[0].substr(vecPositions[0].find('=') + 1);
    return std::stof(tiltStr);
}

boost::optional<float> AxisCamera::getExposure() {
    logger_->debug("getExposure()");
    logger_->warn("Not available");
    return boost::none;
}

boost::optional<float> AxisCamera::getShutterSpeed() {
    logger_->debug("getShutterSpeed()");
    logger_->warn("Not available");
    return boost::none;
}

boost::optional<ICamera::FocusModes> AxisCamera::getFocusMode() {
    logger_->debug("getFocusMode()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }

    std::string response;
    try {
        response =
            httpRequest("/axis-cgi/com/ptz.cgi?query=position");
    } catch (const std::exception& ex) {
        return boost::none;
    }

    auto vecPositions = split(response, '\n');
    if (vecPositions[5].find("on") != std::string::npos) {
        return ICamera::FocusModes::Auto;
    }
    return ICamera::FocusModes::Manual;
}

boost::optional<float> AxisCamera::getFocus() {
    logger_->debug("getFocus()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }

    std::string response;
    try {
        response =
            httpRequest("/axis-cgi/com/ptz.cgi?query=position");
    } catch (const std::exception& ex) {
        return boost::none;
    }

    auto vecPositions = split(response, '\n');
    auto focusStr = vecPositions[3].substr(vecPositions[3].find('=') + 1);
    float focus = std::stof(focusStr)/9999;
    return focus;
}

boost::optional<int> AxisCamera::getISO() {
    logger_->debug("getISO()");
    logger_->warn("Not available");
    return boost::none;
}

cv::Mat AxisCamera::getFrame() {
    logger_->debug("getFrame()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return cv::Mat();
    }

    if (!streamOn_) {
        logger_->warn("Stream is not on");
        return cv::Mat();
    }

    std::unique_lock<std::mutex> lock(frameMutex_);
    auto currentFrameTime = lastFrameTime_;
    if (!frameCv_.wait_for(lock, std::chrono::milliseconds(
        TIMEOUT_FRAMERATE_FACTOR * 1000/currentFramerate_),
            [this, &currentFrameTime]() { return currentFrameTime != lastFrameTime_; })) {
                logger_->warn("Frame timeout");
                return cv::Mat();
    }

    return latestFrame_;
}

std::vector<cv::Size> AxisCamera::getAvailableResolutions() {
    logger_->debug("getAvailableResolutions()");
    std::vector<cv::Size> resolutions;
    if (!initialized_) {
        logger_->warn("Not initialized");
        return resolutions;
    }

    std::string response;
    try {
        response =
            httpRequest("/axis-cgi/param.cgi?action=list&group=root.Properties.Image.Resolution");
    } catch (const std::exception& ex) {
        return resolutions;
    }

    auto resStrVec = split(response.substr(response.find('=')+1), ',');
    for (unsigned int i=0; i < resStrVec.size(); i++) {
        auto thisresVec = split(resStrVec[i], 'x');
        if (thisresVec.size() != 2) {
            resolutions.clear();
            return resolutions;
        }

        cv::Size resolution(std::stoi(thisresVec[0]), std::stoi(thisresVec[1]));
        resolutions.push_back(resolution);
    }

    return resolutions;
}

std::vector<int> AxisCamera::getAvailableFramerates(const cv::Size& resolution){
    //TODO: This function has to be implemented yet.
    logger_->debug("getAvailableFramerates() not implemented yet");
    return std::vector<int>({5,10,15,25,30});
}

bool AxisCamera::setDeviceSpecificParameters(const nlohmann::json& configData) {
    logger_->debug("setDeviceSpecificParameters() not available yet");
    return false;
}

std::string AxisCamera::httpRequest(const std::string& request) {
    std::ostringstream os;
    curlpp::Easy requestCurl;
    requestCurl.setOpt(curlpp::options::Url(cameraCompleteAddress_ +
            request));
    requestCurl.setOpt(curlpp::options::WriteStream(&os));
    requestCurl.setOpt(new curlpp::options::ConnectTimeout(1));
    requestCurl.setOpt(new curlpp::options::Timeout(1));

    try {
        requestCurl.perform();
    } catch (const std::exception& ex) {
        logger_->error("httpRequest error: {}", ex.what());
        throw ex;
    }

    return os.str();
}

std::vector<std::string> AxisCamera::split(const std::string& s, const char seperator) {
    std::vector<std::string> output;

    std::string::size_type prev_pos = 0, pos = 0;
    while ((pos = s.find(seperator, pos)) != std::string::npos) {
        std::string substring(s.substr(prev_pos, pos-prev_pos));
        output.push_back(substring);
        prev_pos = ++pos;
    }

    output.push_back(s.substr(prev_pos, pos-prev_pos));
    return output;
}

}  // namespace cameras
}  // namespace sensors
}  // namespace crf
