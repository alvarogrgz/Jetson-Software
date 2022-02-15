/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Cameras/CameraClient/CameraClient.hpp"
#include "DataPackets/JSONPacket/JSONPacket.hpp"
#include "VideoCodecs/JPEGVideoCodec/JPEGVideoDecoder.hpp"
#include "VideoCodecs/cvMatVideoCodec/cvMatVideoDecoder.hpp"

namespace crf {
namespace sensors {
namespace cameras {

CameraClient::CameraClient(std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    const std::chrono::milliseconds& serverResponseTimeout) :
        logger_("CameraClient"),
        socket_(socket),
        activeDecoder_(),
        serverResponseTimeout_(serverResponseTimeout),
        requestedResolution_(640, 480),
        requestedFramerate_(30),
        encoding_(communication::datapackets::FramePacket::Encoding::JPEG),
        quality_(algorithms::videocodecs::CompressionQuality::Normal),
        jsonMutex_(),
        jsonCv_(),
        parameters_(),
        initialized_(false),
        stopThread_(false),
        frameMutex_(),
        frameCv_(),
        latestFrame_(),
        grabberThread_() {
            logger_->debug("CTor");

            activeDecoder_.reset(new algorithms::videocodecs::JPEGVideoDecoder());
}

CameraClient::~CameraClient() {
    logger_->debug("DTor");
    deinitialize();
}

bool CameraClient::initialize() {
    logger_->debug("initialize()");

    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }

    if (!socket_->open()) {
        logger_->warn("Failed to connect to camera");
        return false;
    }

    stopThread_ = false;
    grabberThread_ = std::thread(&CameraClient::grabber, this);
    initialized_ = true;

    if (!startFrameStream()) {
        logger_->warn("Failed to start stream");
        deinitialize();
        return false;
    }

    if (!reqGetParameters()) {
        logger_->warn("Failed to get parameters");
        deinitialize();
        return false;
    }

    return true;
}

bool CameraClient::deinitialize() {
    logger_->debug("deinitialize()");

    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }
    stopFrameStream();
    socket_->close();

    stopThread_ = true;
    if (grabberThread_.joinable())
        grabberThread_.join();

    initialized_ = false;
    return true;
}

bool CameraClient::setResolution(const cv::Size& resolution) {
    logger_->debug("setResolution()");
    requestedResolution_ = resolution;
    if (initialized_) {
        if (!startFrameStream()) {
            return false;
        }
    }
    return true;
}

bool CameraClient::setFramerate(int fps) {
    logger_->debug("setFramerate()");
    if (fps < 1) {
        logger_->warn("Wrong framerate");
        return false;
    }

    requestedFramerate_ = fps;
    if (initialized_) {
        if (!startFrameStream()) {
            return false;
        }
    }

    return true;
}

bool CameraClient::setZoom(float zoom) {
    logger_->debug("setZoom()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    if (parameters_.find("zoom") == parameters_.end()) {
        logger_->warn("Zoom setting not supported by the camera");
        return false;
    }

    communication::datapackets::JSONPacket json;
    json.data["command"] = "setStatus";
    json.data["zoom"] = zoom;

    return socket_->write(json, json.getHeader());
}

bool CameraClient::setPosition(float pan, float tilt) {
    logger_->debug("setPosition()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    if (parameters_.find("position") == parameters_.end()) {
        logger_->warn("Position setting not supported by the camera");
        return false;
    }

    std::vector<float> position({
            pan,
            tilt
        });
    communication::datapackets::JSONPacket json;
    json.data["command"] = "setStatus";
    json.data["position"] = position;

    return socket_->write(json, json.getHeader());
}

bool CameraClient::setExposure(float exposure) {
    logger_->debug("setExposure()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    if (parameters_.find("exposure") == parameters_.end()) {
        logger_->warn("Exposure setting not supported by the camera");
        return false;
    }

    communication::datapackets::JSONPacket json;
    json.data["command"] = "setStatus";
    json.data["exposure"] = exposure;

    return socket_->write(json, json.getHeader());
}

bool CameraClient::setShutterSpeed(float shutter) {
    logger_->debug("setShutterSpeed()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    if (parameters_.find("shutter") == parameters_.end()) {
        logger_->warn("Shutter setting not supported by the camera");
        return false;
    }

    communication::datapackets::JSONPacket json;
    json.data["command"] = "setStatus";
    json.data["shutter"] = shutter;

    return socket_->write(json, json.getHeader());
}

bool CameraClient::setFocusMode(ICamera::FocusModes focusmode) {
    logger_->debug("setFocusMode()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    if (parameters_.find("focusmode") == parameters_.end()) {
        logger_->warn("Focus mode setting not supported by the camera");
        return false;
    }

    communication::datapackets::JSONPacket json;
    json.data["command"] = "setStatus";
    json.data["focusmode"] = focusmode == FocusModes::Auto ?
        "auto" : "manual";

    return socket_->write(json, json.getHeader());
}

bool CameraClient::setFocus(float focus) {
    logger_->debug("setFocus()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    if (parameters_.find("focus") == parameters_.end()) {
        logger_->warn("Focus setting not supported by the camera");
        return false;
    }

    communication::datapackets::JSONPacket json;
    json.data["command"] = "setStatus";
    json.data["focus"] = focus;

    return socket_->write(json, json.getHeader());
}

bool CameraClient::setISO(int iso) {
    logger_->debug("setISO()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    if (parameters_.find("iso") == parameters_.end()) {
        logger_->warn("ISO setting not supported by the camera");
        return false;
    }

    communication::datapackets::JSONPacket json;
    json.data["command"] = "setStatus";
    json.data["iso"] = iso;

    return socket_->write(json, json.getHeader());
}

boost::optional<cv::Size> CameraClient::getResolution() {
    logger_->debug("getResolution()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }

    return latestFrame_.size();
}

boost::optional<int> CameraClient::getFramerate() {
    logger_->debug("getFramerate()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }
    return requestedFramerate_;
}

boost::optional<float> CameraClient::getZoom() {
    logger_->debug("getZoom()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }

    if (parameters_.find("zoom") == parameters_.end()) {
        logger_->warn("Zoom setting not supported by the camera");
        return boost::none;
    }

    if (!reqGetParameters()) {
        return boost::none;
    }

    float zoom = 0;
    try {
        zoom = parameters_.at("zoom").get<float>();
    }  catch (const std::exception& ex) {
        logger_->warn("Failed reading json response: {}", ex.what());
        return boost::none;
    }
    return zoom;
}

boost::optional<float> CameraClient::getPan() {
    logger_->debug("getPan()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }
    if (parameters_.find("position") == parameters_.end()) {
        logger_->warn("Pan setting not supported by the camera");
        return boost::none;
    }

    if (!reqGetParameters()) {
        return boost::none;
    }

    float pan = 0;
    try {
        auto pos = parameters_.at("position").get<std::vector<float> >();
        if (pos.size() != 2) return boost::none;
        pan = pos[0];
    }  catch (const std::exception& ex) {
        logger_->warn("Failed reading json response: {}", ex.what());
        return boost::none;
    }
    return pan;
}

boost::optional<float> CameraClient::getTilt() {
    logger_->debug("getTilt()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }
    if (parameters_.find("position") == parameters_.end()) {
        logger_->warn("Tilt setting not supported by the camera");
        return boost::none;
    }

    if (!reqGetParameters()) {
        return boost::none;
    }

    float tilt = 0;
    try {
        auto pos = parameters_.at("position").get<std::vector<float> >();
        if (pos.size() != 2) return boost::none;
        tilt = pos[1];
    }  catch (const std::exception& ex) {
        logger_->warn("Failed reading json response: {}", ex.what());
        return boost::none;
    }
    return tilt;
}

boost::optional<float> CameraClient::getExposure() {
    logger_->debug("getExposure()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }
    if (parameters_.find("exposure") == parameters_.end()) {
        logger_->warn("Exposure setting not supported by the camera");
        return boost::none;
    }

    if (!reqGetParameters()) {
        return boost::none;
    }

    float exposure = 0;
    try {
        exposure = parameters_.at("exposure").get<float>();
    }  catch (const std::exception& ex) {
        logger_->warn("Failed reading json response: {}", ex.what());
        return boost::none;
    }
    return exposure;
}

boost::optional<float> CameraClient::getShutterSpeed() {
    logger_->debug("getShutterSpeed()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }

    if (parameters_.find("shutter") == parameters_.end()) {
        logger_->warn("Shutter speed setting not supported by the camera");
        return boost::none;
    }

    if (!reqGetParameters()) {
        return boost::none;
    }

    float shutter = 0;
    try {
        shutter = parameters_.at("shutter").get<float>();
    }  catch (const std::exception& ex) {
        logger_->warn("Failed reading json response: {}", ex.what());
        return boost::none;
    }
    return shutter;
}

boost::optional<ICamera::FocusModes> CameraClient::getFocusMode() {
    logger_->debug("getFocusMode()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }
    if (parameters_.find("focusmode") == parameters_.end()) {
        logger_->warn("Focus setting not supported by the camera");
        return boost::none;
    }

    if (!reqGetParameters()) {
        return boost::none;
    }

    ICamera::FocusModes mode;
    try {
        mode = parameters_.at("focusmode").get<std::string>() == "auto" ?
            ICamera::FocusModes::Auto :
            ICamera::FocusModes::Manual;
    }  catch (const std::exception& ex) {
        logger_->warn("Failed reading json response: {}", ex.what());
        return boost::none;
    }
    return mode;
}

boost::optional<float> CameraClient::getFocus() {
    logger_->debug("getFocus()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }

    if (parameters_.find("focus") == parameters_.end()) {
        logger_->warn("Focus setting not supported by the camera");
        return boost::none;
    }

    if (!reqGetParameters()) {
        return boost::none;
    }

    float focus = 0;
    try {
        focus = parameters_.at("focus").get<float>();
    }  catch (const std::exception& ex) {
        logger_->warn("Failed reading json response: {}", ex.what());
        return boost::none;
    }
    return focus;
}

boost::optional<int> CameraClient::getISO() {
    logger_->debug("getISO()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }

    if (parameters_.find("iso") == parameters_.end()) {
        logger_->warn("ISO setting not supported by the camera");
        return boost::none;
    }

    if (!reqGetParameters()) {
        return boost::none;
    }

    int iso = 0;
    try {
        iso = parameters_.at("iso").get<int>();
    }  catch (const std::exception& ex) {
        logger_->warn("Failed reading json response: {}", ex.what());
        return boost::none;
    }
    return iso;
}

cv::Mat CameraClient::getFrame() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return cv::Mat();
    }

    std::unique_lock<std::mutex> lock(frameMutex_);
    if (frameCv_.wait_for(lock, serverResponseTimeout_*2) == std::cv_status::timeout) {
        logger_->warn("Timeout waiting for frame");
        return cv::Mat();
    }
    return latestFrame_;
}

std::vector<cv::Size> CameraClient::getAvailableResolutions() {
    logger_->debug("getAvailableResolutions()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::vector<cv::Size>();
    }

    if (parameters_.find("profiles") == parameters_.end()) {
        return std::vector<cv::Size>();
    }

    std::vector<cv::Size> resolutions;
    auto profiles = parameters_["profiles"];
    for (nlohmann::json::iterator it = profiles.begin(); it != profiles.end(); ++it) {
        std::pair <int, int> res = it.value()["resolution"].get<std::pair <int, int>>();
        resolutions.push_back(cv::Size(res.first, res.second));
    }
    return resolutions;
}

std::vector<int> CameraClient::getAvailableFramerates(const cv::Size& resolution){
    logger_->debug("getAvailableFramerates()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return std::vector<int>();    
    }

    if (parameters_.find("profiles") == parameters_.end()) {
        return std::vector<int>();    
    }

    std::vector<cv::Size> resolutions;
    auto profiles = parameters_["profiles"];
    for (nlohmann::json::iterator it = profiles.begin(); it != profiles.end(); ++it) {
        std::pair <int, int> res = it.value()["resolution"].get<std::pair <int, int>>();
        if(res.first == resolution.width && res.second == resolution.height)
        return it.value()["framerates"].get<std::vector<int>>();
    }
    return std::vector<int>();
}


bool CameraClient::setDeviceSpecificParameters(const nlohmann::json& configData) {
    return false;
}

bool CameraClient::setEncoding(communication::datapackets::FramePacket::Encoding encoding,
    algorithms::videocodecs::CompressionQuality quality) {
        logger_->debug("setEncoding()");
        encoding_ = encoding;
        quality_ = quality;

        if (initialized_) {
            if (!startFrameStream()) {
                return false;
            }
        }

        return true;
}

void CameraClient::grabber() {
    while (!stopThread_) {
        auto retval = socket_->read();

        if (!socket_->isOpen()) {
            return;
        }

        if (!retval) {
            continue;
        }

        auto buffer = retval.get().first;
        auto header = retval.get().second;

        if (header.type() == communication::datapackets::JSON_PACKET_TYPE) {
            communication::datapackets::JSONPacket json;
            if (!json.deserialize(buffer)) {
                logger_->warn("Failed to deserialize json packet: {}", buffer);
                continue;
            }
            std::unique_lock<std::mutex> lock(jsonMutex_);
            try {
                if ((json.data.at("command").get<std::string>() == "reply") &&
                    (json.data.at("replyCommand").get<std::string>() == "getStatus")) {
                        parameters_ = json.data.at("message");
                }
            } catch (const std::exception& ex) {
                logger_->warn("Failed reading json response: {}", ex.what());
                continue;
            }
            jsonCv_.notify_all();
        }

        if (header.type() == communication::datapackets::FRAME_PACKET) {
            communication::datapackets::FramePacket frame;
            if (!frame.deserialize(buffer)) {
                logger_->warn("Failed to deserialize frame packet");
                continue;
            }

            if (frame.getEncodingType() != encoding_) {
                switch (frame.getEncodingType()) {
                    case communication::datapackets::FramePacket::Encoding::JPEG:
                        activeDecoder_.reset(new algorithms::videocodecs::JPEGVideoDecoder());
                        break;
                    case communication::datapackets::FramePacket::Encoding::CV_MAT:
                        activeDecoder_.reset(new algorithms::videocodecs::cvMatVideoDecoder());
                        break;
                    default:
                        logger_->warn("Not available decoder");
                        continue;
                }

                encoding_ = frame.getEncodingType();
            }

            if (!activeDecoder_->addBytes(frame.getBytes())) {
                logger_->warn("Failed to add bytes to decoder");
                continue;
            }
            cv::Mat decoded = activeDecoder_->getFrame();

            if (decoded.empty()) {
                logger_->warn("Decoded image is empty");
                continue;
            }
            std::unique_lock<std::mutex> lock(frameMutex_);
            latestFrame_ = decoded;
            frameCv_.notify_all();
        }
    }
}

bool CameraClient::reqGetParameters() {
    communication::datapackets::JSONPacket json;
    json.data["command"] = "getStatus";

    std::unique_lock<std::mutex> lock(jsonMutex_);
    if (!socket_->write(json, json.getHeader())) {
        return false;
    }
    if (jsonCv_.wait_for(lock, serverResponseTimeout_) == std::cv_status::timeout) {
        return false;
    }

    return true;
}

bool CameraClient::startFrameStream() {
    communication::datapackets::JSONPacket json;
    json.data["command"] = "startFrameStream";
    std::vector<int32_t> res({
        requestedResolution_.width,
        requestedResolution_.height});
    json.data["resolution"] = res;
    json.data["stream_framerate"] = requestedFramerate_;
    json.data["encoding_quality"] = static_cast<int>(quality_);
    json.data["encoding_format"] = encoding_ == communication::datapackets::FramePacket::Encoding::JPEG ?
        "jpeg" : "cvmat";

    if (!socket_->write(json, json.getHeader())) {
        return false;
    }

    std::unique_lock<std::mutex> lock(frameMutex_);
    if (frameCv_.wait_for(lock, serverResponseTimeout_) == std::cv_status::timeout) {
        logger_->warn("Timeout waiting for frame");
        return false;
    }

    return true;
}

bool CameraClient::stopFrameStream() {
    communication::datapackets::JSONPacket json;
    json.data["command"] = "stopFrameStream";
    return socket_->write(json, json.getHeader());
}

}  // namespace cameras
}  // namespace sensors
}  // namespace crf
