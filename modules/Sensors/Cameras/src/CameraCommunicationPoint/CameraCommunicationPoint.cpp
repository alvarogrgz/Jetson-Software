/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 * Contributors: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
                 Alvaro Garcia Gonzalez CERN BE/CEM/MRO 2022
 *  ==================================================================================================
 */

#include <memory>
#include <string>
#include <vector>

#include "Cameras/CameraCommunicationPoint/CameraCommunicationPoint.hpp"
#include "VideoCodecs/JPEGVideoCodec/JPEGVideoEncoder.hpp"
#include "VideoCodecs/x264VideoCodec/x264VideoEncoder.hpp"
#include "VideoCodecs/cvMatVideoCodec/cvMatVideoEncoder.hpp"

namespace crf {
namespace sensors {
namespace cameras {

std::vector<bool> CameraCommunicationPoint::availableStreams_({
    true, true, true, true, true
});

CameraCommunicationPoint::CameraCommunicationPoint(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    std::shared_ptr<CameraManager> manager) :
    StatusStreamerCommunicationPoint(socket, manager),
    StreamID_(-1),
    requestedStreamFramerate_(),
    frameStreamActive_(false),
    stopFrameStream_(true),
    frameStreamerThread_(),
    encoderMutex_(),
    encoder_(),
    encodingType_(communication::datapackets::FramePacket::Encoding::CV_MAT),
    socket_(socket),
    logger_("CameraCommunicationPoint"),
    manager_(manager) {
    logger_->debug("CTor"); 
    jsonCommandHandlers_.insert({"setStatus",
            std::bind(&CameraCommunicationPoint::setStatusRequestHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({"getFrame",
            std::bind(&CameraCommunicationPoint::getFrameRequestHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({"startFrameStream",
            std::bind(&CameraCommunicationPoint::startFrameStreamRequestHandler,
            this, std::placeholders::_1)});
    jsonCommandHandlers_.insert({"stopFrameStream",
            std::bind(&CameraCommunicationPoint::stopFrameStreamRequestHandler,
            this, std::placeholders::_1)});
}


CameraCommunicationPoint::~CameraCommunicationPoint() {
    logger_->debug("DTor");
    if (initialized_) {
        deinitialize();
    }
}

bool CameraCommunicationPoint::deinitialize() {
    logger_->debug("deinitialize()");
    

    StatusStreamerCommunicationPoint::deinitialize();

    if(frameStreamActive_){
        manager_->stopFrameStream(StreamID_);
        availableStreams_[StreamID_] = true;
    }

    stopFrameStream_ = true;
    if (frameStreamerThread_.joinable())
        frameStreamerThread_.join();

    return true;
}

void CameraCommunicationPoint::setStatusRequestHandler(
    const communication::datapackets::JSONPacket& packet)  {
    logger_->debug("setStatusRequestHandler()");
    bool result = manager_->setStatus(packet.data);
    if(!result){
        sendJSONPacket<std::string>("error", "Failed to set", true);
    } else {
        sendJSONPacket<bool>("setStatus", result, true);
    }
}

void CameraCommunicationPoint::getFrameRequestHandler(
    const communication::datapackets::JSONPacket&) {
    logger_->debug("getFrameRequestHandler()");
    auto frame = manager_->getSingleFrame();
    if (frame.empty()) {

        sendJSONPacket<std::string>("error", " Error getting the frame", true);
        return;
    }

    int quality = 8;
    algorithms::videocodecs::JPEGVideoEncoder encoder(static_cast<algorithms::videocodecs::CompressionQuality>(quality));
    encoder.addFrame(frame);
    std::string bytes = encoder.getBytes();
    communication::datapackets::FramePacket packet(bytes, 
        communication::datapackets::FramePacket::Encoding::JPEG);
    socket_->write(packet, packet.getHeader(), false);
}

void CameraCommunicationPoint::startFrameStreamRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("startFrameStreamRequestHandler()");
    cv::Size resolution;
    std::string encoderStr;
    int cameraFramerate;

    // Parse parameters
    try {
        auto res = packet.data.at("resolution").get<std::vector<uint32_t> >();
        if (res.size() != 2) {
            logger_->error("startFrameStream(): Wrong resolution format");
            sendJSONPacket<std::string>("error", "Wrong resolution format", true);
            return;
        }
        resolution = cv::Size(res[0], res[1]);
        requestedStreamFramerate_ = packet.data.at("stream_framerate").get<int>();        
        if(packet.data.contains("camera_framerate")){
            cameraFramerate = packet.data.at("camera_framerate").get<int>();
        }
        else{
            cameraFramerate=requestedStreamFramerate_;
        }

        int quality = packet.data.at("encoding_quality").get<int>();
        if ((quality < 0) || (quality > 8)) {
            logger_->error("startFrameStream(): Not permitted quality value: " + std::to_string(quality));
            sendJSONPacket<std::string>("error", "Not permitted quality value: " + std::to_string(quality), true);
            return;
        }


        encoderStr = packet.data.at("encoding_format").get<std::string>();
        const std::scoped_lock<std::mutex> guard(encoderMutex_);
        if (encoderStr == "jpeg") {
            encoder_.reset(new algorithms::videocodecs::JPEGVideoEncoder(
                static_cast<algorithms::videocodecs::CompressionQuality>(quality)));
            encodingType_ = communication::datapackets::FramePacket::Encoding::JPEG;
        } else if (encoderStr == "x264") {
            encoder_.reset(new algorithms::videocodecs::x264VideoEncoder(resolution,
                static_cast<algorithms::videocodecs::CompressionQuality>(quality), true));
            encodingType_ = communication::datapackets::FramePacket::Encoding::X264;
        } else if (encoderStr == "cvmat") {
            encoder_.reset(new algorithms::videocodecs::cvMatVideoEncoder());
            encodingType_ = communication::datapackets::FramePacket::Encoding::CV_MAT;
        }
        else{
            logger_->error("startFrameStream(): Unsupported encoder: " + encoderStr);
            sendJSONPacket<std::string>("error", " Unsupported encoder: " + encoderStr, true);
            return;
        }
    } catch (const std::exception&) {
        logger_->error("startFrameStream(): Wrong stream parameters");
        sendJSONPacket<std::string>("error", "Wrong stream parameters", true);
        return;
    }

    // Get available StreamID
    if(!frameStreamActive_){
        auto itr=std::find(availableStreams_.begin(), availableStreams_.end(), true);
        if(itr==availableStreams_.end()){
            logger_->error("startFrameStream(): All streams are busy");
            sendJSONPacket<std::string>("error", "All streams are busy", true);
            return;
        }
        StreamID_ = std::distance(availableStreams_.begin(), itr);
    }

    try{
        if (!manager_->startFrameStream(StreamID_, resolution, cameraFramerate)) {
            logger_->error("startFrameStream(): Could not request frames");
            sendJSONPacket<std::string>("error", "Could not request frames", true);
            return;
            }
    }
    catch(const std::exception& e){
        logger_->error("startFrameStream(): Could not request frames {}", std::string(e.what()));
        sendJSONPacket<std::string>("error", std::string(e.what()), true);
        return;
    }
    
    if (!frameStreamActive_ && !stopThreads_) {
        logger_->debug("Creating new frameStreamer");
        stopFrameStream_ = false;
        frameStreamerThread_ = std::thread(&CameraCommunicationPoint::frameStreamer,
            this, StreamID_);
    }

    logger_->info("Started {} stream with resolution {}", encoderStr, resolution);
}

void CameraCommunicationPoint::stopFrameStreamRequestHandler(
    const communication::datapackets::JSONPacket&) {
    logger_->debug("stopFrameStreamRequestHandler()");
    if (!frameStreamActive_) {
        logger_->error( "stopFrameStream(): Stream was not active" );
        sendJSONPacket<std::string>("error", "Stream was not active", true);
        return;
    }
    stopFrameStream_ = true;
    if(frameStreamerThread_.joinable()){
        frameStreamerThread_.join();
    }
    manager_->stopFrameStream(StreamID_);
}

void CameraCommunicationPoint::frameStreamer(int streamId) {
    logger_->debug("frameStreamer()");
    frameStreamActive_ = true;
    stopFrameStream_ = false;
    availableStreams_[StreamID_] = false;
    cv::Mat frame;
    std::string bytes;
    while (!stopThreads_ && !stopFrameStream_) {
        
        auto start = std::chrono::high_resolution_clock::now();
        try {
            frame = manager_->getFrame(streamId);
        } catch(std::runtime_error& e) {
            sendJSONPacket<std::string>("error", "The frame grabber is not running, set the stream again.", true);
            stopFrameStream_=true;
            continue;
        }

        if (frame.empty())
            continue;
        
        {
            std::scoped_lock<std::mutex> guard(encoderMutex_);
            encoder_->addFrame(frame);
            bytes = encoder_->getBytes();
        }
        
        if (bytes.length() == 0)
            continue;

        communication::datapackets::FramePacket packet(bytes, encodingType_);
        socket_->write(packet, packet.getHeader(), false);
        auto end = std::chrono::high_resolution_clock::now();
        auto loopDuration = end-start;
        if (loopDuration < std::chrono::milliseconds(1000/requestedStreamFramerate_)) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1000/requestedStreamFramerate_) - loopDuration);
        }
    }
 
    std::scoped_lock<std::mutex> guard(encoderMutex_);
    encoder_->flush();
    availableStreams_[StreamID_] = true;
    frameStreamActive_ = false;
}


}   // namespace cameras
}   // namespace sensors
}   // namespace crf
