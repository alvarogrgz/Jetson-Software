/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi & Carlos Prados Sesmero CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <lz4.h>
#include <memory>
#include <string>
#include <vector>

#include "RGBDCameras/RGBDCameraCommunicationPoint/RGBDCameraCommunicationPoint.hpp"
#include "VisionUtility/Image/ImageJSONConverter.hpp"
#include "CommunicationUtility/StreamWriter.hpp"
#include "VideoCodecs/JPEGVideoCodec/JPEGVideoEncoder.hpp"
#include "VideoCodecs/x264VideoCodec/x264VideoEncoder.hpp"
#include "VideoCodecs/cvMatVideoCodec/cvMatVideoEncoder.hpp"
#include "VisionUtility/PointCloud/Communication.hpp"
#include "VisionUtility/PointCloud/Subsample.hpp"

using crf::utility::visionutility::pointcloud::communication::serializePointCloud;
using crf::utility::visionutility::pointcloud::subsample::voxelGridSubsample;

namespace crf {
namespace sensors {
namespace rgbdcameras {

RGBDCameraCommunicationPoint::RGBDCameraCommunicationPoint(
    std::shared_ptr<communication::datapacketsocket::PacketSocket> socket,
    std::shared_ptr<RGBDCameraManager> manager) :
    CameraCommunicationPoint(socket, manager),
    logger_("RGBDCameraCommunicationPoint"),
    manager_(manager),
    currentRgbEncoding_(communication::datapackets::RGBDFramePacket::RGBEncoding::CV_MAT),
    currentDepthEncoding_(communication::datapackets::RGBDFramePacket::DepthEncoding::CV_MAT),
    currentPointCloudEncoding_(
        communication::datapackets::RGBDFramePacket::PointCloudEncoding::PLY),
    rgbRequested_(false),
    depthRequested_(false),
    pointCloudRequested_(false),
    alignFramesRequest_(false),
    extrinsics_(),
    depthIntrinsics_(),
    depthDistortion_(),
    colorIntrinsics_(),
    colorDistortion_() {
    logger_->debug("CTor");
}

RGBDCameraCommunicationPoint::~RGBDCameraCommunicationPoint() {
    logger_->debug("DTor");
}

void RGBDCameraCommunicationPoint::startFrameStreamRequestHandler(
    const communication::datapackets::JSONPacket& packet) {
    logger_->debug("startFrameStreamRequestHandler()");

    cv::Size resolution, depthResolution;
    std::string encoderStr, encoderDepthStr, encoderPcStr;
    rgbRequested_ = false;
    depthRequested_ = false;
    pointCloudRequested_ = false;
    int cameraFramerate, depthFramerate;

    try {
        auto res = packet.data.at("resolution").get<std::vector<uint32_t> >();
        if (res.size() != 2) {
           logger_->error("startFrameStream(): Wrong resolution format");
            sendJSONPacket<std::string>("error", "Wrong resolution format", true);
            return;
        }
        resolution = cv::Size(res[0], res[1]);


        res = packet.data.at("depth_resolution").get<std::vector<uint32_t> >();
        if (res.size() != 2) {
           logger_->error("startFrameStream(): Wrong depth resolution format");
            sendJSONPacket<std::string>("error", "Wrong depth resolution format", true);
            return;
        }
        depthResolution = cv::Size(res[0], res[1]);

        requestedStreamFramerate_ = packet.data.at("stream_framerate").get<int>();
        if(packet.data.contains("camera_framerate")){
            cameraFramerate = packet.data.at("camera_framerate").get<int>();
        }
        else{
            cameraFramerate=requestedStreamFramerate_;
        }
        if(packet.data.contains("depth_framerate")){
            depthFramerate = packet.data.at("depth_framerate").get<int>();
        }
        else{
            depthFramerate=requestedStreamFramerate_;
        }

        if (packet.data.find("align_frames") != packet.data.end()) {
            alignFramesRequest_ = packet.data.at("align_frames").get<bool>();
        }

        if (packet.data.find("encoding_format") != packet.data.end()) {
            rgbRequested_ = true;
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
        }

        if (packet.data.find("depth_format") != packet.data.end()) {
            depthRequested_ = true;
            encoderDepthStr = packet.data.at("depth_format").get<std::string>();

            if (encoderDepthStr == "cvmat") {
                currentDepthEncoding_ = communication::datapackets::RGBDFramePacket::DepthEncoding::CV_MAT;
            } else if (encoderDepthStr == "lz4") {
                currentDepthEncoding_ = communication::datapackets::RGBDFramePacket::DepthEncoding::LZ4;
            } else{
                logger_->error("startFrameStream(): Unsupported depth encoder: " + encoderStr);
                sendJSONPacket<std::string>("error", " Unsupported depth encoder: " + encoderStr, true);
                return;
            }
        }

        if (packet.data.find("point_cloud_format") != packet.data.end()) {
            pointCloudRequested_ = true;
            encoderPcStr = packet.data.at("point_cloud_format").get<std::string>();
            subsampling_ = packet.data.at("subsampling").get<float>();

            if (encoderDepthStr == "ply") {
                currentPointCloudEncoding_ = communication::datapackets::RGBDFramePacket::PointCloudEncoding::PLY;
            } else if (encoderDepthStr == "lz4") {
                currentPointCloudEncoding_ = communication::datapackets::RGBDFramePacket::PointCloudEncoding::LZ4;
            } else{
                logger_->error("startFrameStream(): Unsupported pointcloud encoder: " + encoderStr);
                sendJSONPacket<std::string>("error", " Unsupported pointcloud encoder: " + encoderStr, true);
                return;
            }
        }

    } catch (const std::exception&) {
        logger_->error("startFrameStream(): Wrong stream parameters");
        sendJSONPacket<std::string>("error", "Wrong stream parameters", true);
        return;
    }

    if (!rgbRequested_ && !depthRequested_ && !pointCloudRequested_) {
        logger_->error("startFrameStream(): No stream requested");
        sendJSONPacket<std::string>("error", "No stream requested", true);
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
    //TODO: This is hardcoded just for testing purposes
    depthResolution = cv::Size(640, 480);
    depthFramerate = 15;
    bool rgbdRequested = depthRequested_ || rgbRequested_;
    
    try{
        if (!manager_->startFrameStream(StreamID_, rgbdRequested, pointCloudRequested_, resolution, cameraFramerate, depthResolution, depthFramerate)) {
            logger_->error("startFrameStream(): Could not request frames");
            sendJSONPacket<std::string>("error", "Could not request frames", true);
        }
    }
    catch(const std::exception& e){
        logger_->error("startFrameStream(): Could not request frames");
        sendJSONPacket<std::string>("error", std::string(e.what()), true);
        return;
    }

    if (!frameStreamActive_) {
        logger_->debug("Creating new frameStreamer");
        frameStreamerThread_ = std::thread(&RGBDCameraCommunicationPoint::frameStreamer,
            this, StreamID_);
    }

    if(rgbRequested_)
        logger_->info("Started {} stream with resolution {}", encoderStr, resolution);
    if(depthRequested_)
        logger_->info("Started {} depth stream with resolution {}", encoderDepthStr, depthResolution);
    if(pointCloudRequested_)
        logger_->info("Started {} pointcloud stream with subsampling {}", encoderPcStr, subsampling_);
    return;
}


void RGBDCameraCommunicationPoint::frameStreamer(int streamId) {
    logger_->debug("frameStreamer()");
    frameStreamActive_ = true;
    stopStream_ = false;
    availableStreams_[StreamID_] = false;

    cv::rgbd::RgbdFrame frame;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud;

    while (!stopStream_) {
        auto start = std::chrono::high_resolution_clock::now();
        if (!rgbRequested_ && !pointCloudRequested_ && !depthRequested_) {
            logger_->debug("frameStreamer(): Empty request");
            std::this_thread::sleep_for(std::chrono::milliseconds(150));
            break;
        }
        communication::datapackets::RGBDFramePacket packet;

        if (rgbRequested_ || depthRequested_) {
            frame = manager_->getRGBDFrame(streamId);
        }

        if (pointCloudRequested_) {
             pointCloud = manager_->getPointCloud(streamId, alignFramesRequest_);

            std::vector<float> sideLength = {subsampling_, subsampling_, subsampling_};
            pointCloud = voxelGridSubsample<pcl::PointXYZRGBA>(pointCloud, sideLength);

            if (pointCloud != nullptr) {
                Eigen::Vector4f origin(0.0, 0.0, 0.0, 0.0);
                Eigen::Quaternionf orientation(Eigen::Matrix3f::Identity());
                std::ostringstream oss = serializePointCloud<pcl::PointXYZRGBA>(pointCloud, origin,
                    orientation, false);

                std::string bytes = oss.str();
                if (currentPointCloudEncoding_ == communication::datapackets::
                    RGBDFramePacket::PointCloudEncoding::LZ4) {
                    bytes = compressLZ4(bytes);
                }
                packet.setPointCloudBytes(currentPointCloudEncoding_, bytes);
            }
        }

        if (rgbRequested_ && !frame.image.empty()) {
            std::string bytes;
            {
                std::scoped_lock<std::mutex> guard(encoderMutex_);
                encoder_->addFrame(frame.image);
                bytes = encoder_->getBytes();
            }
            packet.setRGBBytes(currentRgbEncoding_, bytes);
        }

        if (depthRequested_ && !frame.depth.empty()) {
            utility::communicationutility::StreamWriter writer;
            writer.write(frame.depth);
            std::string bytes = writer.toString();
            if (currentDepthEncoding_ ==
                communication::datapackets::RGBDFramePacket::DepthEncoding::LZ4) {
                bytes = compressLZ4(bytes);
            }
            packet.setDepthBytes(currentDepthEncoding_, bytes);
        }


        socket_->write(packet, packet.getHeader(), false);

        auto end = std::chrono::high_resolution_clock::now();
        auto loopDuration = end-start;
        if (loopDuration < std::chrono::milliseconds(1000/requestedStreamFramerate_)) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1000/requestedStreamFramerate_) - loopDuration);
        }
    } //  while (!stopStream_) 
    
    { // Flush the encoder
        std::scoped_lock<std::mutex> guard(encoderMutex_);
            encoder_->flush();
    }
    availableStreams_[StreamID_] = true;
    frameStreamActive_ = false;
}

std::string RGBDCameraCommunicationPoint::compressLZ4(const std::string& bytes) {
    const int maxDstSize = LZ4_compressBound(bytes.length());
    char* compressedData = new char[maxDstSize];
    const int compressedDataSize = LZ4_compress_default(bytes.c_str(), compressedData,
        bytes.length(), maxDstSize);
    std::string compressedString;
    compressedString.resize(4);
    int32_t decompressedLength = static_cast<int32_t>(bytes.length());
    std::memcpy(const_cast<char*>(compressedString.c_str()), &decompressedLength, 4);
    compressedString.append(std::string(compressedData, compressedDataSize));
    delete[] compressedData;
    return compressedString;
}

void RGBDCameraCommunicationPoint::updateCameraParameters() {
    logger_->debug("updateCameraParameters()");
    try {
        auto cameraParameters = manager_->getStatus();
        extrinsics_ = cameraParameters.at("exstrinsic").get<cv::Mat>();
        depthIntrinsics_ = cameraParameters.at("depth_intrinsic").get<cv::Mat>();
        depthDistortion_ = cameraParameters.at("depth_distortion").get<cv::Mat>();
        colorIntrinsics_ = cameraParameters.at("color_intrinsic").get<cv::Mat>();
        colorDistortion_ = cameraParameters.at("color_distortion").get<cv::Mat>();
    } catch (const std::exception& ex) {
        logger_->warn("Could not get camera parameters: {}", ex.what());
        return;
    }
    return;
}

}   // namespace rgbdcameras
}   // namespace sensors
}   // namespace crf
