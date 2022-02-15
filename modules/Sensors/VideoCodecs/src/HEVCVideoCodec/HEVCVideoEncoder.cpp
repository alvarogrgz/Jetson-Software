/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <string>

#include <opencv2/imgproc/imgproc.hpp>

#include "VideoCodecs/HEVCVideoCodec/HEVCVideoEncoder.hpp"

namespace crf {
namespace algorithms {
namespace videocodecs {

HEVCVideoEncoder::HEVCVideoEncoder(cv::Size resolution, float framerate,
    CompressionQuality quality, bool zerolatency) :
    logger_("HEVCVideoEncoder"),
    resolution_(resolution),
    framerate_(framerate),
    quality_(quality),
    flushed_(false),
    zerolatency_(zerolatency) {
        logger_->debug("CTor");
        if ((resolution.width == 0) ||
            (resolution.height == 0)) {
                throw std::runtime_error("Wrong resolution");
            }

        parameters_ = x265_param_alloc();

        std::string tune = zerolatency ? "zerolatency" : "ssim";

        if (x265_param_default_preset(parameters_,
            std::to_string(quality).c_str(), tune.c_str()) < 0) {
            throw std::runtime_error("Could not set x265 parameters");
        }

        std::string resolutionString =
            std::to_string(resolution.width) + "x" + std::to_string(resolution.height);
        if (x265_param_parse(parameters_, "input-res", resolutionString.c_str()) < 0) {
            throw std::runtime_error("Could not set x265 resolution");
        }

        if (x265_param_parse(parameters_, "fps", std::to_string(framerate).c_str()) < 0) {
            throw std::runtime_error("Could not set x265 framerate");
        }

        if (x265_param_parse(parameters_, "log-level", "0") < 0) {
            throw std::runtime_error("Could not set x265 framerate");
        }

        if (x265_param_parse(parameters_, "repeat-headers", "1") < 0) {
            throw std::runtime_error("Could not set x265 repeat-headers option");
        }

        encoder_ = x265_encoder_open(parameters_);

        picture_ = x265_picture_alloc();
        x265_picture_init(parameters_, picture_);
}

HEVCVideoEncoder::~HEVCVideoEncoder() {
    logger_->debug("DTor");
    x265_picture_free(picture_);
    x265_param_free(parameters_);
    x265_encoder_close(encoder_);
    x265_cleanup();
}

bool HEVCVideoEncoder::addFrame(const cv::Mat& frame) {
    if ((frame.size().width != resolution_.width) ||
        (frame.size().height != resolution_.height)) {
            logger_->warn("Frame and encoders have different resolution");
            return false;
    }

    if (flushed_) {
        logger_->warn("Encoder has been already flushed");
        return false;
    }

    cv::Mat convertedFrame;

    cv::cvtColor(frame, convertedFrame, cv::COLOR_BGR2YUV_I420);

    picture_->stride[0] = frame.size().width;
    picture_->stride[1] = picture_->stride[0] >> x265_cli_csps[picture_->colorSpace].width[1];
    picture_->stride[2] = picture_->stride[0] >> x265_cli_csps[picture_->colorSpace].width[2];
    picture_->planes[0] = convertedFrame.data;
    picture_->planes[1] = reinterpret_cast<char*>(picture_->planes[0]) + picture_->stride[0] * frame.size().height;     // NOLINT
    picture_->planes[2] = reinterpret_cast<char*>(picture_->planes[1]) + picture_->stride[1] * (frame.size().height >> x265_cli_csps[picture_->colorSpace].height[1]);     // NOLINT
    x265_nal *pp_nal;
    uint32_t pi_nal = 0;
    x265_picture *pic_out = x265_picture_alloc();
    if (x265_encoder_encode(encoder_, &pp_nal, &pi_nal, picture_, pic_out) < 0) {
        logger_->error("Could not encode picture");
        return false;
    }

    if (pi_nal > 0) {
        for (uint32_t i=0; i < pi_nal; i++) {
            std::lock_guard<std::mutex> lock(flowBytesMutex);
            std::string pp_nal_string(reinterpret_cast<char*>(pp_nal[i].payload),
                pp_nal[i].sizeBytes);

            flowBytes.append(pp_nal_string);
        }
    }

    x265_picture_free(pic_out);

    return true;
}

bool HEVCVideoEncoder::flush() {
    logger_->debug("flush()");

    if (zerolatency_) {
        logger_->warn("Can't flush in zerolatency mode");
        return false;
    }

    std::lock_guard<std::mutex> lock(flowBytesMutex);

    x265_nal *pp_nal;
    uint32_t pi_nal = 0;
    x265_picture *pic_out = x265_picture_alloc();
    while (x265_encoder_encode(encoder_, &pp_nal, &pi_nal, NULL, pic_out) > 0) {
        if (pi_nal > 0) {
            for (uint32_t i=0; i < pi_nal; i++) {
                std::string pp_nal_string(reinterpret_cast<char*>(pp_nal[i].payload),
                    pp_nal[i].sizeBytes);

                flowBytes.append(pp_nal_string);
            }
        }
    }

    flushed_ = true;

    return true;
}

std::string HEVCVideoEncoder::getBytes(bool clearBuffer) {
    std::lock_guard<std::mutex> lock(flowBytesMutex);
    std::string returnString(flowBytes);

    if (clearBuffer)
        flowBytes.clear();

    return returnString;
}

CompressionQuality HEVCVideoEncoder::getCompressionQuality() {
    logger_->debug("getCompressionQuality");
    return quality_;
}

cv::Size HEVCVideoEncoder::getResolution() {
    logger_->debug("getResolution");
    return resolution_;
}

}  // namespace videocodecs
}  // namespace algorithms
}  // namespace crf
