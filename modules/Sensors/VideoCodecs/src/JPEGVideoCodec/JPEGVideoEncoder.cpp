/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <string>
#include <vector>

#include "VideoCodecs/JPEGVideoCodec/JPEGVideoEncoder.hpp"

namespace crf {
namespace algorithms {
namespace videocodecs {

JPEGVideoEncoder::JPEGVideoEncoder(CompressionQuality quality) :
    logger_("JPEGVideoEncoder"),
    quality_(quality),
    compression_params_(),
    bytes_(),
    nalDelimiter_(),
    flushed_(false) {
        logger_->debug("Ctor");
        uint32_t compression = (static_cast<uint32_t>(quality) * 10) + 10;
        compression_params_.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params_.push_back(compression);
        char nalBytes[] = { 0x00, 0x00, 0x00, 0x01 };
        nalDelimiter_ = std::string(nalBytes, 4);
}

bool JPEGVideoEncoder::addFrame(const cv::Mat& frame) {
    if (flushed_) {
        logger_->warn("Encoder has been already flushed");
        return false;
    }
    std::vector<uchar> buf;
    cv::imencode(".jpeg", frame, buf, compression_params_);
    bytes_.append(nalDelimiter_);
    uint32_t bufSize = buf.size();
    bytes_.append(std::string(reinterpret_cast<char*>(&bufSize), sizeof(uint32_t)));
    bytes_.append(std::string(buf.begin(), buf.end()));
    return true;
}

std::string JPEGVideoEncoder::getBytes(bool clearBuffer) {
    std::string retval = bytes_;
    if (clearBuffer)
        bytes_.clear();
    return retval;
}

bool JPEGVideoEncoder::flush() {
    logger_->debug("flush");
    flushed_ = true;
    return true;
}

CompressionQuality JPEGVideoEncoder::getCompressionQuality() {
    logger_->debug("getCompressionQuality");
    return quality_;
}

cv::Size JPEGVideoEncoder::getResolution() {
    logger_->debug("getResolution");
    return cv::Size();
}

}  // namespace videocodecs
}  // namespace algorithms
}  // namespace crf
