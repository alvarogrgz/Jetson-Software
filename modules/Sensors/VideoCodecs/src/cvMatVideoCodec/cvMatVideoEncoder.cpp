/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <string>

#include "CommunicationUtility/StreamWriter.hpp"
#include "VideoCodecs/cvMatVideoCodec/cvMatVideoEncoder.hpp"

namespace crf {
namespace algorithms {
namespace videocodecs {

cvMatVideoEncoder::cvMatVideoEncoder() :
    logger_("cvMatVideoEncoder"),
    bytes_(),
    flushed_(false),
    nalDivider_() {
        logger_->debug("Ctor");
        char nalBytes[] = { 0x00, 0x00, 0x00, 0x01 };
        nalDivider_ = std::string(nalBytes, 4);
}

bool cvMatVideoEncoder::addFrame(const cv::Mat& frame) {
    if (flushed_) {
        logger_->warn("Encoder was already flushed");
        return false;
    }

    crf::utility::communicationutility::StreamWriter writer;
    writer.write(frame);

    bytes_.append(nalDivider_);
    bytes_.append(writer.toString());
    return true;
}

std::string cvMatVideoEncoder::getBytes(bool clearBuffer) {
    std::string retval = bytes_;
    if (clearBuffer)
        bytes_.clear();
    return retval;
}

bool cvMatVideoEncoder::flush() {
    logger_->debug("flush");
    flushed_ = true;
    return true;
}

CompressionQuality cvMatVideoEncoder::getCompressionQuality() {
    logger_->debug("getCompressionQuality");
    return CompressionQuality::Fast;
}

cv::Size cvMatVideoEncoder::getResolution() {
    logger_->debug("getResolution");
    return cv::Size();
}

}  // namespace videocodecs
}  // namespace algorithms
}  // namespace crf
