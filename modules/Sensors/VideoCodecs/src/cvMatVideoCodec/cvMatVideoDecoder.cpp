/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <string>

#include "CommunicationUtility/StreamReader.hpp"
#include "VideoCodecs/cvMatVideoCodec/cvMatVideoDecoder.hpp"

namespace crf {
namespace algorithms {
namespace videocodecs {

cvMatVideoDecoder::cvMatVideoDecoder() :
    logger_("cvMatVideoDecoder"),
    buffer_(),
    nalDivider_() {
        logger_->debug("CTor");

        char nalBytes[] = { 0x00, 0x00, 0x00, 0x01 };
        nalDivider_ = std::string(nalBytes, 4);
}

bool cvMatVideoDecoder::addBytes(const std::string& bytes) {
    buffer_.append(bytes);
    return true;
}

cv::Mat cvMatVideoDecoder::getFrame() {
    auto startNalIndex = buffer_.find(nalDivider_);
    if (startNalIndex == std::string::npos) {
        logger_->warn("No frames in buffer");
        return cv::Mat();
    }

    auto endNalIndex = buffer_.find(nalDivider_, startNalIndex + nalDivider_.length());
    if (endNalIndex == std::string::npos) {
        endNalIndex = buffer_.length();
    }

    std::string frameStr = buffer_.substr(startNalIndex+nalDivider_.length(),
        endNalIndex - startNalIndex+nalDivider_.length());

    crf::utility::communicationutility::StreamReader reader(frameStr);
    cv::Mat frame;
    if (!reader.read(&frame)) {
        return cv::Mat();
    }

    buffer_.erase(0, endNalIndex);
    return frame;
}

bool cvMatVideoDecoder::clear() {
    logger_->debug("clear()");
    buffer_.clear();
    return true;
}

}  // namespace videocodecs
}  // namespace algorithms
}  // namespace crf
