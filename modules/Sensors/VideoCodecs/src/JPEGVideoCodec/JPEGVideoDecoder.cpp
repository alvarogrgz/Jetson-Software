/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */


#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "CommunicationUtility/StreamReader.hpp"
#include "VideoCodecs/JPEGVideoCodec/JPEGVideoDecoder.hpp"

namespace crf {
namespace algorithms {
namespace videocodecs {

JPEGVideoDecoder::JPEGVideoDecoder() :
    logger_("JPEGVideoDecoder"),
    buffer_(),
    nalDivider_() {
        logger_->debug("CTor");

        char nalBytes[] = { 0x00, 0x00, 0x00, 0x01 };
        nalDivider_ = std::string(nalBytes, 4);
}

bool JPEGVideoDecoder::addBytes(const std::string& bytes) {
    buffer_.append(bytes);
    return true;
}

cv::Mat JPEGVideoDecoder::getFrame() {
    auto startNalIndex = buffer_.find(nalDivider_);
    if (startNalIndex == std::string::npos) {
        logger_->warn("No frames in buffer");
        return cv::Mat();
    }

    std::string frameStr = buffer_.substr(startNalIndex+nalDivider_.length(),
        buffer_.length() - startNalIndex+nalDivider_.length());

    crf::utility::communicationutility::StreamReader reader(frameStr);
    uint32_t frameSize;
    if (!reader.read(&frameSize)) {
        return cv::Mat();
    }

    if (frameStr.length() < frameSize + sizeof(uint32_t)) {
        return cv::Mat();
    }

    std::vector<char> buffer_vec(frameStr.begin()+sizeof(uint32_t),
        frameStr.begin()+sizeof(uint32_t)+frameSize);

    buffer_.erase(0, startNalIndex+frameSize+sizeof(uint32_t));
    return cv::imdecode(buffer_vec, cv::IMREAD_COLOR);
}

bool JPEGVideoDecoder::clear() {
    logger_->debug("clear()");
    buffer_.clear();
    return true;
}

}  // namespace videocodecs
}  // namespace algorithms
}  // namespace crf
