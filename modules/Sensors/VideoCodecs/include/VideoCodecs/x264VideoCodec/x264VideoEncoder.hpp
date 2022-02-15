/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>

#include <x264.h>
#include "EventLogger/EventLogger.hpp"
#include "VideoCodecs/IVideoEncoder.hpp"

namespace crf {
namespace algorithms {
namespace videocodecs {

class x264VideoEncoder : public IVideoEncoder {
 public:
    x264VideoEncoder() = delete;
    x264VideoEncoder(cv::Size resolution, CompressionQuality quality, bool zeroLatency);
    ~x264VideoEncoder() override;

    bool addFrame(const cv::Mat&) override;
    bool flush() override;
    std::string getBytes(bool clearBuffer = true) override;
    CompressionQuality getCompressionQuality() override;
    cv::Size getResolution() override;

 private:
    utility::logger::EventLogger logger_;

    cv::Size resolution_;
    CompressionQuality quality_;
    bool zeroLatency_;

    x264_param_t parameters_;
    x264_picture_t picture_;
    x264_picture_t pictureOut_;
    x264_t* encoder_;

    std::mutex flowBytesMutex;
    std::string flowBytes;
};

}  // namespace videocodecs
}  // namespace algorithms
}  // namespace crf
