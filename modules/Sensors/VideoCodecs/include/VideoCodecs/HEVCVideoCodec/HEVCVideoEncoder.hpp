/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>

#include <x265.h>
#include <x265_config.h>

#include "EventLogger/EventLogger.hpp"
#include "VideoCodecs/IVideoEncoder.hpp"

namespace crf {
namespace algorithms {
namespace videocodecs {

class HEVCVideoEncoder : IVideoEncoder {
 public:
    HEVCVideoEncoder() = delete;
    HEVCVideoEncoder(cv::Size resolution, float framerate,
        CompressionQuality quality, bool zerolatency);
    ~HEVCVideoEncoder() override;

    bool addFrame(const cv::Mat&) override;
    bool flush() override;
    std::string getBytes(bool clearBuffer = true) override;
    CompressionQuality getCompressionQuality() override;
    cv::Size getResolution() override;

 private:
    utility::logger::EventLogger logger_;

    cv::Size resolution_;
    float framerate_;
    CompressionQuality quality_;

    bool flushed_;
    bool zerolatency_;

    x265_encoder* encoder_;
    x265_param* parameters_;
    x265_picture* picture_;

    std::mutex flowBytesMutex;
    std::string flowBytes;
};

}  // namespace videocodecs
}  // namespace algorithms
}  // namespace crf
