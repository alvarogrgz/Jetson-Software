/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "EventLogger/EventLogger.hpp"
#include "VideoCodecs/IVideoEncoder.hpp"

namespace crf {
namespace algorithms {
namespace videocodecs {

class JPEGVideoEncoder : public IVideoEncoder {
 public:
    JPEGVideoEncoder() = delete;
    explicit JPEGVideoEncoder(CompressionQuality quality);
    ~JPEGVideoEncoder() override = default;
    bool addFrame(const cv::Mat&) override;
    std::string getBytes(bool clearBuffer = true) override;
    bool flush() override;
    CompressionQuality getCompressionQuality() override;
    cv::Size getResolution() override;

 private:
    utility::logger::EventLogger logger_;
    CompressionQuality quality_;
    std::vector<int> compression_params_;
    std::string bytes_;
    std::string nalDelimiter_;
    bool flushed_;
};

}  // namespace videocodecs
}  // namespace algorithms
}  // namespace crf
