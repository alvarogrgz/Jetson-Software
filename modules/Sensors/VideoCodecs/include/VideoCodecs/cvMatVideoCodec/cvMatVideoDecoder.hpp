/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <opencv2/core/mat.hpp>

#include "EventLogger/EventLogger.hpp"
#include "VideoCodecs/IVideoDecoder.hpp"

namespace crf {
namespace algorithms {
namespace videocodecs {

class cvMatVideoDecoder : public IVideoDecoder {
 public:
    cvMatVideoDecoder();
    ~cvMatVideoDecoder() override = default;

    bool addBytes(const std::string& bytes) override;
    cv::Mat getFrame() override;
    bool clear() override;
 private:
    utility::logger::EventLogger logger_;
    std::string buffer_;

    std::string nalDivider_;
};

}  // namespace videocodecs
}  // namespace algorithms
}  // namespace crf
