#pragma once

/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Camarero Vera CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <gmock/gmock.h>

#include <string>

#include "VideoCodecs/IVideoEncoder.hpp"

namespace crf {
namespace algorithms {
namespace videocodecs {

class VideoEncoderMock : public IVideoEncoder {
 public:
  MOCK_METHOD1(addFrame,
      bool(const cv::Mat&));
  MOCK_METHOD1(getBytes,
      std::string(bool));
  MOCK_METHOD0(flush,
      bool());
  MOCK_METHOD0(getCompressionQuality,
      CompressionQuality());
  MOCK_METHOD0(getResolution,
      cv::Size());
};

}  // namespace videocodecs
}  // namespace algorithms
}  // namespace crf
