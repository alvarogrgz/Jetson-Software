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

namespace crf {
namespace algorithms {
namespace videocodecs {

/**
 * @brief IVideoDecoder interface
 * The IVideoDecoder interface is meant to standardise video decoding methods
 * It allows to add encoded bytes and to obtain cv::Mat frames
 */
class IVideoDecoder {
 public:
    virtual ~IVideoDecoder() = default;
    /**
     * @brief Add bytes to the decoder
     * @param bytes bytes to add
     * @return true bytes were correctly added
     * @return false otherwise
     */
    virtual bool addBytes(const std::string& bytes) = 0;

    /**
     * @brief Get a frame from the decoder
     * It returns an empty cv::Mat if the stored bytes did not contain a valid frame
     * @return cv::Mat the decoded frame
     */
    virtual cv::Mat getFrame() = 0;

    /**
     * @brief Clears the current buffer
     * 
     * @return true if the buffer was correctly cleared
     * @return false otherwise
     */
    virtual bool clear() = 0;
};

}  // namespace videocodecs
}  // namespace algorithms
}  // namespace crf
