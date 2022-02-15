/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <opencv2/core/mat.hpp>

namespace crf {
namespace algorithms {
namespace videocodecs {

enum CompressionQuality {
        Ultrafast = 0, Superfast = 1, Veryfast = 2, Faster = 3,
        Fast = 4, Normal = 5, Slow = 6, Slower = 7, VerySlow = 8 };

/**
 * @brief VideoEncoder interface
 * The VideoEncoder interface is meant to standardise video encoding methods
 * It allows to add cv::Mat frames and obtain encoded bytes
 */
class IVideoEncoder {
 public:
    virtual ~IVideoEncoder() = default;
    /**
     * @brief Add a frame to the encoder
     * @return true the frame was correctly added
     * @return false otherwise
     */
    virtual bool addFrame(const cv::Mat&) = 0;
    /**
     * @brief Get the compressed bytes
     * @param clearBuffer the buffer compressed bytes is cleared
     * @return std::string the compressed bytes
     */
    virtual std::string getBytes(bool clearBuffer = true) = 0;
    /**
     * @brief Flushes the video encoder algorithm
     * After flush, it is not anymore possible to add frames.
     * @return true flush was successful
     * @return false otherwise
     */
    virtual bool flush() = 0;
    /**
     * @brief Get the Compression Quality
     * @return CompressionQuality 
     */
    virtual CompressionQuality getCompressionQuality() = 0;
    /**
     * @brief Get the encoding resolution
     * @return cv::Size 
     */
    virtual cv::Size getResolution() = 0;
};

}  // namespace videocodecs
}  // namespace algorithms
}  // namespace crf
