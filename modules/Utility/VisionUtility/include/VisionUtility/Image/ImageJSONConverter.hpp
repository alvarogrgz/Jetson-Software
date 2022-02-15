/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

namespace nlohmann {

template <>
struct adl_serializer<cv::Mat> {
    static cv::Mat from_json(const json& j);
    static void to_json(json& j, cv::Mat t);  // NOLINT
};

}  // namespace nlohmann
