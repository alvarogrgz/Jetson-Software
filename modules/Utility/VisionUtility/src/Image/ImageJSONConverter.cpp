/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <exception>
#include <vector>

#include <nlohmann/json.hpp>

#include "VisionUtility/Image/ImageJSONConverter.hpp"

namespace nlohmann {

cv::Mat adl_serializer<cv::Mat>::from_json(const json& j) {  // NOLINT
    std::vector<std::vector<float>> matArray;
    try {
        matArray = j.get<std::vector<std::vector<float>>>();
    } catch (std::exception& e) {
        throw e;
    }

    cv::Mat mat(matArray.size(), matArray[0].size(), CV_32F);
    for (int i=0; i < mat.rows; i++) {
        for (int j=0; j < mat.cols; j++) {
            mat.at<float>(i, j) = matArray[i][j];
        }
    }

    return mat;
}

void adl_serializer<cv::Mat>::to_json(json& j, cv::Mat t) {  // NOLINT
    std::vector<std::vector<float> > matArray;
    matArray.resize(t.rows);

    if (t.type() != CV_32F) {
        throw std::invalid_argument("Not accepted Mat type");
    }

    for (int i=0; i < t.rows; i++) {
        for (int j=0; j < t.cols; j++) {
            if (j == 0) matArray[i].resize(t.cols);
            matArray[i][j] = t.at<float>(i, j);
        }
    }

    j = matArray;
}

}  // namespace nlohmann
