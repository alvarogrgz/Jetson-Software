/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo Lunghi CERN EN/STI/ECE 2020
 * 
 *  ==================================================================================================
 */

#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "VisionUtility/Image/ImageJSONConverter.hpp"
#include "EventLogger/EventLogger.hpp"

using testing::_;
using testing::Return;
using testing::NiceMock;
using testing::AtLeast;
using testing::Invoke;

class ImageJSONConverterShould : public ::testing::Test {
 protected:
    ImageJSONConverterShould() :
        logger_("ImageJSONConverterShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~ImageJSONConverterShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    crf::utility::logger::EventLogger logger_;
};

TEST_F(ImageJSONConverterShould, correctlySerializeAndDeserialize) {
    cv::Mat mat(10, 10, CV_32F);
    nlohmann::json json;

    ASSERT_NO_THROW(json["mat"] = mat);
    cv::Mat backMat;
    ASSERT_NO_THROW(backMat = json["mat"].get<cv::Mat>());
    ASSERT_EQ(mat.rows, backMat.rows);
    ASSERT_EQ(mat.cols, backMat.cols);
    ASSERT_EQ(mat.type(), backMat.type());
}

TEST_F(ImageJSONConverterShould, throwsExceptionIfNotKnownMatType) {
    cv::Mat mat(10, 10, CV_8UC3);

    nlohmann::json json;
    ASSERT_THROW(json["mat"] = mat, std::invalid_argument);
}
