/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/STI/ECE
 *
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>

#include "EventLogger/EventLogger.hpp"
#include "VideoCodecs/cvMatVideoCodec/cvMatVideoEncoder.hpp"
#include "VideoCodecs/IVideoEncoder.hpp"

using testing::_;
using testing::Return;

using crf::algorithms::videocodecs::cvMatVideoEncoder;
using crf::algorithms::videocodecs::CompressionQuality;

class cvMatVideoEncoderShould: public ::testing::Test {
 protected:
    cvMatVideoEncoderShould() :
    logger_("cvMatVideoEncoderShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
    }

     ~cvMatVideoEncoderShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    std::unique_ptr<cvMatVideoEncoder> sut_;
    crf::utility::logger::EventLogger logger_;
};

TEST_F(cvMatVideoEncoderShould, returnEmptyBufferIfNoImageCompressed) {
    cv::Size resolution(1280, 720);

    ASSERT_NO_THROW(sut_.reset(new cvMatVideoEncoder()));

    ASSERT_TRUE(sut_->getBytes().empty());
}

TEST_F(cvMatVideoEncoderShould, correctlyEmptyBufferOnGetBytes) {
    cv::Size resolution(1280, 720);

    ASSERT_NO_THROW(sut_.reset(new cvMatVideoEncoder()));

    cv::Mat frame(resolution, CV_8UC3);
    cv::randu(frame, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
    ASSERT_TRUE(sut_->addFrame(frame));

    ASSERT_TRUE(sut_->flush());

    ASSERT_FALSE(sut_->getBytes(false).empty());
    ASSERT_FALSE(sut_->getBytes().empty());
    ASSERT_TRUE(sut_->getBytes().empty());
}

TEST_F(cvMatVideoEncoderShould, cantAddFrameAgainAfterFlush) {
    cv::Size resolution(1280, 720);

    ASSERT_NO_THROW(sut_.reset(new cvMatVideoEncoder()));

    cv::Mat frame(resolution, CV_8UC3);
    cv::randu(frame, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
    ASSERT_TRUE(sut_->addFrame(frame));
    ASSERT_TRUE(sut_->flush());
    ASSERT_FALSE(sut_->addFrame(frame));
}

TEST_F(cvMatVideoEncoderShould, correctlyEncodeStreamTest) {
    cv::Size resolution(100, 100);

    ASSERT_NO_THROW(sut_.reset(new cvMatVideoEncoder()));

    for (uint32_t i=0; i < 30; i++) {
        cv::Mat frame(resolution, CV_8UC3);
        cv::randu(frame, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
        ASSERT_TRUE(sut_->addFrame(frame));
    }

    ASSERT_TRUE(sut_->flush());
    std::string buffer = sut_->getBytes();
    ASSERT_FALSE(buffer.empty());
    ASSERT_GT(buffer.length(), 0);
}
