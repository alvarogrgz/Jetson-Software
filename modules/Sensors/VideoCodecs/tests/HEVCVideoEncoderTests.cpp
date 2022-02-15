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
#include "VideoCodecs/HEVCVideoCodec/HEVCVideoEncoder.hpp"
#include "VideoCodecs/IVideoEncoder.hpp"

using testing::_;
using testing::Return;

using crf::algorithms::videocodecs::HEVCVideoEncoder;
using crf::algorithms::videocodecs::CompressionQuality;

class HEVCVideoEncoderShould: public ::testing::Test {
 protected:
    HEVCVideoEncoderShould() :
    logger_("HEVCVideoEncoderShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
    }

     ~HEVCVideoEncoderShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    std::unique_ptr<HEVCVideoEncoder> sut_;
    crf::utility::logger::EventLogger logger_;
};

TEST_F(HEVCVideoEncoderShould, throwsExceptionWithBadResolutionParameter) {
    cv::Size resolution(0, 0);

    ASSERT_THROW(sut_.reset(new HEVCVideoEncoder(resolution, 15,
        CompressionQuality::Fast, false)), std::runtime_error);

    resolution = cv::Size(1280, 720);

    ASSERT_THROW(sut_.reset(new HEVCVideoEncoder(resolution, 0,
        CompressionQuality::Fast, false)), std::runtime_error);
}

TEST_F(HEVCVideoEncoderShould, returnEmptyBufferIfNoImageCompressed) {
    cv::Size resolution(1280, 720);

    ASSERT_NO_THROW(sut_.reset(new HEVCVideoEncoder(resolution, 15,
        CompressionQuality::Fast, false)));

    ASSERT_TRUE(sut_->getBytes().empty());
}

// Test disabled due to a constant file. Needs to be checked
TEST_F(HEVCVideoEncoderShould, DISABLED_cantFlushIfZeroLatency) {
    cv::Size resolution(100, 100);

    ASSERT_NO_THROW(sut_.reset(new HEVCVideoEncoder(resolution, 15,
        CompressionQuality::Fast, true)));

    cv::Mat frame(resolution, CV_8UC3);
    cv::randu(frame, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
    ASSERT_TRUE(sut_->addFrame(frame));
    ASSERT_FALSE(sut_->flush());
}

TEST_F(HEVCVideoEncoderShould, correctlyEmptyBufferOnGetBytes) {
    cv::Size resolution(100, 100);

    ASSERT_NO_THROW(sut_.reset(new HEVCVideoEncoder(resolution, 15,
        CompressionQuality::Fast, false)));

    cv::Mat frame(resolution, CV_8UC3);
    cv::randu(frame, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
    ASSERT_TRUE(sut_->addFrame(frame));

    ASSERT_TRUE(sut_->flush());

    ASSERT_FALSE(sut_->getBytes(false).empty());
    ASSERT_FALSE(sut_->getBytes().empty());
    ASSERT_TRUE(sut_->getBytes().empty());
}

TEST_F(HEVCVideoEncoderShould, cantAddFrameAgainAfterFlush) {
    cv::Size resolution(100, 100);

    ASSERT_NO_THROW(sut_.reset(new HEVCVideoEncoder(resolution, 15,
        CompressionQuality::Fast, false)));

    cv::Mat frame(resolution, CV_8UC3);
    cv::randu(frame, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
    ASSERT_TRUE(sut_->addFrame(frame));
    ASSERT_TRUE(sut_->flush());
    ASSERT_FALSE(sut_->addFrame(frame));
}

TEST_F(HEVCVideoEncoderShould, returnFalseOnWrongFrameAdd) {
    cv::Size resolution(1280, 720);

    ASSERT_NO_THROW(sut_.reset(new HEVCVideoEncoder(resolution, 15,
        CompressionQuality::Fast, false)));

    cv::Mat frame(1200, 820, CV_8UC3);
    cv::randu(frame, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
    ASSERT_FALSE(sut_->addFrame(frame));

    ASSERT_TRUE(sut_->getBytes().empty());
}

TEST_F(HEVCVideoEncoderShould, correctlyEncodeStreamTest) {
    cv::Size resolution(100, 100);

    ASSERT_NO_THROW(sut_.reset(new HEVCVideoEncoder(resolution, 15,
        CompressionQuality::Ultrafast, false)));

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

TEST_F(HEVCVideoEncoderShould, getResolution) {
    cv::Size resolution(1920, 1080);

    ASSERT_NO_THROW(sut_.reset(new HEVCVideoEncoder(resolution, 15,
        CompressionQuality::Ultrafast, false)));

    ASSERT_EQ(sut_->getResolution(), resolution);
}

TEST_F(HEVCVideoEncoderShould, getQuality) {
    cv::Size resolution(1920, 1080);

    ASSERT_NO_THROW(sut_.reset(new HEVCVideoEncoder(resolution, 15,
        CompressionQuality::Ultrafast, false)));

    ASSERT_EQ(sut_->getCompressionQuality(), CompressionQuality::Ultrafast);
}
