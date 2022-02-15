/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo Lunghi CERN EN/STI/ECE
 * 
 *  ==================================================================================================
 */

#include <memory>
#include <string>

#include <opencv2/opencv.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <VideoCodecs/x264VideoCodec/x264VideoEncoder.hpp>
#include <VideoCodecs/IVideoEncoder.hpp>

using testing::_;
using testing::Return;

using crf::algorithms::videocodecs::CompressionQuality;
using crf::algorithms::videocodecs::IVideoEncoder;
using crf::algorithms::videocodecs::x264VideoEncoder;

class x264VideoEncoderShould: public ::testing::Test {
 protected:
    x264VideoEncoderShould() :
        logger_("x264VideoEncoderShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~x264VideoEncoderShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }
    std::unique_ptr<IVideoEncoder> sut_;
    crf::utility::logger::EventLogger logger_;
};

TEST_F(x264VideoEncoderShould, throwsExceptionWithBadResolutionParameter) {
    cv::Size resolution(0, 0);

    ASSERT_THROW(sut_.reset(new x264VideoEncoder(resolution,
        CompressionQuality::Fast, false)), std::runtime_error);
}

TEST_F(x264VideoEncoderShould, returnEmptyBufferIfNoImageCompressed) {
    cv::Size resolution(1280, 720);

    ASSERT_NO_THROW(sut_.reset(new x264VideoEncoder(resolution,
        CompressionQuality::Fast, false)));

    ASSERT_TRUE(sut_->getBytes().empty());
}

TEST_F(x264VideoEncoderShould, cantFlushIfZeroLatency) {
    cv::Size resolution(50, 50);

    ASSERT_NO_THROW(sut_.reset(new x264VideoEncoder(resolution,
        CompressionQuality::Fast, true)));

    cv::Mat frame(resolution, CV_8UC3);
    cv::randu(frame, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
    ASSERT_TRUE(sut_->addFrame(frame));
    ASSERT_FALSE(sut_->flush());
}

TEST_F(x264VideoEncoderShould, correctlyEmptyBufferOnGetBytes) {
    cv::Size resolution(50, 50);

    ASSERT_NO_THROW(sut_.reset(new x264VideoEncoder(resolution,
        CompressionQuality::Fast, false)));

    cv::Mat frame(resolution, CV_8UC3);
    cv::randu(frame, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
    ASSERT_TRUE(sut_->addFrame(frame));
    ASSERT_TRUE(sut_->flush());
    ASSERT_FALSE(sut_->getBytes(false).empty());
    ASSERT_FALSE(sut_->getBytes().empty());
    ASSERT_TRUE(sut_->getBytes().empty());
}

TEST_F(x264VideoEncoderShould, cantAddFrameAgainAfterFlush) {
    cv::Size resolution(50, 50);

    ASSERT_NO_THROW(sut_.reset(new x264VideoEncoder(resolution,
        CompressionQuality::Fast, false)));

    cv::Mat frame(resolution, CV_8UC3);
    cv::randu(frame, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
    ASSERT_TRUE(sut_->addFrame(frame));
    ASSERT_TRUE(sut_->flush());
    ASSERT_FALSE(sut_->getBytes().empty());
    ASSERT_FALSE(sut_->addFrame(frame));
}

TEST_F(x264VideoEncoderShould, returnFalseOnWrongFrameAdd) {
    cv::Size resolution(1280, 720);

    ASSERT_NO_THROW(sut_.reset(new x264VideoEncoder(resolution,
        CompressionQuality::Fast, false)));

    cv::Mat frame(1200, 820, CV_8UC3);
    ASSERT_FALSE(sut_->addFrame(frame));

    ASSERT_TRUE(sut_->getBytes().empty());
}

TEST_F(x264VideoEncoderShould, correctlyEncodeStreamTest) {
    cv::Size resolution(100, 100);

    ASSERT_NO_THROW(sut_.reset(new x264VideoEncoder(resolution,
        CompressionQuality::Normal, false)));

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

TEST_F(x264VideoEncoderShould, getResolution) {
    cv::Size resolution(1920, 1080);

    ASSERT_NO_THROW(sut_.reset(new x264VideoEncoder(resolution,
        CompressionQuality::Normal, false)));

    ASSERT_EQ(sut_->getResolution(), resolution);
}

TEST_F(x264VideoEncoderShould, getQuality) {
    cv::Size resolution(1920, 1080);

    ASSERT_NO_THROW(sut_.reset(new x264VideoEncoder(resolution,
        CompressionQuality::Normal, false)));

    ASSERT_EQ(sut_->getCompressionQuality(), CompressionQuality::Normal);
}
