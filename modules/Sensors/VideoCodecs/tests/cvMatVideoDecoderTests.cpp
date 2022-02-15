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
#include "VideoCodecs/cvMatVideoCodec/cvMatVideoDecoder.hpp"
#include "VideoCodecs/IVideoEncoder.hpp"

using testing::_;
using testing::Return;

using crf::algorithms::videocodecs::cvMatVideoEncoder;
using crf::algorithms::videocodecs::cvMatVideoDecoder;
using crf::algorithms::videocodecs::CompressionQuality;

class cvMatVideoDecoderShould: public ::testing::Test {
 protected:
    cvMatVideoDecoderShould() :
    logger_("cvMatVideoDecoderShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
    }

     ~cvMatVideoDecoderShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    std::unique_ptr<cvMatVideoDecoder> sut_;
    std::unique_ptr<cvMatVideoEncoder> encoder_;
    crf::utility::logger::EventLogger logger_;
};

TEST_F(cvMatVideoDecoderShould, returnEmptyFrameIfNotBytesAdded) {
    sut_.reset(new cvMatVideoDecoder());

    ASSERT_TRUE(sut_->getFrame().empty());
}

TEST_F(cvMatVideoDecoderShould, correctlyDecodeFrames) {
    cv::Size resolution(100, 100);
    sut_.reset(new cvMatVideoDecoder());
    encoder_.reset(new cvMatVideoEncoder());

    for (int i=0; i < 10; i++) {
        cv::Size resolution(100+i, 100+i);
        cv::Mat frame(resolution, CV_8UC3);
        cv::randu(frame, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
        ASSERT_TRUE(encoder_->addFrame(frame));
        auto bytes = encoder_->getBytes();
        ASSERT_TRUE(sut_->addBytes(bytes));
        auto decodedFrame = sut_->getFrame();
        ASSERT_FALSE(decodedFrame.empty());
        ASSERT_EQ(decodedFrame.cols, resolution.width);
        ASSERT_EQ(decodedFrame.rows, resolution.height);
    }
}

TEST_F(cvMatVideoDecoderShould, failsToDecodeFrameWithPartialBuffer) {
    sut_.reset(new cvMatVideoDecoder());
    encoder_.reset(new cvMatVideoEncoder());

    for (int i=0; i < 10; i++) {
        cv::Size resolution(100+i, 100+i);
        cv::Mat frame(resolution, CV_8UC3);
        cv::randu(frame, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
        ASSERT_TRUE(encoder_->addFrame(frame));
        auto bytes = encoder_->getBytes();
        int length = bytes.length();
        ASSERT_TRUE(sut_->addBytes(bytes.substr(0, length/2)));
        auto decodedFrame = sut_->getFrame();
        ASSERT_TRUE(decodedFrame.empty());
        ASSERT_TRUE(sut_->addBytes(bytes.substr(length/2, bytes.length() - length/2)));
        decodedFrame = sut_->getFrame();
        ASSERT_FALSE(decodedFrame.empty());
        ASSERT_EQ(decodedFrame.cols, resolution.width);
        ASSERT_EQ(decodedFrame.rows, resolution.height);
    }
}

TEST_F(cvMatVideoDecoderShould, clearTest) {
    sut_.reset(new cvMatVideoDecoder());
    encoder_.reset(new cvMatVideoEncoder());

    for (int i=0; i < 10; i++) {
        cv::Size resolution(100+i, 100+i);
        cv::Mat frame(resolution, CV_8UC3);
        cv::randu(frame, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
        ASSERT_TRUE(encoder_->addFrame(frame));
        auto bytes = encoder_->getBytes();
        int length = bytes.length();
        if (i%2 == 0) {
            ASSERT_TRUE(sut_->addBytes(bytes.substr(0, length/2)));
            continue;
        }
        auto decodedFrame = sut_->getFrame();
        ASSERT_TRUE(decodedFrame.empty());
        ASSERT_TRUE(sut_->clear());
        ASSERT_TRUE(sut_->addBytes(bytes));
        decodedFrame = sut_->getFrame();
        ASSERT_FALSE(decodedFrame.empty());
        ASSERT_EQ(decodedFrame.cols, resolution.width);
        ASSERT_EQ(decodedFrame.rows, resolution.height);
    }
}
