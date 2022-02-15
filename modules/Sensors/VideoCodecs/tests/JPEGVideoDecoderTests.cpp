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
#include "VideoCodecs/JPEGVideoCodec/JPEGVideoEncoder.hpp"
#include "VideoCodecs/JPEGVideoCodec/JPEGVideoDecoder.hpp"
#include "VideoCodecs/IVideoEncoder.hpp"

using testing::_;
using testing::Return;

using crf::algorithms::videocodecs::JPEGVideoEncoder;
using crf::algorithms::videocodecs::JPEGVideoDecoder;
using crf::algorithms::videocodecs::CompressionQuality;

class JPEGVideoDecoderShould: public ::testing::Test {
 protected:
    JPEGVideoDecoderShould() :
    logger_("JPEGVideoDecoderShould") {
        logger_->info("{} BEGIN",
            testing::UnitTest::GetInstance()->current_test_info()->name());
    }

     ~JPEGVideoDecoderShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    std::unique_ptr<JPEGVideoDecoder> sut_;
    std::unique_ptr<JPEGVideoEncoder> encoder_;
    crf::utility::logger::EventLogger logger_;
};

TEST_F(JPEGVideoDecoderShould, returnEmptyFrameIfNotBytesAdded) {
    sut_.reset(new JPEGVideoDecoder());

    ASSERT_TRUE(sut_->getFrame().empty());
}

TEST_F(JPEGVideoDecoderShould, correctlyDecodeFrames) {
    cv::Size resolution(100, 100);
    sut_.reset(new JPEGVideoDecoder());
    encoder_.reset(new JPEGVideoEncoder(crf::algorithms::videocodecs::CompressionQuality::Normal));

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

TEST_F(JPEGVideoDecoderShould, failsToDecodeFrameWithPartialBuffer) {
    sut_.reset(new JPEGVideoDecoder());
    encoder_.reset(new JPEGVideoEncoder(crf::algorithms::videocodecs::CompressionQuality::Normal));

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

TEST_F(JPEGVideoDecoderShould, clearTest) {
    sut_.reset(new JPEGVideoDecoder());
    encoder_.reset(new JPEGVideoEncoder(crf::algorithms::videocodecs::CompressionQuality::Normal));

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
