/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#pragma once

#include <chrono>
#include <gmock/gmock.h>
#include <memory>
#include <thread>
#include <vector>

#include "Cameras/ICamera.hpp"
#include "Cameras/CameraMock.hpp"

class ICameraSimulator {
 public:
    explicit ICameraSimulator(std::shared_ptr<crf::sensors::cameras::CameraMock> mock) :
        initializeDelay(std::chrono::milliseconds(1000)),
        failsToInitialize(false),
        failsToGetFrame(false),
        failsSetResolution(false),
        failsSetFramerate(false),
        canZoom(true),
        canMove(true),
        canSetExposure(true),
        canSetShutterSpeed(true),
        canSetFocus(true),
        canFocus(true),
        canSetIso(true),
        mock_(mock),
        initialized_(false),
        resolution_(0, 0),
        framerate_(10),
        zoom_(0),
        pan_(0),
        tilt_(0),
        exposure_(0),
        speed_(0),
        focusMode_(crf::sensors::cameras::ICamera::FocusModes::Auto),
        focus_(0),
        iso_(1200) {
        ON_CALL(*mock_, initialize()).WillByDefault(testing::Invoke([this]() {
            if (failsToInitialize) return false;
            if (initialized_) return false;
            std::this_thread::sleep_for(initializeDelay);
            initialized_ = true;
            return true;
        }));

        ON_CALL(*mock_, deinitialize()).WillByDefault(testing::Invoke([this]() {
            if (failsToInitialize) return false;
            if (!initialized_) return false;
            std::this_thread::sleep_for(initializeDelay);
            initialized_ = false;
            return true;
        }));

        ON_CALL(*mock_, getFrame()).WillByDefault(testing::Invoke([this]() {
            if (!initialized_) return cv::Mat();
            if (failsToGetFrame) return cv::Mat();
            auto mat = cv::Mat(resolution_, CV_8UC3);
            cv::randu(mat, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
            return mat;
        }));

        ON_CALL(*mock_, getAvailableResolutions()).WillByDefault(testing::Invoke([this]() {
            return std::vector<cv::Size>({
                cv::Size(100, 100),
                cv::Size(150, 150),
                cv::Size(640, 480),
                cv::Size(800, 600),
                cv::Size(1280, 720),
                cv::Size(1920, 1080)
            });
        }));

        ON_CALL(*mock_, getAvailableFramerates(testing::_)).WillByDefault(testing::Invoke([this](const cv::Size& resolution) {
            return std::vector<int>({ 5, 10, 15, 20, 25, 30 });
        }));

        ON_CALL(*mock_, setResolution(testing::_)).WillByDefault(testing::Invoke([this](const cv::Size& res) {  // NOLINT
            if (failsSetResolution) return false;
            if (!initialized_) return false;

            resolution_ = res;
            return true;
        }));

        ON_CALL(*mock_, setFramerate(testing::_)).WillByDefault(testing::Invoke([this](int fps) {  // NOLINT
            if (failsSetFramerate) return false;
            if (!initialized_) return false;

            framerate_ = fps;
            return true;
        }));

        ON_CALL(*mock_, setZoom(testing::_)).WillByDefault(testing::Invoke([this](float zoom) {  // NOLINT
            if (!canZoom) return false;
            if (!initialized_) return false;

            zoom_ = zoom;
            return true;
        }));

        ON_CALL(*mock_, setPosition(testing::_, testing::_)).WillByDefault(testing::Invoke([this](float pan, float tilt) {  // NOLINT
            if (!canMove) return false;
            if (!initialized_) return false;

            pan_ = pan;
            tilt_ = tilt;
            return true;
        }));

        ON_CALL(*mock_, setExposure(testing::_)).WillByDefault(testing::Invoke([this](float exposure) {  // NOLINT
            if (!canSetExposure) return false;
            if (!initialized_) return false;

            exposure_ = exposure;
            return true;
        }));

        ON_CALL(*mock_, setShutterSpeed(testing::_)).WillByDefault(testing::Invoke([this](float speed) {  // NOLINT
            if (!canSetShutterSpeed) return false;
            if (!initialized_) return false;

            speed_ = speed;
            return true;
        }));

        ON_CALL(*mock_, setFocusMode(testing::_)).WillByDefault(testing::Invoke([this](crf::sensors::cameras::ICamera::FocusModes focusMode) {  // NOLINT
            if (!canSetFocus) return false;
            if (!initialized_) return false;

            focusMode_ = focusMode;
            return true;
        }));

        ON_CALL(*mock_, setFocus(testing::_)).WillByDefault(testing::Invoke([this](float focus) {  // NOLINT
            if (!canFocus) return false;
            if (!initialized_) return false;

            focus_ = focus;
            return true;
        }));

        ON_CALL(*mock_, setISO(testing::_)).WillByDefault(testing::Invoke([this](float iso) {  // NOLINT
            if (!canSetIso) return false;
            if (!initialized_) return false;

            iso_ = iso;
            return true;
        }));

        ON_CALL(*mock_, getResolution()).WillByDefault(testing::Return(resolution_));
        ON_CALL(*mock_, getFramerate()).WillByDefault(testing::Return(framerate_));
        ON_CALL(*mock_, getZoom()).WillByDefault(testing::Invoke([this]() -> boost::optional<float> {  // NOLINT
            if (!canZoom) return boost::none;
            return zoom_;
        }));

        ON_CALL(*mock_, getPan()).WillByDefault(testing::Invoke([this]() -> boost::optional<float> {  // NOLINT
            if (!canMove) return boost::none;
            return pan_;
        }));
        ON_CALL(*mock_, getTilt()).WillByDefault(testing::Invoke([this]() -> boost::optional<float> {  // NOLINT
            if (!canMove) return boost::none;
            return tilt_;
        }));
        ON_CALL(*mock_, getExposure()).WillByDefault(testing::Invoke([this]() -> boost::optional<float> {  // NOLINT
            if (!canSetExposure) return boost::none;
            return exposure_;
        }));
        ON_CALL(*mock_, getShutterSpeed()).WillByDefault(testing::Invoke([this]() -> boost::optional<float> {  // NOLINT
            if (!canSetShutterSpeed) return boost::none;
            return speed_;
        }));
        ON_CALL(*mock_, getFocusMode()).WillByDefault(testing::Invoke([this]() -> boost::optional<crf::sensors::cameras::ICamera::FocusModes> {  // NOLINT
            if (!canSetFocus) return boost::none;
            return focusMode_;
        }));
        ON_CALL(*mock_, getFocus()).WillByDefault(testing::Invoke([this]() -> boost::optional<float> {  // NOLINT
            if (!canFocus) return boost::none;
            return focus_;
        }));
        ON_CALL(*mock_, getISO()).WillByDefault(testing::Invoke([this]() -> boost::optional<int> {  // NOLINT
            if (!canSetIso) return boost::none;
            return iso_;
        }));
    }

    std::chrono::milliseconds initializeDelay;
    bool failsToInitialize;
    bool failsToGetFrame;

    bool failsSetResolution;
    bool failsSetFramerate;
    bool canZoom;
    bool canMove;
    bool canSetExposure;
    bool canSetShutterSpeed;
    bool canSetFocus;
    bool canFocus;
    bool canSetIso;

 private:
    std::shared_ptr<crf::sensors::cameras::CameraMock> mock_;

    bool initialized_;

    cv::Size resolution_;
    int framerate_;

    float zoom_;
    float pan_, tilt_;
    float exposure_;
    float speed_;
    crf::sensors::cameras::ICamera::FocusModes focusMode_;
    float focus_;
    int iso_;
};
