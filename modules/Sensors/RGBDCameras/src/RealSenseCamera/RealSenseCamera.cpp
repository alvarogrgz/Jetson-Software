/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *         Carlos Prados Sesmero CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#define REALSENSE_FILTER_MAX_RANGE 4.0f
#define FRAME_TIMEOUT 5000

#include <algorithm>
#include <librealsense2/rsutil.h>
#include <librealsense2/rs.h>
#include <librealsense2/rs_advanced_mode.h>
#include <limits>
#include <string>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>

#include "RGBDCameras/RealSenseCamera/RealSenseCamera.hpp"

namespace crf {
namespace sensors {
namespace rgbdcameras {

RealSenseCamera::RealSenseCamera(const std::string& serialNumber) :
    logger_("RealSenseCamera"),
    serialNumber_(serialNumber),
    initialized_(false),
    pipeMutex_(),
    cfg_(),
    pipe_(),
    profile_(),
    device_(),
    extrinsic_(),
    colorSensor_(),
    depthSensor_(),
    motionSensor_(),
    activeColorStream_(),
    activeDepthStream_(),
    alignToColor_(new rs2::align(RS2_STREAM_COLOR)) {
        logger_->debug("CTor");
}

RealSenseCamera::~RealSenseCamera() {
    logger_->debug("DTor");
    deinitialize();
}

bool RealSenseCamera::initialize() {
    logger_->debug("initialize()");
    if (initialized_) {
        logger_->info("Already initialized");
        return false;
    }

    cfg_.disable_all_streams();
    cfg_.enable_device(serialNumber_);

    size_t trials{0};
    for (auto& resolution : defaultColorResolution) {
        try {
            cfg_.enable_stream(RS2_STREAM_COLOR,
                resolution.first.first, resolution.first.second, RS2_FORMAT_RGB8);
            cfg_.enable_stream(RS2_STREAM_DEPTH,
                resolution.second.first, resolution.second.second, RS2_FORMAT_Z16);

            profile_ = pipe_.start(cfg_);
            break;
        } catch (const rs2::error& e) {
            logger_->debug("There was a problem to start the camera: {0}", e.what());
            trials++;
        }
    }

    if (trials == defaultColorResolution.size()) {
        logger_->error("Unable to initialize any of the cameras");
        return false;
    }

    try {
        pipe_.wait_for_frames(FRAME_TIMEOUT);
    } catch (const rs2::error& ex) {
        logger_->warn("Failed to grab frame: {}", ex.what());
        return false;
    }

    activeColorStream_.reset(new rs2::video_stream_profile(
        profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>()));
    activeDepthStream_.reset(new rs2::video_stream_profile(
        profile_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>()));

    device_ = profile_.get_device();
    for (auto&& sensor : device_.query_sensors()) {
        logger_->debug("Sensor name: {}", sensor.get_info(RS2_CAMERA_INFO_NAME));
        if (sensor.get_info(RS2_CAMERA_INFO_NAME) == std::string("RGB Camera")) {
            colorSensor_.reset(new rs2::color_sensor(sensor));
        } else if (sensor.get_info(RS2_CAMERA_INFO_NAME) == std::string("Stereo Module")) {
            depthSensor_.reset(new rs2::depth_sensor(sensor));
        } else if (sensor.get_info(RS2_CAMERA_INFO_NAME) == std::string("L500 Depth Sensor")) {
            depthSensor_.reset(new rs2::depth_sensor(sensor));
        } else if (sensor.get_info(RS2_CAMERA_INFO_NAME) == std::string("Motion Module")) {
            motionSensor_.reset(new rs2::motion_sensor(sensor));
        }
    }

    try {
        extrinsic_ = profile_.get_stream(RS2_STREAM_DEPTH).get_extrinsics_to(
            profile_.get_stream(RS2_STREAM_COLOR));
    } catch (const rs2::error& er) {
        logger_->warn("Could not get exstrinsic parameters: {}", er.what());
        return false;
    }

    logger_->info("Realsense correctly initialized");
    initialized_ = true;
    return true;
}

bool RealSenseCamera::deinitialize() {
    logger_->debug("deinitialize()");
    if (!initialized_) {
        logger_->info("Not initialized");
        return false;
    }

    try {
        pipe_.stop();
    } catch (const std::exception& ex) {
        logger_->warn("Failed to stop pipe: {}", ex.what());
        return false;
    }
    cfg_.disable_all_streams();
    pipe_ = rs2::pipeline();
    cfg_ = rs2::config();

    initialized_ = false;
    return true;
}

cv::Mat RealSenseCamera::getFrame() {
    if (!initialized_) {
        logger_->info("Not initialized");
        return cv::Mat();
    }

    int width = activeColorStream_->width();
    int height = activeColorStream_->height();
    rs2::frameset frames;
    {
        std::unique_lock<std::mutex> lock(pipeMutex_);
        try {
            frames = pipe_.wait_for_frames(FRAME_TIMEOUT);
        } catch (const rs2::error& ex) {
            logger_->warn("Failed to grab frame: {}", ex.what());
            return cv::Mat();
        }
    }
    auto *color_image = const_cast<void *>(frames.first(RS2_STREAM_COLOR).get_data());
    cv::Mat img = cv::Mat(height, width, CV_8UC3, color_image);
    cv::Mat bgr;
    cv::cvtColor(img, bgr, cv::COLOR_RGB2BGR);
    return bgr;
}

bool RealSenseCamera::setResolution(const cv::Size& res) {
    logger_->debug("setResolution()");
    if (!initialized_) {
        logger_->error("Not initialized");
        return false;
    }

    // Check if the resolution is usable
    std::vector<cv::Size> resolutions = getAvailableResolutions();
    if(!(std::find(resolutions.begin(), resolutions.end(), res) != resolutions.end())) {
        logger_->error("This resolution is not available for the color stream: ({0}, {1})",
            res.width, res.height);
        return false;
    }

    std::unique_lock<std::mutex> lock(pipeMutex_);
    pipe_.stop();
    
    cfg_.enable_stream(RS2_STREAM_COLOR,
        res.width, res.height,
        RS2_FORMAT_RGB8);
    try {
        profile_ = pipe_.start(cfg_);
    } catch (const rs2::error& e) {
        logger_->error("There was a problem to start the camera: {0}", e.what());
        return false;
    }

    try {
        pipe_.wait_for_frames(FRAME_TIMEOUT);
    } catch (const rs2::error& ex) {
        logger_->warn("Failed to grab frame: {}", ex.what());
        return false;
    }

    activeColorStream_.reset(new rs2::video_stream_profile(
        profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>()));
    activeDepthStream_.reset(new rs2::video_stream_profile(
        profile_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>()));
    return true;
}
bool RealSenseCamera::setFramerate(int fps) {
    logger_->debug("setFramerate()");
    if (!initialized_) {
        logger_->error("Not initialized");
        return false;
    }

    bool allowed_fps = false;
    for (auto&& profile : colorSensor_->get_stream_profiles()) {
        if (auto video = profile.as<rs2::video_stream_profile>()) {
            if (video.width()==activeColorStream_->width() &&
                video.height()==activeColorStream_->height() && video.fps() == fps){
                allowed_fps = true;
                break;
            }
        }
    }

    if(!(allowed_fps)){
        logger_->error("Framerate {0}, is not supported for resolution: ({1}, {2})", fps, activeColorStream_->width(), activeColorStream_->height());
        return false;
    }

    std::unique_lock<std::mutex> lock(pipeMutex_);
    pipe_.stop();
    cfg_.enable_stream(RS2_STREAM_COLOR,
        activeColorStream_->width(), activeColorStream_->height(),
        RS2_FORMAT_RGB8,
        fps);
    try {
        profile_ = pipe_.start(cfg_);
    } catch (const rs2::error &e) {
        logger_->error("There was a problem to start the camera: {0}", e.what());
        return false;
    }

    try {
        pipe_.wait_for_frames(FRAME_TIMEOUT);
    } catch (const rs2::error& ex) {
        logger_->warn("Failed to grab frame: {}", ex.what());
        return false;
    }

    activeColorStream_.reset(new rs2::video_stream_profile(
        profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>()));
    activeDepthStream_.reset(new rs2::video_stream_profile(
        profile_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>()));

    return true;
}

bool RealSenseCamera::setZoom(float zoom) {
    logger_->debug("setZoom()");
    logger_->warn("setZoom not supported");
    return false;
}
bool RealSenseCamera::setPosition(float pan, float tilt) {
    logger_->debug("setPosition()");
    logger_->warn("setPosition not supported");
    return false;
}
bool RealSenseCamera::setExposure(float exposure) {
    logger_->debug("setExposure()");
    logger_->warn("setExposure not supported");
    return false;
}
bool RealSenseCamera::setShutterSpeed(float speed) {
    logger_->debug("setShutterSpeed()");
    logger_->warn("setShutterSpeed not supported");
    return false;
}

bool RealSenseCamera::setFocusMode(ICamera::FocusModes mode) {
    logger_->debug("setFocusMode()");
    logger_->warn("setFocusMode not supported");
    return false;
}
bool RealSenseCamera::setFocus(float focus) {
    logger_->debug("setFocus()");
    logger_->warn("setFocus not supported");
    return false;
}
bool RealSenseCamera::setISO(int iso) {
    logger_->debug("setISO()");
    logger_->warn("setISO not supported");
    return false;
}

boost::optional<cv::Size> RealSenseCamera::getResolution() {
    logger_->debug("getResolution()");
    if (!initialized_) {
        logger_->info("Not initialized");
        return boost::none;
    }
    return cv::Size(activeColorStream_->width(), activeColorStream_->height());
}

boost::optional<int> RealSenseCamera::getFramerate() {
    logger_->debug("getFramerate()");
    if (!initialized_) {
        logger_->info("Not initialized");
        return boost::none;
    }
    return activeColorStream_->fps();
}

boost::optional<float> RealSenseCamera::getZoom() {
    logger_->debug("getZoom()");
    logger_->warn("getZoom not supported");
    return boost::none;
}
boost::optional<float> RealSenseCamera::getPan() {
    logger_->debug("getPan()");
    logger_->warn("getPan not supported");
    return boost::none;
}
boost::optional<float> RealSenseCamera::getTilt() {
    logger_->debug("getTilt()");
    logger_->warn("getTilt not supported");
    return boost::none;
}

boost::optional<float> RealSenseCamera::getExposure() {
    logger_->debug("getExposure()");
    logger_->warn("getExposure not supported");
    return boost::none;
}

boost::optional<float> RealSenseCamera::getShutterSpeed() {
    logger_->debug("getShutterSpeed()");
    logger_->warn("getShutterSpeed not supported");
    return boost::none;
}

boost::optional<cameras::ICamera::FocusModes> RealSenseCamera::getFocusMode() {
    logger_->debug("getFocusMode()");
    logger_->warn("getFocusMode not supported");
    return boost::none;
}

boost::optional<float> RealSenseCamera::getFocus() {
    logger_->debug("getFocus()");
    logger_->warn("getFocus not supported");
    return boost::none;
}

boost::optional<int> RealSenseCamera::getISO() {
    logger_->debug("getISO()");
    logger_->warn("getISO not supported");
    return boost::none;
}

std::vector<cv::Size> RealSenseCamera::getAvailableResolutions() {
    logger_->debug("getAvailableResolutions()");
    if (!initialized_) {
        logger_->info("Not initialized");
        return std::vector<cv::Size>();
    }
    std::vector<cv::Size> resolutions;
    for (auto&& profile : colorSensor_->get_stream_profiles()) {
        if (auto video = profile.as<rs2::video_stream_profile>()) {
            cv::Size resolution(video.width(), video.height());
            if (std::find(resolutions.begin(), resolutions.end(), resolution) == resolutions.end()) {  // NOLINT
                resolutions.push_back(resolution);
            }
        }
    }
    return resolutions;
}

std::vector<int>  RealSenseCamera::getAvailableFramerates(const cv::Size& resolution){
    logger_->debug("getAvailableFramerates()");
    if (!initialized_) {
        logger_->info("Not initialized");
        return std::vector<int>();
    }
    
    std::vector<int> framerates;
    for (auto&& profile : colorSensor_->get_stream_profiles()) {
        if (auto video = profile.as<rs2::video_stream_profile>()) {
            if (video.width()==resolution.width && video.height()==resolution.height){
                framerates.push_back(video.fps());
            }
        }
    }
    return framerates;
}

cv::rgbd::RgbdFrame RealSenseCamera::getRgbdFrame() {
    logger_->debug("getRgbdFrame()");
    if (!initialized_) {
        logger_->info("Not initialized");
        return cv::rgbd::RgbdFrame();
    }

    int widthColor = activeColorStream_->width();
    int heightColor = activeColorStream_->height();

    rs2::frameset frames;
    {
        std::unique_lock<std::mutex> lock(pipeMutex_);
        try {
            frames = pipe_.wait_for_frames(FRAME_TIMEOUT);
        } catch (const rs2::error& ex) {
            logger_->warn("Failed to grab frame: {}", ex.what());
            return cv::rgbd::RgbdFrame();
        }
    }

    auto *color_image = const_cast<void *>(frames.first(RS2_STREAM_COLOR).get_data());
    cv::Mat rgb = cv::Mat(heightColor, widthColor, CV_8UC3, color_image);
    cv::Mat img;
    cv::cvtColor(rgb, img, cv::COLOR_RGB2BGR);

    int widthDepth = activeDepthStream_->width();
    int heightDepth = activeDepthStream_->height();
    auto *depth_image = const_cast<void *>(frames.first(RS2_STREAM_DEPTH).get_data());
    cv::Mat imgDepth = cv::Mat(heightDepth, widthDepth, CV_16UC1, depth_image);

    float scale = static_cast<uint16_t>(depthSensor_->get_depth_scale() * 1000);
    if (scale!=1.0){
        logger_->debug("Applying a scale of {}", static_cast<float>(scale));
        imgDepth = scale * imgDepth;
    }

    cv::rgbd::RgbdFrame frame;
    frame.image = img;
    frame.depth = imgDepth;
    return frame;
}

bool RealSenseCamera::setDepthResolution(const cv::Size& res) {
    logger_->debug("setDepthResolution()");
    if (!initialized_) {
        logger_->info("Not initialized");
        return false;
    }

    // Check if the resolution is usable
    std::vector<cv::Size> resolutions = getAvailableDepthResolutions();
    if(!(std::find(resolutions.begin(), resolutions.end(), res) != resolutions.end())) {
        logger_->error("This resolution is not available for the depth stream: ({0}, {1})",
            res.width, res.height);
        return false;
    }

    std::unique_lock<std::mutex> lock(pipeMutex_);
    pipe_.stop();
    cfg_.enable_stream(RS2_STREAM_DEPTH,
        res.width, res.height,
        RS2_FORMAT_Z16);
    try {
        profile_ = pipe_.start(cfg_);
    } catch (const rs2::error& e) {
        logger_->error("There was a problem to start the camera: {0}", e.what());
        return false;
    }

    try {
        pipe_.wait_for_frames(FRAME_TIMEOUT);
    } catch (const rs2::error& ex) {
        logger_->warn("Failed to grab frame: {}", ex.what());
        return false;
    }

    activeColorStream_.reset(new rs2::video_stream_profile(
        profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>()));
    activeDepthStream_.reset(new rs2::video_stream_profile(
        profile_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>()));
    return true;
}

bool RealSenseCamera::setDepthFramerate(int fps) {
    logger_->debug("setDepthFramerate()");
    if (!initialized_) {
        logger_->info("Not initialized");
        return false;
    }

    bool allowed_fps = false;
    for (auto&& profile : depthSensor_->get_stream_profiles()) {
        if (auto video = profile.as<rs2::video_stream_profile>()) {
            if (video.width()==activeDepthStream_->width() &&
                video.height()==activeDepthStream_->height() && video.fps() == fps){
                allowed_fps = true;
                break;
            }
        }
    }

    if(!(allowed_fps)){
        logger_->error("Framerate {0}, is not supported for resolution: ({1}, {2})", fps, activeDepthStream_->width(), activeDepthStream_->height());
        return false;
    }

    pipe_.stop();
    cfg_.enable_stream(RS2_STREAM_DEPTH,
        activeDepthStream_->width(), activeDepthStream_->height(),
        RS2_FORMAT_Z16,
        fps);
    try {
        profile_ = pipe_.start(cfg_);
    } catch (const rs2::error &e) {
        logger_->error("There was a problem to start the camera: {0}", e.what());
        return false;
    }

    try {
        pipe_.wait_for_frames(FRAME_TIMEOUT);
    } catch (const rs2::error& ex) {
        logger_->warn("Failed to grab frame: {}", ex.what());
        return false;
    }

    activeColorStream_.reset(new rs2::video_stream_profile(
        profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>()));
    activeDepthStream_.reset(new rs2::video_stream_profile(
        profile_.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>()));

    return true;
}

boost::optional<cv::Size> RealSenseCamera::getDepthResolution() {
    logger_->debug("getDepthResolution()");
    if (!initialized_) {
        logger_->info("Not initialized");
        return boost::none;
    }
    return cv::Size(activeDepthStream_->width(), activeDepthStream_->height());
}

boost::optional<int> RealSenseCamera::getDepthFramerate() {
    logger_->debug("getDepthFramerate()");
    if (!initialized_) {
        logger_->info("Not initialized");
        return boost::none;
    }
    return activeDepthStream_->fps();
}

std::vector<cv::Size> RealSenseCamera::getAvailableDepthResolutions() {
    logger_->debug("getAvailableDepthResolutions()");
    if (!initialized_) {
        logger_->info("Not initialized");
        return std::vector<cv::Size>();
    }

    std::vector<cv::Size> resolutions;
    for (auto&& profile : depthSensor_->get_stream_profiles()) {
        auto video = profile.as<rs2::video_stream_profile>();
        if (video && (profile.stream_name() == std::string("Depth"))) {
            cv::Size resolution(video.width(), video.height());
            if (std::find(resolutions.begin(), resolutions.end(), resolution) == resolutions.end()) {  // NOLINT
                resolutions.push_back(resolution);
            }
        }
    }
    return resolutions;
}

    std::vector<int> RealSenseCamera::getAvailableDepthFramerates(const cv::Size& resolution){
    logger_->debug("getAvailableFramerates()");
    if (!initialized_) {
        logger_->info("Not initialized");
        return std::vector<int>();
    }
    
    std::vector<int> framerates;
    for (auto&& profile : depthSensor_->get_stream_profiles()) {
        auto video = profile.as<rs2::video_stream_profile>();
        if (video && (profile.stream_name() == std::string("Depth"))) {
            if (video.width()==resolution.width && video.height()==resolution.height){
                framerates.push_back(video.fps());
            }
        }
    }
    return framerates;
}


boost::optional<cv::Mat> RealSenseCamera::getColorCameraMatrix()  {
    logger_->debug("getColorCameraMatrix()");
    if (!initialized_) {
        logger_->info("Not initialized");
        return boost::none;
    }

    std::unique_lock<std::mutex> lock(pipeMutex_);
    cv::Mat matrix(3, 3, CV_32F);
    matrix.at<float>(0, 0) = activeColorStream_->get_intrinsics().fx;
    matrix.at<float>(0, 1) = 0;
    matrix.at<float>(0, 2) = activeColorStream_->get_intrinsics().ppx;
    matrix.at<float>(1, 0) = 0;
    matrix.at<float>(1, 1) = activeColorStream_->get_intrinsics().fy;
    matrix.at<float>(1, 2) = activeColorStream_->get_intrinsics().ppy;
    matrix.at<float>(2, 0) = 0;
    matrix.at<float>(2, 1) = 0;
    matrix.at<float>(2, 2) = 1;
    return matrix;
}

boost::optional<cv::Mat> RealSenseCamera::getColorDistortionMatrix()  {
    logger_->debug("getColorDistortionMatrix()");
    if (!initialized_) {
        logger_->info("Not initialized");
        return boost::none;
    }

    std::unique_lock<std::mutex> lock(pipeMutex_);
    cv::Mat coeffs(1, 5, CV_32F);
    for (int i=0; i < 5; i++) {
        coeffs.at<float>(0, i) = activeColorStream_->get_intrinsics().coeffs[i];
    }
    return coeffs;
}

boost::optional<cv::Mat> RealSenseCamera::getDepthCameraMatrix()  {
    logger_->debug("getDepthCameraMatrix()");
    if (!initialized_) {
        logger_->info("Not initialized");
        return boost::none;
    }

    std::unique_lock<std::mutex> lock(pipeMutex_);
    cv::Mat matrix(3, 3, CV_32F);
    matrix.at<float>(0, 0) = activeDepthStream_->get_intrinsics().fx;
    matrix.at<float>(0, 1) = 0;
    matrix.at<float>(0, 2) = activeDepthStream_->get_intrinsics().ppx;
    matrix.at<float>(1, 0) = 0;
    matrix.at<float>(1, 1) = activeDepthStream_->get_intrinsics().fy;
    matrix.at<float>(1, 2) = activeDepthStream_->get_intrinsics().ppy;
    matrix.at<float>(2, 0) = 0;
    matrix.at<float>(2, 1) = 0;
    matrix.at<float>(2, 2) = 1;

    return matrix;
}

boost::optional<cv::Mat> RealSenseCamera::getDepthDistortionMatrix()  {
    logger_->debug("getDepthDistortionMatrix()");
    if (!initialized_) {
        logger_->info("Not initialized");
        return boost::none;
    }

    std::unique_lock<std::mutex> lock(pipeMutex_);
    cv::Mat coeffs(1, 5, CV_32F);
    for (int i=0; i < 5; i++) {
        coeffs.at<float>(0, i) =  activeDepthStream_->get_intrinsics().coeffs[i];
    }
    return coeffs;
}

boost::optional<cv::Mat> RealSenseCamera::getDepth2ColorExtrinsics()  {
    logger_->debug("getDepth2ColorExtrinsics()");
    if (!initialized_) {
        logger_->info("Not initialized");
        return boost::none;
    }

    cv::Mat matrix(4, 4, CV_32F);

    matrix.at<float>(0, 0) = extrinsic_.rotation[0];
    matrix.at<float>(0, 1) = extrinsic_.rotation[1];
    matrix.at<float>(0, 2) = extrinsic_.rotation[2];
    matrix.at<float>(0, 3) = extrinsic_.translation[0];

    matrix.at<float>(1, 0) = extrinsic_.rotation[3];
    matrix.at<float>(1, 1) = extrinsic_.rotation[4];
    matrix.at<float>(1, 2) = extrinsic_.rotation[5];
    matrix.at<float>(1, 3) = extrinsic_.translation[1];

    matrix.at<float>(2, 0) = extrinsic_.rotation[6];
    matrix.at<float>(2, 1) = extrinsic_.rotation[7];
    matrix.at<float>(2, 2) = extrinsic_.rotation[8];
    matrix.at<float>(2, 3) = extrinsic_.translation[2];

    matrix.at<float>(3, 0) = 0;
    matrix.at<float>(3, 1) = 0;
    matrix.at<float>(3, 2) = 0;
    matrix.at<float>(3, 3) = 1;
    return matrix;
}

boost::optional<std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
        cv::rgbd::RgbdFrame> > RealSenseCamera::getPointCloud(bool alignFrames) {
    logger_->debug("getPointCloud()");
    if (!initialized_) {
        logger_->info("Not initialized");
        return boost::none;
    }

    auto frame = getRgbdFrame();
    if (frame.image.empty()) {
        return boost::none;
    }

    pcl::PointXYZRGBA invalidPoint;
    invalidPoint.x = std::numeric_limits<float>::quiet_NaN();
    invalidPoint.y = std::numeric_limits<float>::quiet_NaN();
    invalidPoint.z = std::numeric_limits<float>::quiet_NaN();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>(
        frame.depth.cols, frame.depth.rows));
    cloud->is_dense = false;

    cv::Mat alignedDepth(frame.image.rows, frame.image.cols, frame.depth.type(), cv::Scalar(0));

    auto colorIntrinsics = activeColorStream_->get_intrinsics();
    cv::Mat points;
    cv::rgbd::depthTo3d(frame.depth, getDepthCameraMatrix().get(), points);
    float color_point[3];
    float color_pixel[2];
    for (int dy = 0; dy < frame.depth.rows; ++dy) {
        for (int dx = 0; dx < frame.depth.cols; ++dx) {
            auto p = reinterpret_cast<const cv::Point3f*>(points.ptr(dy, dx));
            // If the point is invalid, continue to the next
            if ((p->z == 0) || (isnanf(p->z)) || (p->z > REALSENSE_FILTER_MAX_RANGE)) {
                cloud->at(dx, dy) = invalidPoint;
                continue;
            }

            float depth_point[3] = { p->x, p->y, p->z };
            rs2_transform_point_to_point(color_point, &extrinsic_, depth_point);
            rs2_project_point_to_pixel(color_pixel,
                &colorIntrinsics, color_point);

            auto cx = static_cast<int>(color_pixel[0]);
            auto cy = static_cast<int>(color_pixel[1]);
            if (cx < 0 || cy < 0 || cx >= frame.image.cols || cy >= frame.image.rows) {
                cloud->at(dx, dy).rgb = 1.0;
            } else {
                if (alignFrames)
                    alignedDepth.at<uint16_t>(cy, cx) = static_cast<uint16_t>(p->z*1000);
                cloud->at(dx, dy).r = frame.image.at<cv::Vec3b>(cy, cx)[2];
                cloud->at(dx, dy).g = frame.image.at<cv::Vec3b>(cy, cx)[1];
                cloud->at(dx, dy).b = frame.image.at<cv::Vec3b>(cy, cx)[0];

                cloud->at(dx, dy).x = depth_point[0];
                cloud->at(dx, dy).y = depth_point[1];
                cloud->at(dx, dy).z = depth_point[2];
                cloud->at(dx, dy).a = 255;
            }
        }
    }

    if (alignFrames)
        frame.depth = alignedDepth;

    return std::pair<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
        cv::rgbd::RgbdFrame>({cloud, frame});
}

bool RealSenseCamera::setDeviceSpecificParameters(const nlohmann::json& configData) {
    logger_->debug("setDeviceSpecificParameters()");
    if (!initialized_ || configData.empty()) {
        logger_->info("Not initialized or configuration is empty");
        return false;
    }

    try {
        rs2::pipeline_profile profile = pipe_.get_active_profile();

        auto serializable  = profile.get_device().as<rs2::serializable_device>();
        serializable.load_json(configData.dump());
    } catch (const std::exception &ex) {
        logger_->error("Unable to change the configuration because \"{}\"", ex.what());
        return false;
    }

    return true;
}

}  // namespace rgbdcameras
}  // namespace sensors
}  // namespace crf
