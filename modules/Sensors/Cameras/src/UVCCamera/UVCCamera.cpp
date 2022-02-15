/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author:  Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#define DEFAULT_V4L_BUFFERS 4

#include <fcntl.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>
#include <mutex>
#include <boost/format.hpp>

#include "Cameras/UVCCamera/UVCCamera.hpp"

namespace crf {
namespace sensors {
namespace cameras {

UVCCamera::UVCCamera(const std::string& device_name) :
    logger_("UVCCamera"),
    devName_(device_name),
    streamOn_(false),
    initialized_(false),
    isZoomSupported_(),
    cameraFd_(-1),
    requestedFps_(0),
    pixelFormat_(0),
    digitalZoom_(0),
    resolution_(),
    requestedResolution_(),
    buffers_(),
    buffersinfo_(),
    currentBuffer_(0),
    tryIoctlMutex_() {
        logger_->debug("CTor");
}

UVCCamera::UVCCamera(const std::string& device_name, unsigned int encoding) :
    UVCCamera(device_name) {
        pixelFormat_ = encoding;
    }

UVCCamera::~UVCCamera() {
    logger_->debug("DTor");
    deinitialize();
}

bool UVCCamera::initialize() {
    logger_->debug("initialize()");

    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }

    // Open the camera
    cameraFd_ = ::open(devName_.c_str(), O_RDWR);
    if (cameraFd_ < 0) {
        logger_->warn("Error opening the device: {} ", std::strerror(errno));
        return false;
    }

    struct v4l2_capability cap;
    memset(&cap, 0, sizeof(cap));
    if (!tryIoctl(VIDIOC_QUERYCAP, &cap)) {
        logger_->warn("VIDIOC_QUERYCAP: {}", std::strerror(errno));
        return false;
    }

    // Print device flags
    logger_->debug("Device V4L2 flags: {}", boost::str(boost::format("%08X") % cap.device_caps));
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        logger_->warn("The device does not handle single-planar video capture.");
        return false;
    }

        // If no format selected, choose MJPEG, if not, first format
    if (pixelFormat_ == 0) {
        std::vector<unsigned int> formats = getAvailableFormats();
        if (std::find(formats.begin(), formats.end(), V4L2_PIX_FMT_MJPEG) != formats.end()) {
            pixelFormat_ = V4L2_PIX_FMT_MJPEG;  // If possible, choose MJPEG
        } else {
            pixelFormat_ = formats.front();    // Else, pick first
        }
    }

    initialized_ = true;

    // This line is necessary otherwise the first frames are empty
    auto resolution = getResolution();
    if (!resolution) {
        logger_->warn("Failed to get resolution");
        return false;
    }
    if (!setResolution(resolution.get())) {
        logger_->warn("Could not set startup resolution");
        initialized_ = false;
        return false;
    }

    isZoomSupported_ = isZoomSupported();    // Lense or digital zoom

    ::close(cameraFd_);

    if (!startFrameStream()) {
        logger_->warn("Failed to start stream");
        initialized_ = false;
        return false;
    }

    return true;
}

bool UVCCamera::deinitialize() {
    logger_->debug("deinitialize()");

    if ((streamOn_) && (!stopFrameStream())) {
        logger_->warn("Failed to stop stream");
        return false;
    }

    ::close(cameraFd_);

    initialized_ = false;
    return true;
}

bool UVCCamera::setResolution(const cv::Size& resolution) {
    logger_->debug("setResolution()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    requestedResolution_ = resolution;
    if (streamOn_) {
        if (!stopFrameStream()) {
            logger_->warn("Could not stop stream to change resolution");
            return false;
        }

        if (!startFrameStream()) {
            logger_->warn("Could not restart the stream after changing resolution");
            return false;
        }
    }
    resolution_ = resolution;
    logger_->info("Resolution set to {}", resolution);
    return true;
}

bool UVCCamera::setFramerate(int fps) {
    logger_->debug("setFramerate()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    requestedFps_ = fps;
    if (streamOn_) {
        if (!stopFrameStream()) {
            logger_->warn("Could not stop stream to change framerate");
            return false;
        }

        if (!startFrameStream()) {
            logger_->warn("Could not restart the stream after changing framerate");
            return false;
        }
    }
    logger_->info("Framerate set to {}", fps);

    return true;
}

bool UVCCamera::setZoom(float zoom) {
    logger_->debug("setZoom()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    if (isZoomSupported_) {
        struct v4l2_queryctrl queryctrl;
        memset(&queryctrl, 0, sizeof(queryctrl));
        queryctrl.id = V4L2_CID_ZOOM_ABSOLUTE;
        struct v4l2_control ctrl;
        memset(&ctrl, 0, sizeof(ctrl));
        ctrl.id = V4L2_CID_ZOOM_ABSOLUTE;

        if (!tryIoctl(VIDIOC_QUERYCTRL, &queryctrl)) {
            logger_->warn("VIDIOC_QUERYCTRL: {}", std::strerror(errno));
            return false;
        }

        if (queryctrl.minimum == queryctrl.maximum) {
            logger_->warn("Zoom is not supported");
            return false;
        }

        ctrl.value = static_cast<int>(zoom*(queryctrl.maximum-queryctrl.minimum));
        if (!tryIoctl(VIDIOC_S_CTRL, &ctrl)) {
            logger_->warn("VIDIOC_S_CTRL: {}", std::strerror(errno));
            return false;
        }
        return true;
    } else {
        digitalZoom_ = zoom;
    }
    logger_->info("Zoom set to {}", zoom);
    return true;
}

bool UVCCamera::setPosition(float pan, float tilt) {
    logger_->debug("setPosition()");
    logger_->warn("Not available for current uvc cameras");
    return false;
}

bool UVCCamera::setExposure(float exposure) {
    logger_->debug("setExposure()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    struct v4l2_queryctrl queryctrl;
    memset(&queryctrl, 0, sizeof(queryctrl));
    queryctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;

    if (!tryIoctl(VIDIOC_QUERYCTRL, &queryctrl)) {
        logger_->warn("VIDIOC_QUERYCTRL: {}", std::strerror(errno));
        return false;
    }

    if (queryctrl.minimum == queryctrl.maximum) {
        logger_->warn("Exposure is not supported");
        return false;
    }

    struct v4l2_control ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = V4L2_CID_EXPOSURE_AUTO;

    if (!tryIoctl(VIDIOC_G_CTRL, &ctrl)) {
        logger_->warn("VIDIOC_G_CTRL: {}", std::strerror(errno));
        return false;
    }

    if ((exposure > 0) && (ctrl.value != V4L2_EXPOSURE_SHUTTER_PRIORITY)) {
        memset(&ctrl, 0, sizeof(ctrl));
        ctrl.id = V4L2_CID_EXPOSURE_AUTO;
        ctrl.value = V4L2_EXPOSURE_SHUTTER_PRIORITY;
        if (!tryIoctl(VIDIOC_S_CTRL, &ctrl)) {
            logger_->warn("VIDIOC_S_CTRL V4L2_EXPOSURE_SHUTTER_PRIORITY: {}", std::strerror(errno));
            return false;
        }
    } else if ((exposure == 0) && (ctrl.value != V4L2_EXPOSURE_AUTO)) {
        memset(&ctrl, 0, sizeof(ctrl));
        ctrl.id = V4L2_CID_EXPOSURE_AUTO;
        ctrl.value = V4L2_EXPOSURE_AUTO;
        if (!tryIoctl(VIDIOC_S_CTRL, &ctrl)) {
            logger_->warn("VIDIOC_S_CTRL V4L2_EXPOSURE_AUTO: {}", std::strerror(errno));
            return false;
        }
        return true;
    }

    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ctrl.value = static_cast<int>(exposure*(queryctrl.maximum-queryctrl.minimum));

    if (!tryIoctl(VIDIOC_S_CTRL, &ctrl)) {
        logger_->warn("VIDIOC_S_CTRL V4L2_CID_EXPOSURE_ABSOLUTE: {}", std::strerror(errno));
        return false;
    }

    return true;
}

bool UVCCamera::setShutterSpeed(float) {
    logger_->debug("setShutterSpeed()");
    logger_->warn("Not supported by UVC cameras");
    return false;
}

bool UVCCamera::setFocusMode(FocusModes mode) {
    logger_->debug("setFocusMode()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    std::unique_ptr<v4l2_ext_control[]> extCtrl(new v4l2_ext_control[1]);
    memset(&extCtrl[0], 0, sizeof(extCtrl[0]));
    extCtrl[0].id = V4L2_CID_FOCUS_AUTO;
    extCtrl[0].size = 0;
    extCtrl[0].value = mode == FocusModes::Manual ? 0 : 1;

    struct v4l2_ext_controls extCtrls;
    memset(&extCtrls, 0, sizeof(extCtrls));
    extCtrls.controls = extCtrl.get();
    extCtrls.count = 1;
    extCtrls.ctrl_class = V4L2_CTRL_ID2CLASS(extCtrl[0].id);

    if (!tryIoctl(VIDIOC_S_EXT_CTRLS, &extCtrls)) {
        logger_->warn("VIDIOC_S_EXT_CTRLS: {}", std::strerror(errno));
        return false;
    }

    return true;
}

bool UVCCamera::setFocus(float focus) {
    logger_->debug("setFocus()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    struct v4l2_queryctrl queryctrl;
    memset(&queryctrl, 0, sizeof(queryctrl));
    struct v4l2_control ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    queryctrl.id = V4L2_CID_FOCUS_ABSOLUTE;
    ctrl.id = V4L2_CID_FOCUS_ABSOLUTE;

    if (!tryIoctl(VIDIOC_QUERYCTRL, &queryctrl)) {
        logger_->warn("VIDIOC_QUERYCTRL: {}", std::strerror(errno));
        return false;
    }

    if (queryctrl.minimum == queryctrl.maximum) {
        logger_->warn("Focus is not supported");
        return false;
    }

    ctrl.value = static_cast<int>(focus*(queryctrl.maximum-queryctrl.minimum));
    if (!tryIoctl(VIDIOC_S_CTRL, &ctrl)) {
        logger_->warn("VIDIOC_S_CTRL: {}", std::strerror(errno));
        return false;
    }
    logger_->info("Focus set to {}", focus);
    return true;
}

bool UVCCamera::setISO(int iso) {
    logger_->debug("setISO()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    struct v4l2_control ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = V4L2_CID_ISO_SENSITIVITY_AUTO;
    ctrl.value = iso;
    if (!tryIoctl(VIDIOC_S_CTRL, &ctrl)) {
        logger_->warn("VIDIOC_G_CTRL: {}", std::strerror(errno));
        return false;
    }
    logger_->info("ISO set to {}", iso);
    return true;
}

boost::optional<cv::Size> UVCCamera::getResolution() {
    logger_->debug("getResolution()");

    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }
    struct v4l2_format queryctrl;
    memset(&queryctrl, 0, sizeof(queryctrl));
    queryctrl.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (!tryIoctl(VIDIOC_G_FMT, &queryctrl)) {
        logger_->warn("VIDIOC_G_FMT: {}", std::strerror(errno));
        return boost::none;
    }

    return cv::Size(queryctrl.fmt.pix.width, queryctrl.fmt.pix.height);
}

boost::optional<int> UVCCamera::getFramerate() {
    logger_->debug("getFramerate()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }

    struct v4l2_streamparm parm;
    memset(&parm, 0, sizeof(parm));
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (!tryIoctl(VIDIOC_G_PARM, &parm)) {
        logger_->warn("VIDIOC_G_PARM: {}", std::strerror(errno));
        return boost::none;
    }

    return parm.parm.capture.timeperframe.denominator;
}

boost::optional<float> UVCCamera::getZoom() {
    logger_->debug("getZoom()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }

    if (isZoomSupported_) {
        struct v4l2_queryctrl queryctrl;
        memset(&queryctrl, 0, sizeof(queryctrl));
        struct v4l2_control ctrl;
        memset(&ctrl, 0, sizeof(ctrl));
        queryctrl.id = V4L2_CID_ZOOM_ABSOLUTE;
        ctrl.id = V4L2_CID_ZOOM_ABSOLUTE;

        if (!tryIoctl(VIDIOC_QUERYCTRL, &queryctrl)) {
            logger_->warn("VIDIOC_QUERYCTRL: {}", std::strerror(errno));
            return boost::none;
        }

        if (queryctrl.minimum == queryctrl.maximum) {
            logger_->warn("Zoom not supported by this model");
            return boost::none;
        }

        if (!tryIoctl(VIDIOC_G_CTRL, &ctrl)) {
            logger_->warn("VIDIOC_G_CTRL: {}", std::strerror(errno));
            return boost::none;
        }

        return static_cast<float>(ctrl.value)/
            (queryctrl.maximum-queryctrl.minimum);
    }

    return digitalZoom_;
}

boost::optional<float> UVCCamera::getPan() {
    logger_->debug("getPan()");
    logger_->warn("Not available for current uvc cameras");
    return boost::none;
}

boost::optional<float> UVCCamera::getTilt() {
    logger_->debug("getTilt()");
    logger_->warn("Not available for current uvc cameras");
    return boost::none;
}

boost::optional<float> UVCCamera::getExposure() {
    logger_->debug("getExposure()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }

    struct v4l2_queryctrl queryctrl;
    memset(&queryctrl, 0, sizeof(queryctrl));
    struct v4l2_control ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    queryctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;

    if (!tryIoctl(VIDIOC_QUERYCTRL, &queryctrl)) {
        logger_->warn("VIDIOC_QUERYCTRL: {}", std::strerror(errno));
        return boost::none;
    }

    if (queryctrl.minimum == queryctrl.maximum) {
        logger_->warn("Zoom not supported by this model");
        return boost::none;
    }

    if (!tryIoctl(VIDIOC_G_CTRL, &ctrl)) {
        logger_->warn("VIDIOC_G_CTRL: {}", std::strerror(errno));
        return boost::none;
    }

    return static_cast<float>(ctrl.value)/
        (queryctrl.maximum-queryctrl.minimum);
}

boost::optional<float> UVCCamera::getShutterSpeed() {
    logger_->debug("getShutterSpeed()");
    logger_->warn("Not supported by UVC cameras");
    return boost::none;
}

boost::optional<ICamera::FocusModes> UVCCamera::getFocusMode() {
    logger_->debug("getFocusMode()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }

    struct v4l2_control queryctrl;
    memset(&queryctrl, 0, sizeof(queryctrl));
    queryctrl.id = V4L2_CID_FOCUS_AUTO;

    if (!tryIoctl(VIDIOC_G_CTRL, &queryctrl)) {
        logger_->warn("VIDIOC_G_CTRL: {}", std::strerror(errno));
        return boost::none;
    }

    if (queryctrl.value == 1) {
        return ICamera::FocusModes::Auto;
    }

    return ICamera::FocusModes::Manual;
}

boost::optional<float> UVCCamera::getFocus() {
    logger_->debug("getFocus()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }

    struct v4l2_queryctrl queryctrl;
    memset(&queryctrl, 0, sizeof(queryctrl));
    struct v4l2_control ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    queryctrl.id = V4L2_CID_FOCUS_ABSOLUTE;
    ctrl.id = V4L2_CID_FOCUS_ABSOLUTE;

    if (!tryIoctl(VIDIOC_QUERYCTRL, &queryctrl)) {
        logger_->warn("VIDIOC_QUERYCTRL: {}", std::strerror(errno));
        return boost::none;
    }

    if (!tryIoctl(VIDIOC_G_CTRL, &ctrl)) {
        logger_->warn("VIDIOC_G_CTRL: {}", std::strerror(errno));
        return boost::none;
    }

    if (queryctrl.minimum == queryctrl.maximum) {
        logger_->warn("Focus is not supported");
        return boost::none;
    }

    return static_cast<float>(ctrl.value)/
        (queryctrl.maximum-queryctrl.minimum);
}

boost::optional<int> UVCCamera::getISO() {
    logger_->debug("getISO()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }

    struct v4l2_queryctrl queryctrl;
    memset(&queryctrl, 0, sizeof(queryctrl));
    struct v4l2_control ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    queryctrl.id = V4L2_CID_ISO_SENSITIVITY;
    ctrl.id = V4L2_CID_ISO_SENSITIVITY;

    if (!tryIoctl(VIDIOC_QUERYCTRL, &queryctrl)) {
        logger_->warn("VIDIOC_QUERYCTRL: {}", std::strerror(errno));
        return boost::none;
    }

    if (!tryIoctl(VIDIOC_G_CTRL, &ctrl)) {
        logger_->warn("VIDIOC_G_CTRL V4L2_CID_ISO_SENSITIVITY: {}", std::strerror(errno));
        return boost::none;
    }

    if (queryctrl.minimum == queryctrl.maximum) {
        logger_->warn("ISO is not supported");
        return boost::none;
    }

    return ctrl.value;
}

cv::Mat UVCCamera::getFrame() {
    // logger_->debug("getFrame()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return cv::Mat();
    }

    if (!streamOn_) {
        logger_->warn("Stream is off");
        return cv::Mat();
    }

    int nextBuffer = currentBuffer_+1;
    if (nextBuffer == DEFAULT_V4L_BUFFERS) {
        nextBuffer = 0;
    }

    if (!tryIoctl(VIDIOC_QBUF, &buffersinfo_[nextBuffer])) {
        logger_->warn("VIDIOC_QBUF: {}", std::strerror(errno));
        return cv::Mat();
    }
    if (!tryIoctl(VIDIOC_DQBUF, &buffersinfo_[currentBuffer_])) {
        logger_->warn("VIDIOC_DQBUF: {}", std::strerror(errno));
        return cv::Mat();
    }

    cv::Mat matRGB;
    if (pixelFormat_== V4L2_PIX_FMT_YUYV) {  // YUYV conversion to RGB
        cv::Mat mat = cv::Mat(resolution_, CV_8UC2, buffers_[currentBuffer_].get());

        cvtColor(mat, matRGB, cv::COLOR_YUV2BGR_YUY2);

    } else {  // If decode needed
        std::vector<char> buffer_vec(buffers_[currentBuffer_].get(),
        buffers_[currentBuffer_].get() + buffersinfo_[currentBuffer_].bytesused);
        matRGB = cv::imdecode(buffer_vec, cv::IMREAD_COLOR);
    }

    if (matRGB.data == NULL) {
        logger_->warn("Error reading raw data");
        return cv::Mat();
    }

    // Apply digital zoom
    if (!isZoomSupported_ && digitalZoom_ > 0) {
        matRGB = applyZoom(matRGB, digitalZoom_);
    }

    currentBuffer_ = nextBuffer;
    return matRGB;
}


cv::Mat UVCCamera::applyZoom(const cv::Mat &frame, float zoom) {
    // get the image size
    int width = frame.cols;
    int height = frame.rows;

    // prepare the crop
    int centerX = static_cast<int>(width/2);
    int centerY = static_cast<int>(height/2);

    // zoom is at maximum 10% of the picture
    float zoomValue = std::max(0.05f, (1-zoom)/2);
    int radiusX = static_cast<int>(width*zoomValue);
    int radiusY = static_cast<int>(height*zoomValue);

    // crop
    int minX = centerX-radiusX;
    int minY = centerY-radiusY;

    int rect_width = centerX+radiusX-minX;
    int rect_height = centerY+radiusY-minY;

    cv::Rect roi(minX, minY, rect_width, rect_height);
    cv::Mat croppedImage = frame(roi);

    // resize image
    cv::Mat resizedCropped;

    cv::resize(croppedImage, resizedCropped, cv::Size(width, height));
    return resizedCropped;
}

bool UVCCamera::setMmapVideo() {
    logger_->debug("setMmapVideo()");
    currentBuffer_ = 0;

    struct v4l2_format format;
    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.pixelformat = pixelFormat_;
    format.fmt.pix.width = requestedResolution_.width;
    format.fmt.pix.height = requestedResolution_.height;

    if (!tryIoctl(VIDIOC_S_FMT, &format)) {
        logger_->warn("VIDIOC_S_FMT: {}", std::strerror(errno));
        return false;
    }

    struct v4l2_streamparm parm;
    memset(&parm, 0, sizeof(parm));
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = requestedFps_;

    if (!tryIoctl(VIDIOC_S_PARM, &parm)) {
        logger_->warn("VIDIOC_S_PARM: {}", std::strerror(errno));
        return false;
    }

    buffers_.clear();
    buffers_.resize(DEFAULT_V4L_BUFFERS);
    buffersinfo_.clear();
    buffersinfo_.resize(DEFAULT_V4L_BUFFERS);
    struct v4l2_requestbuffers bufrequest;
    memset(&bufrequest, 0, sizeof(bufrequest));
    bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufrequest.memory = V4L2_MEMORY_MMAP;
    bufrequest.count = DEFAULT_V4L_BUFFERS;

    if (!tryIoctl(VIDIOC_REQBUFS, &bufrequest)) {
        logger_->warn("VIDIOC_REQBUFS {}", std::strerror(errno));
        return false;
    }

    for (int i = 0; i < DEFAULT_V4L_BUFFERS; i++) {
        buffersinfo_[i].type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffersinfo_[i].memory = V4L2_MEMORY_MMAP;
        buffersinfo_[i].index = i;

        if (!tryIoctl(VIDIOC_QUERYBUF, &buffersinfo_[i])) {
            logger_->warn("VIDIOC_QUERYBUF [{}]: {}", i, std::strerror(errno));
            return false;
        }

        auto deleter = mmap_deleter(buffersinfo_[i].length);
        buffers_[i] = std::unique_ptr<char, mmap_deleter>(reinterpret_cast<char*>(mmap(
            NULL,
            buffersinfo_[i].length,
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            cameraFd_,
            buffersinfo_[i].m.offset)), deleter);

        if (buffers_[i].get() == MAP_FAILED) {
            logger_->warn("Could not set mmap");
            return false;
        }

        memset(buffers_[i].get(), 0, buffersinfo_[i].length);
    }

    return true;
}

bool UVCCamera::startFrameStream() {
    logger_->debug("startFrameStream()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    cameraFd_ = ::open(devName_.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (cameraFd_ < 0) {
        logger_->warn("Error opening the device: {} ", std::strerror(errno));
        return false;
    }

    if (!setMmapVideo()) {
        return false;
    }

    int type = buffersinfo_[0].type;
    if (!tryIoctl(VIDIOC_STREAMON, &type)) {
        logger_->warn("VIDIOC_STREAMON: {}", std::strerror(errno));
        return false;
    }

    if (!tryIoctl(VIDIOC_QBUF, &buffersinfo_[0])) {
        logger_->warn("VIDIOC_DQBUF: {}", std::strerror(errno));
        return false;
    }

    streamOn_ = true;

    return true;
}

bool UVCCamera::stopFrameStream() {
    logger_->debug("stopFrameStream()");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    if (!tryIoctl(VIDIOC_STREAMOFF, &buffersinfo_[0].type)) {
        logger_->warn("VIDIOC_STREAMOFF: {}", std::strerror(errno));
        return false;
    }

    buffers_.clear();

    ::close(cameraFd_);

    streamOn_ = false;

    return true;
}

std::vector<unsigned int> UVCCamera::getAvailableFormats() {
    logger_->debug("getAvailableFormats()");
    std::vector<unsigned int> formats;

    // Get video format
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_fmtdesc fmt;
    memset(&fmt, 0, sizeof(fmt));

    fmt.index = 0;
    fmt.type = type;
    logger_->debug("Available formats:");
    while (tryIoctl(VIDIOC_ENUM_FMT, &fmt)) {
        logger_->debug(fmt.description);
        formats.push_back(fmt.pixelformat);
        fmt.index++;
    }
    return formats;
}


std::vector<cv::Size> UVCCamera::getAvailableResolutions() {
    logger_->debug("getAvailableResolutions()");

    std::vector<cv::Size> resolutions;
    if (!initialized_) {
        logger_->warn("Not initialized");
        return resolutions;
    }

    struct v4l2_frmsizeenum frmsize;
    memset(&frmsize, 0, sizeof(frmsize));
    frmsize.pixel_format = pixelFormat_;
    frmsize.index = 0;
    while (tryIoctl(VIDIOC_ENUM_FRAMESIZES, &frmsize)) {
        if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
            resolutions.push_back(cv::Size(frmsize.discrete.width, frmsize.discrete.height));
        }
        frmsize.index++;
    }

    return resolutions;
}
std::vector<int> UVCCamera::getAvailableFramerates(const cv::Size& resolution){
    logger_->debug("getAvailableFramerates()");

    std::vector<int> framerates;
    if (!initialized_) {
        logger_->warn("Not initialized");
        return framerates;
    }

    struct v4l2_frmivalenum fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.pixel_format = pixelFormat_;
    fmt.width = resolution.width;
    fmt.height = resolution.height;
    fmt.type = V4L2_FRMIVAL_TYPE_DISCRETE;
    fmt.index = 0;
    while (tryIoctl(VIDIOC_ENUM_FRAMEINTERVALS, &fmt)) {
        framerates.push_back(fmt.discrete.denominator);
        fmt.index+=1;
    }
    return framerates;
}

bool UVCCamera::setDeviceSpecificParameters(const nlohmann::json& configData) {
    logger_->debug("setDeviceSpecificParameters() not available yet");
    return false;
}

bool UVCCamera::isZoomSupported() {
    logger_->debug("isZoomSupported()");

    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    struct v4l2_queryctrl queryctrl;
    memset(&queryctrl, 0, sizeof(queryctrl));

        if (!tryIoctl(VIDIOC_QUERYCTRL, &queryctrl)) {
        logger_->warn("VIDIOC_QUERYCTRL: {}", std::strerror(errno));
        logger_->warn("Applying digital zoom");
        return false;
    }

    if (queryctrl.minimum == queryctrl.maximum) {
        logger_->warn("Zoom not supported by this model");
        logger_->warn("Applying digital zoom");
        return false;
    }

    return true;
}

bool UVCCamera::tryIoctl(uint64_t ioctlCode, void *parameter, bool failIfBusy, int attempts) {
    std::scoped_lock lock(tryIoctlMutex_);
    while (true) {
        errno = 0;
        int result = ioctl(cameraFd_, ioctlCode, parameter);
        int err = errno;
        if (result != -1) {
            return true;
        }

        const bool isBusy = (err == EBUSY);
        if (isBusy && failIfBusy) {
            return false;
        }
        if (!(isBusy || errno == EAGAIN)) {
            return false;
        }

        if (--attempts == 0) {
            return false;
        }

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(cameraFd_, &fds);

        struct timeval timeout;
        timeout.tv_sec = tryIoctlTimeoutS_;
        timeout.tv_usec = 0;

        errno = 0;
        result = select(cameraFd_ + 1, &fds, NULL, NULL, &timeout);
        err = errno;
        if (result == 0) {
            return false;
        }
        if (err == EINTR) {
            return false;
        }
    }
    return true;
}

}  // namespace cameras
}  // namespace sensors
}  // namespace crf
