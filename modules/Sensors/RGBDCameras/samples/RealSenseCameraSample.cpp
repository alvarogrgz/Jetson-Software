/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <csignal>
#include <iostream>
#include <memory>
#include <string>

#include <boost/program_options.hpp>
#include <opencv2/highgui.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "RGBDCameras/RealSenseCamera/RealSenseCamera.hpp"

volatile std::sig_atomic_t gSignalStatus;
void signal_handler(int signal) {
    gSignalStatus = signal;
}

void trackbars(const std::string& window, std::shared_ptr<crf::sensors::cameras::ICamera> cam) {
    auto resolutions = cam->getAvailableResolutions();
    if (resolutions.size() > 1) {
        cv::createTrackbar("Resolutions",
            window,
            0,
            resolutions.size() - 1,
            [](int pos, void *userdata) {
                auto cam = reinterpret_cast<crf::sensors::cameras::ICamera*>(userdata);
                auto resolutions = cam->getAvailableResolutions();
                cam->setResolution(resolutions[pos]);
            },
            cam.get());
    }
    if (cam->getFramerate()) {
        cv::createTrackbar("Framerate",
            window,
            0,
            30,
            [](int pos, void *userdata) {
                auto cam = reinterpret_cast<crf::sensors::cameras::ICamera*>(userdata);
                cam->setFramerate(pos);
            },
            cam.get());
    }
    if (cam->getZoom()) {
        cv::createTrackbar("Zoom",
            window,
            0,
            1000,
            [](int pos, void *userdata) {
                auto cam = reinterpret_cast<crf::sensors::cameras::ICamera*>(userdata);
                float zoom = static_cast<float>(pos)/1000.0f;
                cam->setZoom(zoom);
            },
            cam.get());
    }
    if (cam->getFocusMode()) {
        cv::createTrackbar("Focus mode",
            window,
            0,
            1,
            [](int pos, void *userdata) {
                auto cam = reinterpret_cast<crf::sensors::cameras::ICamera*>(userdata);
                if (pos == 1) {
                    cam->setFocus(crf::sensors::cameras::ICamera::FocusModes::Manual);
                } else {
                    cam->setFocus(crf::sensors::cameras::ICamera::FocusModes::Auto);
                }
            },
            cam.get());
    }
    if (cam->getFocus()) {
        cv::createTrackbar("Focus",
            window,
            0,
            1000,
            [](int pos, void *userdata) {
                auto cam = reinterpret_cast<crf::sensors::cameras::ICamera*>(userdata);
                float focus = static_cast<float>(pos)/1000.0f;
                cam->setFocus(focus);
            },
            cam.get());
    }
}

int main(int argc, char* argv[]) {
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("serial_number",
            boost::program_options::value<std::string>(),
            "Camera serial number (the serial number is directly written on the camera)");

    boost::program_options::variables_map vm;
    try {
        boost::program_options::store(
            boost::program_options::parse_command_line(argc, argv, desc),
            vm);
    } catch (std::exception&) {
        std::cout << "Bad command line values" << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }
    if (!vm.count("serial_number")) {
        std::cout << "Missing serial number" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::shared_ptr<crf::sensors::rgbdcameras::IRGBDCamera> camera(
        new crf::sensors::rgbdcameras::RealSenseCamera(vm["serial_number"].as<std::string>()));
    if (!camera->initialize()) {
        std::cout << "Failed to initialize the camera" << std::endl;
        return -1;
    }
    std::cout << "Real Sense started correctly\n";

    cv::namedWindow("RGB Viewer");
    trackbars("RGB Viewer", camera);

    pcl::visualization::PCLVisualizer depthViewer("Depth Viewer");
    depthViewer.setBackgroundColor(0.0, 0.0, 0.0);
    depthViewer.addCoordinateSystem(0.25, "Origin");
    camera->setResolution(cv::Size(1280, 720));
    auto extrinsic = camera->getDepth2ColorExtrinsics().get();

    std::signal(SIGINT, signal_handler);
    while (gSignalStatus != SIGINT) {
        auto frame = camera->getPointCloud(true);
        if (!frame) continue;

        depthViewer.removeAllPointClouds();
        cv::imshow("RGB Viewer", frame->second.image);
        depthViewer.addPointCloud(frame->first, "Point Cloud");
        depthViewer.spinOnce();

        cv::imshow("Depth viewer", frame->second.depth);
        cv::waitKey(10);
    }

    camera->deinitialize();
    std::cout << "Received closing command" << std::endl;
    return 0;
}
