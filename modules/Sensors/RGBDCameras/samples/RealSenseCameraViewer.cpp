/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO 2020
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

namespace po = boost::program_options;

volatile std::sig_atomic_t gSignalStatus;
void signal_handler(int signal) {
    gSignalStatus = signal;
}

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("serial_number", po::value<std::string>(),
            "Camera serial number (the serial number is directly written on the camera)")
        ("configFilePath", po::value<std::string>(), "Path of the configFile");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
    } catch (std::exception&) {
        std::cout << "Bad command line values" << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 0;
    }
    if (!vm.count("serial_number")) {
        std::cout << "Missing serial number" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    if (!vm.count("configFilePath")) {
        std::cout << "Missing path of the config file" << std::endl << std::endl;
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

    // Change parameters
    std::ifstream cameraConfig(vm["configFilePath"].as<std::string>());
    nlohmann::json jCameraConfig;
    cameraConfig >> jCameraConfig;
    if (!camera->setDeviceSpecificParameters(jCameraConfig)) {
        std::cout << "There was a problem loading the camera parameteres" << std::endl;
        return -1;
    }

    pcl::visualization::PCLVisualizer depthViewer("Depth Viewer");
    depthViewer.setBackgroundColor(0.0, 0.0, 0.0);
    depthViewer.addCoordinateSystem(0.25, "Origin");

    std::signal(SIGINT, signal_handler);
    while (gSignalStatus != SIGINT) {
        auto frame = camera->getPointCloud(true);
        if (!frame) continue;

        depthViewer.addPointCloud(frame->first, "Point Cloud");
        depthViewer.spin();
        depthViewer.removeAllPointClouds();
    }

    camera->deinitialize();
    std::cout << "Received closing command" << std::endl;
    return 0;
}
