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
#include <thread>

#include <boost/program_options.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

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
        ("serialNumber", po::value<std::string>(), "Camera serial number")
        ("cameraType", po::value<std::string>(), "Camera model (L515, D455, D435, D415)")
        ("configFilePath", po::value<std::string>(), "(Optinal) Path of the configFile");

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
    if (!vm.count("serialNumber")) {
        std::cout << "Missing serial number" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }
    if (!vm.count("cameraType")) {
        std::cout << "Missing serial number" << std::endl << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::shared_ptr<crf::sensors::rgbdcameras::IRGBDCamera> camera(
        new crf::sensors::rgbdcameras::RealSenseCamera(vm["serialNumber"].as<std::string>()));
    if (!camera->initialize()) {
        std::cout << "Failed to initialize the camera" << std::endl;
        return -1;
    }
    std::cout << "Real Sense started correctly\n";

    // Change parameters
    if (vm.count("configFilePath")) {
        std::ifstream cameraConfig(vm["configFilePath"].as<std::string>());
        nlohmann::json jCameraConfig;
        cameraConfig >> jCameraConfig;
        if (!camera->setDeviceSpecificParameters(jCameraConfig)) {
            std::cout << "There was a problem loading the camera parameteres" << std::endl;
            return -1;
        }
    }

    std::string type = vm["cameraType"].as<std::string>();
    if ((type == "L515") || (type == "D435") || (type == "D415")) {
        if (!camera->setResolution({1920, 1080})) {
            std::cout << "Unable to change the resolution" << std::endl;
            return -1;
        }
    } else if (type == "D455") {
        if (!camera->setResolution({1280, 800})) {
            std::cout << "Unable to change the resolution" << std::endl;
            return -1;
        }
    } else {
        std::cout << "Bad model type provided" << std::endl;
        return -1;
    }

    std::cout << "Dataset name: ";
    std::string datasetName{""};
    std::getline(std::cin, datasetName);

    std::cout << "Creating folders... ";

    std::string command0{"mkdir ../dataset/"};
    char *strCommand0 = new char[command0.size() + 1];
    strcpy(strCommand0, command0.c_str());  // NOLINT
    if (std::system(strCommand0) == 0) {
        std::cout << std::endl << "Creating dataset folder" << std::endl;
    }
    delete strCommand0;

    std::string command1{command0};
    command1 += datasetName;
    char *strCommand1 = new char[command1.size() + 1];
    strcpy(strCommand1, command1.c_str());  // NOLINT
    if (std::system(strCommand1) != 0) {
        std::cout << std::endl << "Error creating your dataset folder" << std::endl;
        return -1;
    }
    delete strCommand1;

    std::string command2{command1};
    command2 += "/cloud/";
    char *strCommand2 = new char[command2.size() + 1];
    strcpy(strCommand2, command2.c_str());  // NOLINT
    if (std::system(strCommand2) != 0) {
        std::cout << std::endl << "Error creating cloud folder" << std::endl;
        return -1;
    }
    delete strCommand2;

    std::string command3{command1};
    command3 += "/rgb/";
    char *strCommand3 = new char[command3.size() + 1];
    strcpy(strCommand3, command3.c_str());  // NOLINT
    if (std::system(strCommand3) != 0) {
        std::cout << std::endl << "Error creating picture folder" << std::endl;
        return -1;
    }
    delete strCommand3;

    std::string command4{command1};
    command4 += "/depth/";
    char *strCommand4 = new char[command4.size() + 1];
    strcpy(strCommand4, command4.c_str());  // NOLINT
    if (std::system(strCommand4) != 0) {
        std::cout << std::endl << "Error creating depth folder" << std::endl;
        return -1;
    }
    delete strCommand4;

    std::cout << "Done" << std::endl;

    std::signal(SIGINT, signal_handler);
    int counter = 0;
    char next;
    while (gSignalStatus != SIGINT) {
        std::cout << "Write 'a' and press 'Enter' to take frame number " << counter << ": ";
        std::cin >> next;
        if (next != 'a') {
            continue;
        }
        auto frame = camera->getPointCloud(true);
        if (frame) {
            std::cout << std::endl << "  - Saving point cloud... ";
            std::string cloudName = "../dataset/" + datasetName + "/cloud/";
            cloudName += std::to_string(counter) + ".pcd";
            pcl::io::savePCDFileASCII(cloudName, *frame.get().first);
            std::cout << "Done" << std::endl;

            std::cout << "  - Saving rgb... ";
            std::string rgbName = "../dataset/" + datasetName + "/rgb/";
            rgbName += std::to_string(counter) + ".png";
            cv::imwrite(rgbName, frame.get().second.image);
            std::cout << "Done" << std::endl;

            std::cout << "  - Saving depth... ";
            std::string depthName = "../dataset/" + datasetName + "/depth/";
            depthName += std::to_string(counter) + ".png";
            cv::imwrite(depthName, frame.get().second.depth);
            std::cout << "Done" << std::endl;
        } else {
            std::cout << "Unable to save information" << std::endl;
        }
        counter++;
    }

    camera->deinitialize();
    std::cout << "Received closing command" << std::endl;
    return 0;
}
