/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2020
 *
 *  ==================================================================================================
 */

#include <boost/program_options.hpp>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>

#include "RGBDCameras/RealSenseCamera/RealSenseCamera.hpp"
#include "RGBDCameras/RGBDCameraCommunicationPoint/RGBDCameraCommunicationPoint.hpp"
#include "RGBDCameras/RGBDCameraCommunicationPoint/RGBDCameraCommunicationPointFactory.hpp"
#include "RGBDCameras/RGBDCameraCommunicationPoint/RGBDCameraManager.hpp"
#include "CommunicationPointServer/CommunicationPointServer.hpp"
#include "Sockets/IPC/UnixSocketServer.hpp"
#include "Sockets/TCP/TCPServer.hpp"

namespace po = boost::program_options;

using crf::sensors::rgbdcameras::RGBDCameraManager;
using crf::sensors::rgbdcameras::RGBDCameraCommunicationPointFactory;
using crf::communication::communicationpointserver::CommunicationPointServer;
using crf::communication::sockets::ISocketServer;
using crf::communication::sockets::TCPServer;
using crf::communication::sockets::UnixSocketServer;
using crf::sensors::rgbdcameras::IRGBDCamera;
using crf::sensors::rgbdcameras::RealSenseCamera;

namespace {
volatile std::sig_atomic_t gSignalStatus;
void signal_handler(int signal) {
    gSignalStatus = signal;
}
}  // unnamed namespace

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("serial_number", po::value<std::string>(), "Camera serial number (the serial number is directly written on the camera)")  // NOLINT
        ("protocol", po::value<std::string>(), "Protocol type (available: tcp) [Required if port is set]")  // NOLINT
        ("port", po::value<unsigned int>(), "Network port [1-65535] [Required if protocol is set]")
        ("ipc_name", po::value<std::string>(), "IPC socket name");

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

    if (!(vm.count("protocol") && vm.count("port")) && !vm.count("ipc_name")) {
        std::cout << "Missing at least one socket connection" << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::shared_ptr<IRGBDCamera> camera(new RealSenseCamera(vm["serial_number"].as<std::string>()));
    if (!camera->initialize()) {
        std::cout << "Failed to initialize the camera" << std::endl;
        return -1;
    }
    camera->deinitialize();

    std::shared_ptr<RGBDCameraManager> manager(new RGBDCameraManager(camera,
        std::chrono::seconds(5)));
    std::shared_ptr<RGBDCameraCommunicationPointFactory> communicationPointFactory(
        new RGBDCameraCommunicationPointFactory(manager));

    std::unique_ptr<CommunicationPointServer> networkServer;
    if (vm.count("protocol") && vm.count("port")) {
        std::string net_protocol = vm["protocol"].as<std::string>();
        int net_port = vm["port"].as<unsigned int>();

        if ((net_protocol != "tcp") || (net_port < 1) || (net_port > 65535)) {
            std::cout << "Wrong network parameters" << std::endl << std::endl;
            std::cout << desc << std::endl;
            return -1;
        }

        std::shared_ptr<ISocketServer> server;
        if (net_protocol == "tcp") {
            server = std::make_shared<TCPServer>(net_port);
        }
        networkServer.reset(new CommunicationPointServer(server, communicationPointFactory));
        if (!networkServer->initialize()) {
            std::cout << "Failed to initialize network server, check logger for details" << std::endl;  // NOLINT
            return -1;
        }

        std::cout << "Started network server on port " << net_port << std::endl;
    }

    std::unique_ptr<CommunicationPointServer> ipcServer;
    if (vm.count("ipc_name")) {
        std::string ipc_name = vm["ipc_name"].as<std::string>();
        std::shared_ptr<ISocketServer> server;
        server = std::make_shared<UnixSocketServer>(ipc_name);
        ipcServer.reset(new CommunicationPointServer(server, communicationPointFactory));

        if (!ipcServer->initialize()) {
            std::cout << "Failed to initialize ips server, check logger for details" << std::endl;  // NOLINT
            return -1;
        }

        std::cout << "Started ipc server on resource " << ipc_name << std::endl;
    }

    std::signal(SIGTSTP, signal_handler);
    std::cout << "Camera communication point started correctly\n";
    while (gSignalStatus != SIGTSTP) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cout << "Received closing command" << std::endl;
    return 0;
}
