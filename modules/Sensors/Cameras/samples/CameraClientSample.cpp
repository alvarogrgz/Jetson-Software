/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <boost/program_options.hpp>
#include <csignal>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <memory>
#include <string>

#include "Cameras/CameraClient/CameraClient.hpp"
#include "Sockets/IPC/UnixSocket.hpp"
#include "Sockets/TCP/TCPSocket.hpp"

namespace po = boost::program_options;

using crf::communication::sockets::UnixSocket;
using crf::communication::sockets::TCPSocket;

using crf::communication::datapacketsocket::PacketSocket;
using crf::sensors::cameras::CameraClient;

namespace {
volatile std::sig_atomic_t gSignalStatus;
void signal_handler(int signal) {
    gSignalStatus = signal;
}
}  // unnamed namespace

void setupTrackbars(
    const std::string& windowName,
    std::shared_ptr<crf::sensors::cameras::ICamera> camera) {
        auto resolutions = camera->getAvailableResolutions();
        if (resolutions.size() > 1) {
            cv::createTrackbar("Resolutions",
                windowName,
                0,
                resolutions.size() - 1,
                [](int pos, void *userdata) {
                    auto camera = reinterpret_cast<crf::sensors::cameras::ICamera*>(userdata);
                    auto resolutions = camera->getAvailableResolutions();
                    camera->setResolution(resolutions[pos]);
                },
                camera.get());
        }

        cv::createTrackbar("Framerate",
                windowName,
                0,
                30,
                [](int pos, void *userdata) {
                    auto camera = reinterpret_cast<crf::sensors::cameras::ICamera*>(userdata);
                    camera->setFramerate(pos);
                },
                camera.get());

        if (camera->getZoom()) {
            cv::createTrackbar("Zoom",
                windowName,
                0,
                1000,
                [](int pos, void *userdata) {
                    auto camera = reinterpret_cast<crf::sensors::cameras::ICamera*>(userdata);
                    float zoom = static_cast<float>(pos)/1000.0f;
                    camera->setZoom(zoom);
                },
                camera.get());
        }

        if (camera->getFocusMode()) {
            cv::createTrackbar("Focus mode",
                windowName,
                0,
                1,
                [](int pos, void *userdata) {
                    auto camera = reinterpret_cast<crf::sensors::cameras::ICamera*>(userdata);
                    if (pos == 1) {
                        camera->setFocus(crf::sensors::cameras::ICamera::FocusModes::Manual);
                    } else {
                        camera->setFocus(crf::sensors::cameras::ICamera::FocusModes::Auto);
                    }
                },
                camera.get());
        }

        if (camera->getFocus()) {
            cv::createTrackbar("Focus",
                windowName,
                0,
                1000,
                [](int pos, void *userdata) {
                    auto camera = reinterpret_cast<crf::sensors::cameras::ICamera*>(userdata);
                    float focus = static_cast<float>(pos)/1000.0f;
                    camera->setFocus(focus);
                },
                camera.get());
        }
}

int main(int argc, char* argv[]) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show help message")
        ("protocol", po::value<std::string>(), "Protocol type (available: tcp) [Required if port is set]")  // NOLINT
        ("host", po::value<std::string>(), "Server device [Required if protocol is set]")
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

    if (!(vm.count("protocol") && vm.count("host") && vm.count("port")) && !vm.count("ipc_name")) {
        std::cout << "Missing at least one socket connection" << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    std::shared_ptr<CameraClient> camera;

    if (vm.count("protocol") && vm.count("host") && vm.count("port")) {
        std::string networkProtocol = vm["protocol"].as<std::string>();
        std::string networkHost = vm["host"].as<std::string>();
        int networkPort = vm["port"].as<unsigned int>();

        if ((networkProtocol != "tcp") || (networkPort < 1) || (networkPort > 65535)) {
            std::cout << "Wrong network parameters" << std::endl << std::endl;
            std::cout << desc << std::endl;
            return -1;
        }

        std::shared_ptr<TCPSocket> server(new TCPSocket(networkHost, networkPort));
        std::shared_ptr<PacketSocket> socket(new PacketSocket(server));
        camera.reset(new CameraClient(socket, std::chrono::seconds(5)));
    }

    if (vm.count("ipc_name")) {
        std::string ipcName = vm["ipc_name"].as<std::string>();

        std::shared_ptr<UnixSocket> server(new UnixSocket(ipcName));
        std::shared_ptr<PacketSocket> socket(new PacketSocket(server));
        camera.reset(new CameraClient(socket, std::chrono::seconds(5)));
    }

    if (!camera->initialize()) {
        std::cout << "Could not initialize the camera client" << std::endl;
        return -1;
    }

    std::signal(SIGTSTP, signal_handler);
    std::cout << "Camera client started correctly\n";
    cv::namedWindow("viewer");

    setupTrackbars("viewer", camera);
    while (gSignalStatus != SIGTSTP) {
        auto frame = camera->getFrame();
        if (frame.empty())
            continue;
        cv::imshow("viewer", frame);
        cv::waitKey(10);
    }

    camera->deinitialize();
    std::cout << "Received closing command" << std::endl;
    return 0;
}
