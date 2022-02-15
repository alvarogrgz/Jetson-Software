#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

# Check for the modules dependencies
check_module_status(REQUESTER
                        Cameras
                    CHECK
                        CommonInterfaces
)

# Check for the libraries dependencies
check_library_status(REQUESTER
                         Cameras
                     CHECK
                         OpenCV
                         Boost
                         nlohmann_json
)

# The communication point can be treated as an implementation for CMake
crf_implementation(IMPLEMENTATION CameraCommunicationPoint         OF Cameras         IS ON)

# Add the different implementations to check their dependencies
crf_implementation(IMPLEMENTATION AxisCamera                       OF Cameras         IS ON)
crf_implementation(IMPLEMENTATION CameraClient                     OF Cameras         IS ON)
crf_implementation(IMPLEMENTATION UVCCamera                        OF Cameras         IS ON)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(Cameras)