#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

# Check for the libraries dependencies
check_library_status(REQUESTER
                         DeviceManager
                     CHECK
                         nlohmann_json
)

# The communication point can be treated as an implementation for CMake
crf_implementation(IMPLEMENTATION DeviceManagerCommunicationPoint         OF DeviceManager      IS ON)

# Add the different implementations to check their dependencies
crf_implementation(IMPLEMENTATION DeviceManagerWithAutoInitialization     OF DeviceManager      IS ON)
crf_implementation(IMPLEMENTATION DeviceManagerWithPriorityAccess         OF DeviceManager      IS ON)

# Inform that this module was already checked to not repeat this procedure
mark_module_or_library_as_checked(DeviceManager)
