#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(COMPONENTACCESSCONTROL_TESTS_FILES main.cpp
)

if (NOT DISABLE_SimpleAccessControl)
    set(COMPONENTACCESSCONTROL_TESTS_FILES ${COMPONENTACCESSCONTROL_TESTS_FILES}
                                           SimpleAccessControlTests.cpp
    )
endif()

add_executable(ut-componentaccesscontrol ${COMPONENTACCESSCONTROL_TESTS_FILES})
target_link_libraries(ut-componentaccesscontrol gmock
                                                ComponentAccessControl
)

add_test(ComponentAccessControlUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-componentaccesscontrol)
