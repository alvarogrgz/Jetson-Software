#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Carlos Prados Sesmero & Alejandro Diaz Rosales CERN EN/SMM/MRO 2020                                       ##
##                                                                                                                   ##
#######################################################################################################################

set(VISION_UTILITY_TESTS_FILES main.cpp
                               ./Image/ImageJSONConverterTests.cpp
                               ./PointCloud/CommunicationTests.cpp
                               ./PointCloud/EdgeTests.cpp
                               ./PointCloud/GeneratorTests.cpp
                               ./PointCloud/FilterTests.cpp
                               ./PointCloud/KeypointTests.cpp
                               ./PointCloud/NormalTests.cpp
                               ./PointCloud/SubsampleTests.cpp
)


add_executable(ut-visionutility ${VISION_UTILITY_TESTS_FILES})
target_link_libraries(ut-visionutility gmock
                                       VisionUtility
)

add_test(VisionUtilityUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-visionutility)

execute_process(COMMAND bash ${CMAKE_SOURCE_DIR}/modules/Utility/VisionUtility/tests/download.sh [${CMAKE_SOURCE_DIR}])
