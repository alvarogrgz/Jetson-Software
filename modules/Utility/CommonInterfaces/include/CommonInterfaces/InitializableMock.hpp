/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include "CommonInterfaces/IInitializable.hpp"

namespace crf {
namespace utility {
namespace commoninterfaces {

class InitializableMock : public IInitializable {
 public:
    MOCK_METHOD0(initialize, bool());
    MOCK_METHOD0(deinitialize, bool());
};

}  // namespace commoninterfaces
}  // namespace utility
}  // namespace crf
