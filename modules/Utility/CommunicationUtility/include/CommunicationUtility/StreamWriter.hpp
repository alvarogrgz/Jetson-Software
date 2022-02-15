/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MNRO 2019
 *         Krzysztof Szczurek CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <sstream>
#include <string>
#include <vector>

namespace crf {
namespace utility {
namespace communicationutility {

class StreamWriter {
 public:
    StreamWriter() = default;

    /*
     * @brief
     * @param
     * @return
     */
    template<class T>
    bool write(const T & value);
    /*
     * @brief
     * @param
     * @param
     * @return
     */
    template<class T>
    bool write(const T * value, uint32_t size);
    /*
     * @brief
     * @param
     * @return
     */
    template<class T>
    bool write(const std::vector<T> & value);
    /*
     * @brief
     * @return
     */
    inline std::string toString() { return os_.str(); }

 protected:
    std::ostringstream os_;
};

}  // namespace communicationutility
}  // namespace utility
}  // namespace crf
