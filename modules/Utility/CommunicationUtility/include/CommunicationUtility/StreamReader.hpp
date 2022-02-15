/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *          Krzysztof Szczurek CERN EN/SMM/MRO 2019
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

class StreamReader {
 public:
    StreamReader() = delete;
    explicit StreamReader(std::string buffer);

    /*
     * @brief
     * @param
     * @return
     */
    template<class T>
    bool read(T * value);
    /*
     * @brief
     * @param
     * @param
     * @return
     */
    template<class T>
    bool read(T * value, uint32_t size);
    /*
     * @brief
     * @param
     * @return
     */
    template<class T>
    bool read(std::vector<T> * value);

 protected:
    std::istringstream is_;
    size_t length_;

    inline size_t leftToRead() { return length_-is_.tellg(); }
};

}  // namespace communicationutility
}  // namespace utility
}  // namespace crf
