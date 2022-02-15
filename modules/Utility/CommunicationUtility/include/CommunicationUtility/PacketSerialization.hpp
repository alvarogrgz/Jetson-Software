/* Â© Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MNRO 2019
 *
 *  ==================================================================================================
 */

#pragma once

#include <istream>
#include <ostream>
#include <vector>

namespace crf {
namespace utility {
namespace communicationutility {

class Serializer {
 public:
    template<class T>
    static std::ostream& serialize(std::ostream& os, const T value) {
        os.write(reinterpret_cast<const char*>(&value), sizeof(T));
        return os;
    }

    template<class T>
    static std::ostream& serialize(std::ostream& os, const std::vector<T>& value) {
        int32_t size = value.size();
        os.write(reinterpret_cast<const char*>(&size), 4);
        for (int i=0; i < value.size(); i++) {
            int32_t val = value[i];
            os.write(reinterpret_cast<const char*>(&val), sizeof(T));
        }
        return os;
    }
};

class Deserializer {
 public:
    template<class T>
    static bool deserialize(std::istream& is, T &value) {  // NOLINT
        int original_position = is.tellg();
        is.seekg(0, is.end);
        int length = is.tellg();
        is.seekg(original_position, is.beg);
        if (length - original_position < sizeof(T)) {
            return false;
        }
        is.read(reinterpret_cast<char*>(&value), sizeof(T));
        return true;
    }

    template<class T>
    static bool deserialize(std::istream& is, std::vector<T>& value) {  // NOLINT
        int original_position = is.tellg();
        is.seekg(0, is.end);
        int length = is.tellg();
        is.seekg(original_position, is.beg);
        if (length - original_position < 4) {
            return false;
        }
        value.clear();
        int32_t size;
        is.read(reinterpret_cast<char*>(&size), 4);
        if (length - original_position < 4+sizeof(T)*size) {
            return false;
        }
        for (int i=0; i < size; i++) {
            T tmp;
            is.read(reinterpret_cast<char*>(&tmp), sizeof(T));
            value.push_back(tmp);
        }
        return true;
    }
};

}  // namespace communicationutility
}  // namespace utility
}  // namespace crf
