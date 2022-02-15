/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MNRO
 *         Krzysztof Szczurek CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <sstream>
#include <string>
#include <vector>

#include <eigen3/Eigen/Core>
#include <kdl/frames.hpp>
#include <opencv2/core.hpp>

#include "CommunicationUtility/StreamWriter.hpp"

namespace crf {
namespace utility {
namespace communicationutility {

template<class T>
bool StreamWriter::write(const T & value) {
    if (!std::is_arithmetic<T>::value) {
        return false;
    }
    os_.write(reinterpret_cast<const char*>(&value), sizeof(T));
    return true;
}

template<>
bool StreamWriter::write(const std::string & value) {
    uint32_t size = value.length();
    os_.write(reinterpret_cast<const char*>(&size), sizeof(size));
    os_ << value;
    return true;
}

template<>
bool StreamWriter::write(const KDL::Frame & value) {
    for (auto i = 0; i < 3; ++i) {
        for (auto j = 0; j < 4; ++j) {
            float element = value(i, j);
            os_.write(reinterpret_cast<const char*>(&element), sizeof(element));
        }
    }
    return true;
}

template<>
bool StreamWriter::write(const Eigen::VectorXf & value) {
    std::vector<float> vec;
    for (unsigned int i = 0; i < value.size(); ++i) {
        vec.push_back(value[i]);
    }
    write(vec);
    return true;
}

template<>
bool StreamWriter::write(const Eigen::MatrixXf & value) {
    uint32_t rows = value.rows();
    uint32_t cols = value.cols();
    os_.write(reinterpret_cast<const char*>(&rows), sizeof(rows));
    os_.write(reinterpret_cast<const char*>(&cols), sizeof(cols));
    for (uint32_t i = 0; i < rows; ++i) {
        for (uint32_t j = 0; j < cols; ++j) {
        os_.write(reinterpret_cast<const char*>(&value(i, j)), sizeof(value(i, j)));
        }
    }
    return true;
}

template<>
bool StreamWriter::write(const cv::Mat & value) {
    if (value.empty()) {
        return false;
    }
    int32_t type = value.type();
    int32_t rows = value.rows;
    int32_t cols = value.cols;
    int32_t elemSize = value.elemSize();
    os_.write(reinterpret_cast<const char*>(&type), sizeof(type));
    os_.write(reinterpret_cast<const char*>(&rows), sizeof(rows));
    os_.write(reinterpret_cast<const char*>(&cols), sizeof(cols));
    os_.write(reinterpret_cast<const char*>(&elemSize), sizeof(elemSize));
    os_.write(reinterpret_cast<const char*>(value.data), elemSize * rows * cols);
    return true;
}

template<>
bool StreamWriter::write(const cv::Rect & value) {
    int width = value.width;
    int height = value.height;
    int x = value.x;
    int y = value.y;
    os_.write(reinterpret_cast<const char*>(&width), sizeof(width));
    os_.write(reinterpret_cast<const char*>(&height), sizeof(height));
    os_.write(reinterpret_cast<const char*>(&x), sizeof(x));
    os_.write(reinterpret_cast<const char*>(&y), sizeof(y));
    return true;
}

template<class T>
bool StreamWriter::write(const T * value, uint32_t size) {
    if (!std::is_arithmetic<T>::value) {
        return false;
    }
    os_.write(reinterpret_cast<const char*>(value), size * sizeof(T));
    return true;
}

template<class T>
inline bool StreamWriter::write(const std::vector<T> & value) {
    uint32_t size = value.size();
    if (size == 0) {
        return false;
    }
    os_.write(reinterpret_cast<const char*>(&size), sizeof(size));
    if (std::is_arithmetic<T>::value) {
        for (unsigned int i=0; i < value.size(); i++) {
        os_.write(reinterpret_cast<const char*>(&value[i]), sizeof(T));
        }
    } else {
        for (unsigned int i=0; i < value.size(); i++) {
        write<T>(value[i]);
        }
    }
    return true;
}

template bool StreamWriter::write<int8_t>(const int8_t & value);
template bool StreamWriter::write<int16_t>(const int16_t & value);
template bool StreamWriter::write<int32_t>(const int32_t & value);
template bool StreamWriter::write<int64_t>(const int64_t & value);
template bool StreamWriter::write<uint8_t>(const uint8_t & value);
template bool StreamWriter::write<uint16_t>(const uint16_t & value);
template bool StreamWriter::write<uint32_t>(const uint32_t & value);
template bool StreamWriter::write<uint64_t>(const uint64_t & value);
template bool StreamWriter::write<float>(const float & value);
template bool StreamWriter::write<double>(const double & value);

template bool StreamWriter::write<int32_t>(const int32_t * value, uint32_t size);
template bool StreamWriter::write<double>(const double * value, uint32_t size);

template bool StreamWriter::write<int8_t>(const std::vector<int8_t> & value);
template bool StreamWriter::write<int16_t>(const std::vector<int16_t> & value);
template bool StreamWriter::write<int32_t>(const std::vector<int32_t> & value);
template bool StreamWriter::write<int64_t>(const std::vector<int64_t> & value);
template bool StreamWriter::write<uint8_t>(const std::vector<uint8_t> & value);
template bool StreamWriter::write<uint16_t>(const std::vector<uint16_t> & value);
template bool StreamWriter::write<uint32_t>(const std::vector<uint32_t> & value);
template bool StreamWriter::write<uint64_t>(const std::vector<uint64_t> & value);
template bool StreamWriter::write<float>(const std::vector<float> & value);
template bool StreamWriter::write<double>(const std::vector<double> & value);
template bool StreamWriter::write<Eigen::VectorXf>(const std::vector<Eigen::VectorXf> & value);
template bool StreamWriter::write<Eigen::MatrixXf>(const std::vector<Eigen::MatrixXf> & value);
template bool StreamWriter::write<KDL::Frame>(const std::vector<KDL::Frame> & value);
template bool StreamWriter::write<cv::Mat>(const std::vector<cv::Mat> & value);
template bool StreamWriter::write<cv::Rect>(const std::vector<cv::Rect> & value);

}  // namespace communicationutility
}  // namespace utility
}  // namespace crf
