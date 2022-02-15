/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Giacomo Lunghi CERN EN/SMM/MRO 2019
 *          Krzysztof Szczurek CERN EN/SMM/MRO 2019
 *          Jorge Camarero Vera CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <eigen3/Eigen/Core>
#include <kdl/frames.hpp>
#include <opencv2/core.hpp>

#include "CommunicationUtility/StreamReader.hpp"

namespace crf {
namespace utility {
namespace communicationutility {

StreamReader::StreamReader(std::string buffer) :
    is_(buffer),
    length_(buffer.length()) {
}

template<class T>
bool StreamReader::read(T * value) {
    if (!std::is_arithmetic<T>::value) {
        return false;
    }
    uint32_t size = sizeof(T);
    if (leftToRead() < size) {
        return false;
    }
    is_.read(reinterpret_cast<char*>(value), size);
    return true;
}

template<>
bool StreamReader::read<std::string>(std::string * value) {
    if (leftToRead() < sizeof(uint32_t)) {
        return false;
    }
    uint32_t size;
    is_.read(reinterpret_cast<char*>(&size), sizeof(size));

    if (leftToRead() < size) {
        return false;
    }
    for (uint32_t i = 0; i < size; ++i) {
        char tmp;
        is_.read(reinterpret_cast<char*>(&tmp), sizeof(tmp));
        *value += tmp;
    }
    return true;
}

template<>
bool StreamReader::read<KDL::Frame>(KDL::Frame * value) {
    if (leftToRead() < 48) {
        return false;
    }
    float tmpMatrix[3][4];
    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = 0; j < 4; ++j) {
            float element;
            is_.read(reinterpret_cast<char*>(&element), sizeof(element));
            tmpMatrix[i][j] = element;
        }
    }
    KDL::Vector position;
    position.x(tmpMatrix[0][3]);
    position.y(tmpMatrix[1][3]);
    position.z(tmpMatrix[2][3]);
    KDL::Rotation rotation(tmpMatrix[0][0], tmpMatrix[0][1], tmpMatrix[0][2],
        tmpMatrix[1][0], tmpMatrix[1][1], tmpMatrix[1][2],
        tmpMatrix[2][0], tmpMatrix[2][1], tmpMatrix[2][2]);
    *value = KDL::Frame(rotation, position);
    return true;
}

template<>
bool StreamReader::read<Eigen::VectorXf>(Eigen::VectorXf * value) {
    std::vector<float> vec;
    if (!read(&vec)) {
        return false;
    }
    value->resize(vec.size());
    for (unsigned int i = 0; i < vec.size(); ++i) {
        (*value)[i] = vec[i];
    }
    return true;
}

template<>
bool StreamReader::read<Eigen::MatrixXf>(Eigen::MatrixXf * value) {
    if (leftToRead() < 2*sizeof(uint32_t)) {
        return false;
    }
    uint32_t rows;
    uint32_t cols;
    is_.read(reinterpret_cast<char*>(&rows), sizeof(rows));
    is_.read(reinterpret_cast<char*>(&cols), sizeof(cols));
    if (leftToRead() < rows*cols*sizeof(float)) {
        return false;
    }
    value->resize(rows, cols);
    for (uint32_t i = 0; i < rows; ++i) {
        for (uint32_t j = 0; j < cols; ++j) {
        float element;
        is_.read(reinterpret_cast<char*>(&element), sizeof(element));
            (*value)(i, j) = element;
        }
    }
    return true;
}

template<>
bool StreamReader::read<cv::Mat>(cv::Mat * value) {
    int32_t type;
    int32_t rows;
    int32_t cols;
    int32_t elemSize;

    if (leftToRead() < 4*sizeof(int32_t)) {
        return false;
    }

    is_.read(reinterpret_cast<char*>(&type), sizeof(type));
    is_.read(reinterpret_cast<char*>(&rows), sizeof(rows));
    is_.read(reinterpret_cast<char*>(&cols), sizeof(cols));

    is_.read(reinterpret_cast<char*>(&elemSize), sizeof(elemSize));
    if (leftToRead() < static_cast<unsigned int>(elemSize * rows * cols)) {
        return false;
    }

    *value = cv::Mat(rows, cols, type);
    is_.read(reinterpret_cast<char*>(value->data), elemSize * rows * cols);
    return true;
}

template<>
bool StreamReader::read<cv::Rect>(cv::Rect * value) {
    if (leftToRead() < 4*sizeof(int)) {
        return false;
    }
    int width;
    int height;
    int x;
    int y;
    is_.read(reinterpret_cast<char*>(&width), sizeof(width));
    is_.read(reinterpret_cast<char*>(&height), sizeof(height));
    is_.read(reinterpret_cast<char*>(&x), sizeof(x));
    is_.read(reinterpret_cast<char*>(&y), sizeof(y));
    *value = cv::Rect(x, y, width, height);
    return true;
}

/*
 * Reading an arry
 */
template<class T>
bool StreamReader::read(T * value, uint32_t size) {
    if (!std::is_arithmetic<T>::value) {
        return false;
    }
    if (leftToRead() < size*sizeof(T)) {
        return false;
    }
    is_.read(reinterpret_cast<char*>(value), size * sizeof(T));
    return true;
}

template<class T>
bool StreamReader::read(std::vector<T> * value) {
    value->clear();
    if (leftToRead() < sizeof(uint32_t)) {
        return false;
    }
    uint32_t size;
    is_.read(reinterpret_cast<char*>(&size), sizeof(size));

    value->resize(size);
    for (uint32_t i = 0; i < size; ++i) {
        T tmp;
        if (std::is_arithmetic<T>::value) {
            is_.read(reinterpret_cast<char*>(&tmp), sizeof(T));
        } else {
            if (!read<T>(&tmp)) {
                return false;
            }
        }
        value->at(i) = tmp;
    }
    return true;
}

template<>
bool StreamReader::read<cv::Mat>(std::vector<cv::Mat> * value) {
    value->clear();
    uint32_t size;
    is_.read(reinterpret_cast<char*>(&size), sizeof(size));
    for (uint32_t i = 0; i < size; ++i) {
        cv::Mat tmp;
        read<cv::Mat>(&tmp);
        value->push_back(tmp);
    }
    return true;
}

template bool StreamReader::read<int8_t>(int8_t * value);
template bool StreamReader::read<int16_t>(int16_t * value);
template bool StreamReader::read<int32_t>(int32_t * value);
template bool StreamReader::read<int64_t>(int64_t * value);
template bool StreamReader::read<uint8_t>(uint8_t * value);
template bool StreamReader::read<uint16_t>(uint16_t * value);
template bool StreamReader::read<uint32_t>(uint32_t * value);
template bool StreamReader::read<uint64_t>(uint64_t * value);
template bool StreamReader::read<float>(float * value);
template bool StreamReader::read<double>(double * value);

template bool StreamReader::read<int32_t>(int32_t * value, uint32_t size);
template bool StreamReader::read<double>(double * value, uint32_t size);

template bool StreamReader::read<int8_t>(std::vector<int8_t> * value);
template bool StreamReader::read<int16_t>(std::vector<int16_t> * value);
template bool StreamReader::read<int32_t>(std::vector<int32_t> * value);
template bool StreamReader::read<int64_t>(std::vector<int64_t> * value);
template bool StreamReader::read<uint8_t>(std::vector<uint8_t> * value);
template bool StreamReader::read<uint16_t>(std::vector<uint16_t> * value);
template bool StreamReader::read<uint32_t>(std::vector<uint32_t> * value);
template bool StreamReader::read<uint64_t>(std::vector<uint64_t> * value);
template bool StreamReader::read<float>(std::vector<float> * value);
template bool StreamReader::read<double>(std::vector<double> * value);
template bool StreamReader::read<Eigen::VectorXf>(std::vector<Eigen::VectorXf> * value);
template bool StreamReader::read<Eigen::MatrixXf>(std::vector<Eigen::MatrixXf> * value);
template bool StreamReader::read<KDL::Frame>(std::vector<KDL::Frame> * value);
template bool StreamReader::read<cv::Mat>(std::vector<cv::Mat> * value);
template bool StreamReader::read<cv::Rect>(std::vector<cv::Rect> * value);

}  // namespace communicationutility
}  // namespace utility
}  // namespace crf
