/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Krzysztof Szczurek CERN EN/SMM/MRO 2019
 *         Jorge Camarero Vera CERN EN/SMM/MRO 2019
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <limits>
#include <string>
#include <type_traits>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <eigen3/Eigen/Core>
#include <kdl/frames.hpp>
#include <opencv2/core.hpp>

#include "CommunicationUtility/StreamReader.hpp"
#include "CommunicationUtility/StreamWriter.hpp"
#include "EventLogger/EventLogger.hpp"


class StreamReaderWriterShould: public ::testing::Test {
 protected:
    StreamReaderWriterShould() :
        logger_("StreamReaderWriterShould") {
        logger_->info("{} BEGIN", testing::UnitTest::GetInstance()->current_test_info()->name());
    }
    ~StreamReaderWriterShould() {
        logger_->info("{} END with {}",
            testing::UnitTest::GetInstance()->current_test_info()->name(),
            testing::UnitTest::GetInstance()->current_test_info()->result()->Passed());
    }

    template<class T>
    bool successFailure(const T value, const T valueRead) {
        if (value == valueRead) {
            logger_->info("Success!");
            return true;
        } else {
            logger_->error("Failure!");
            return false;
        }
    }

    template<class T>
    bool checkSingle(const T & value) {
        logger_->info("Checking type: ", typeid(value).name());
        crf::utility::communicationutility::StreamWriter writer;
        writer.write(value);
        std::string buffer(writer.toString());
        crf::utility::communicationutility::StreamReader reader(buffer);
        T valueRead;
        reader.read(&valueRead);
        return successFailure(value, valueRead);
    }

    bool checkString(const std::string & value) {
        logger_->info("Checking string");
        crf::utility::communicationutility::StreamWriter writer;
        writer.write(value);
        std::string buffer(writer.toString());
        crf::utility::communicationutility::StreamReader reader(buffer);
        std::string valueRead;
        reader.read(&valueRead);
        return successFailure(value, valueRead);
    }

    template<class T>
    bool checkArray(T * value, uint32_t size) {
        logger_->info("Checking array with elements of type ", typeid(T).name());
        crf::utility::communicationutility::StreamWriter writer;
        writer.write(value, size);
        std::string buffer(writer.toString());
        crf::utility::communicationutility::StreamReader reader(buffer);
        T * valueRead = new T[size];
        bool result = true;
        result = reader.read(valueRead, size);
        if (result) {
            for (auto i = 0; i < size; ++i) {
                if (value[i] != valueRead[i]) {
                    result = false;
                }
            }
        }
        if (valueRead != 0) {
            delete [] valueRead;
        }
        return successFailure(true, result);
    }

    bool checkKDLFrame(const KDL::Frame & value) {
        logger_->info("Checking KDL::Frame");
        crf::utility::communicationutility::StreamWriter writer;
        writer.write(value);
        std::string buffer(writer.toString());
        crf::utility::communicationutility::StreamReader reader(buffer);
        KDL::Frame valueRead;
        bool result = true;
        reader.read(&valueRead);
        return successFailure(value, valueRead);
    }

    bool checkEigenVectorXf(const Eigen::VectorXf & value) {
        logger_->info("Checking Eigen::VectorXf");
        crf::utility::communicationutility::StreamWriter writer;
        writer.write(value);
        std::string buffer(writer.toString());
        crf::utility::communicationutility::StreamReader reader(buffer);
        Eigen::VectorXf valueRead(value.size());
        bool result = true;
        reader.read(&valueRead);
        if (value != valueRead) {
            result = false;
        }
        return successFailure(true, result);
    }

    bool checkEigenMatrixXf(const Eigen::MatrixXf & value) {
        logger_->info("Checking Eigen::MatrixXf");
        crf::utility::communicationutility::StreamWriter writer;
        writer.write(value);
        std::string buffer(writer.toString());
        crf::utility::communicationutility::StreamReader reader(buffer);
        Eigen::MatrixXf valueRead;
        bool result = true;
        reader.read(&valueRead);
        if (value != valueRead) {
            result = false;
        }
        return successFailure(true, result);
    }

    template <class T>
    bool compareCvMat(const cv::Mat & mat1, const cv::Mat & mat2) {
        bool result = true;
        for (auto i = 0; i < 3; ++i) {
            for (auto j = 0; j < 4; ++j) {
                if (mat1.at<T>(i, j) != mat2.at<T>(i, j)) {
                    result = false;
                }
            }
        }
        return result;
    }

    template <class T>
    bool checkCvMat(const cv::Mat & value) {
        logger_->info("Checking cv::Mat");
        crf::utility::communicationutility::StreamWriter writer;
        writer.write(value);
        std::string buffer(writer.toString());
        crf::utility::communicationutility::StreamReader reader(buffer);
        cv::Mat valueRead;
        reader.read(&valueRead);
        bool result = compareCvMat<T>(value, valueRead);
        return successFailure(true, result);
    }

    bool compareCvRect(const cv::Rect & rect1, const cv::Rect & rect2) {
        return  rect1.width == rect2.width && rect1.height == rect2.height &&
            rect1.x == rect2.x && rect1.y == rect2.y;
    }

    bool checkCvRect(const cv::Rect & value) {
        logger_->info("Checking cv::Rect");
        crf::utility::communicationutility::StreamWriter writer;
        writer.write(value);
        std::string buffer(writer.toString());
        crf::utility::communicationutility::StreamReader reader(buffer);
        cv::Rect valueRead;
        reader.read(&valueRead);
        return successFailure(true, compareCvRect(value, valueRead));
    }

    /*
     * Tests vector of arithmetic, Eigen::VectorXf, Eigen::MatrixXf and KDL::Frame types
     */
    template<class T>
    bool checkVector(const std::vector<T> & value) {
        logger_->info("Checking vector with elements of type ", typeid(T).name());
        crf::utility::communicationutility::StreamWriter writer;
        writer.write(value);
        std::string buffer(writer.toString());
        crf::utility::communicationutility::StreamReader reader(buffer);
        std::vector<T> valueRead;
        reader.read(&valueRead);
        bool result = true;
        if (std::is_arithmetic<T>::value || std::is_same<T, Eigen::VectorXf>::value ||
            std::is_same<T, Eigen::MatrixXf>::value || std::is_same<T, KDL::Frame>::value) {
            result = value == valueRead;
        } else {
            return false;
        }
        return successFailure(true, result);
    }

    /*
     * Tests vector of cv::Mat type serialization
     */
    template<class T>
    bool checkVectorCvMat(const std::vector<cv::Mat> & value) {
        logger_->info("Checking vector with elements of type cv::Mat");
        crf::utility::communicationutility::StreamWriter writer;
        writer.write(value);
        std::string buffer(writer.toString());
        crf::utility::communicationutility::StreamReader reader(buffer);
        std::vector<cv::Mat> valueRead;
        reader.read(&valueRead);
        bool result = true;
        for (auto i = 0; i < value.size(); ++i) {
            if (!compareCvMat<T>(value[i], valueRead[i])) {
                result = false;
            }
        }
        return successFailure(true, result);
    }

    /*
     * Tests vector of cv::Rect type serialization
     */
    bool checkVectorCvRect(const std::vector<cv::Rect> & value) {
        logger_->info("Checking vector with elements of type cv::Rect");
        crf::utility::communicationutility::StreamWriter writer;
        writer.write(value);
        std::string buffer(writer.toString());
        crf::utility::communicationutility::StreamReader reader(buffer);
        std::vector<cv::Rect> valueRead;
        reader.read(&valueRead);
        bool result = true;
        for (auto i = 0; i < value.size(); ++i) {
            if (!compareCvRect(value[i], valueRead[i])) {
                result = false;
            }
        }
        return successFailure(true, result);
    }

    crf::utility::logger::EventLogger logger_;
};

TEST_F(StreamReaderWriterShould, Single) {
    EXPECT_TRUE(checkSingle((int8_t)std::numeric_limits<int8_t>::max()));
    EXPECT_TRUE(checkSingle((int8_t)std::numeric_limits<int8_t>::min()));
    EXPECT_TRUE(checkSingle((int16_t)std::numeric_limits<int16_t>::max()));
    EXPECT_TRUE(checkSingle((int16_t)std::numeric_limits<int16_t>::min()));
    EXPECT_TRUE(checkSingle((int32_t)std::numeric_limits<int32_t>::max()));
    EXPECT_TRUE(checkSingle((int32_t)std::numeric_limits<int32_t>::min()));
    EXPECT_TRUE(checkSingle((int64_t)std::numeric_limits<int64_t>::max()));
    EXPECT_TRUE(checkSingle((int64_t)std::numeric_limits<int64_t>::min()));
    EXPECT_TRUE(checkSingle((uint8_t)std::numeric_limits<uint8_t>::max()));
    EXPECT_TRUE(checkSingle((uint8_t)std::numeric_limits<uint8_t>::min()));
    EXPECT_TRUE(checkSingle((uint16_t)std::numeric_limits<uint16_t>::max()));
    EXPECT_TRUE(checkSingle((uint16_t)std::numeric_limits<uint16_t>::min()));
    EXPECT_TRUE(checkSingle((uint32_t)std::numeric_limits<uint32_t>::max()));
    EXPECT_TRUE(checkSingle((uint32_t)std::numeric_limits<uint32_t>::min()));
    EXPECT_TRUE(checkSingle((uint64_t)std::numeric_limits<uint64_t>::max()));
    EXPECT_TRUE(checkSingle((uint64_t)std::numeric_limits<uint64_t>::min()));
    EXPECT_TRUE(checkSingle(static_cast<float>(std::numeric_limits<float>::max())));
    EXPECT_TRUE(checkSingle(static_cast<float>(std::numeric_limits<float>::min())));
    EXPECT_TRUE(checkSingle(static_cast<double>(std::numeric_limits<double>::max())));
    EXPECT_TRUE(checkSingle(static_cast<double>(std::numeric_limits<double>::min())));
}

TEST_F(StreamReaderWriterShould, String) {
    std::string s = "This is a test string";
    EXPECT_TRUE(checkString(s));
}

TEST_F(StreamReaderWriterShould, Array) {
    int32_t a1[3] = { -1, 4, 6 };
    double a2[4] = { -1.6, 4.6, 6.9, 45334.6 };
    EXPECT_TRUE(checkArray(a1, 3));
    EXPECT_TRUE(checkArray(a2, 4));
}

TEST_F(StreamReaderWriterShould, KDLFrame) {
    KDL::Vector position;
    position.x(0.1);
    position.y(0.2);
    position.z(0.3);
    KDL::Rotation rotation(0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2);
    KDL::Frame testFrame(rotation, position);
    EXPECT_TRUE(checkKDLFrame(testFrame));
}

TEST_F(StreamReaderWriterShould, EigenVectorXf) {
    Eigen::VectorXf vec(6);
    vec << 0.2, 65.4, 76.3, 87.3, -56.8, 68;
    EXPECT_TRUE(checkEigenVectorXf(vec));
}

TEST_F(StreamReaderWriterShould, EigenMatrixXf) {
    Eigen::MatrixXf m;
    m.resize(4, 4);
    for (auto i = 0; i < 4; ++i) {
        for (auto j = 0; j < 4; ++j) {
            m(i, j) = (i + 1.0) * (j + 1.0);
        }
    }
    EXPECT_TRUE(checkEigenMatrixXf(m));
}

TEST_F(StreamReaderWriterShould, cvMat) {
    double m[3][4] = { {1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12} };
    cv::Mat testMat(3, 4, CV_64F, m);
    EXPECT_TRUE(checkCvMat<double>(testMat));
}

TEST_F(StreamReaderWriterShould, cvRect) {
    cv::Rect testRect(1, 7, 284, 2945);
    EXPECT_TRUE(checkCvRect(testRect));
}

TEST_F(StreamReaderWriterShould, VectorArithmetic) {
    std::vector<int32_t> v1{ 123, -567, 6947 };
    EXPECT_TRUE(checkVector(v1));

    std::vector<double> v2{ 12.5344, -5.355, 0.654, 1e10 };
    EXPECT_TRUE(checkVector(v2));

    std::vector<int16_t> v3{ -5 };
    EXPECT_TRUE(checkVector(v3));

    std::vector<uint64_t> v4{ std::numeric_limits<uint64_t>::max(),
    std::numeric_limits<uint64_t>::min() };
    EXPECT_TRUE(checkVector(v4));
}

TEST_F(StreamReaderWriterShould, VectorEigenVectorXf) {
    std::vector<Eigen::VectorXf> v;
    for (auto i = 0; i < 5; ++i) {
        Eigen::VectorXf vec(6);
        vec << i, i + 1, i + 2, i + 3, i + 4, i + 5;
        v.push_back(vec);
    }
    EXPECT_TRUE(checkVector(v));
}

TEST_F(StreamReaderWriterShould, VectorEigenMatrixXf) {
    std::vector<Eigen::MatrixXf> v;
    Eigen::MatrixXf m[3];
    for (auto k = 0; k < 3; ++k) {
        m[k].resize(3, 5);
        for (auto i = 0; i < 3; ++i) {
            for (auto j = 0; j < 5; ++j) {
                m[k](i, j) = (k + 1.0) * (i + 1.0) * (j + 1.0);
            }
        }
        v.push_back(m[k]);
    }
    EXPECT_TRUE(checkVector(v));
}

TEST_F(StreamReaderWriterShould, VectorKDLFrame) {
    std::vector<KDL::Frame> v;
    for (auto i = 1; i < 5; ++i) {
        KDL::Vector position;
        position.x(0.1 * i);
        position.y(0.2 * i);
        position.z(0.3 * i);
        KDL::Rotation rotation(0.4 * i, 0.5 * i, 0.6 * i, 0.7 * i,
            0.8 * i, 0.9 * i, 1.0 * i, 1.1 * i, 1.2 * i);
        KDL::Frame testFrame(rotation, position);
        v.push_back(testFrame);
    }
    EXPECT_TRUE(checkVector(v));
}

TEST_F(StreamReaderWriterShould, VectorCvMatCV_64F) {
    std::vector<cv::Mat> v;
    double m[3][4];
    cv::Mat testMat1[3];
    for (auto i = 0; i < 3; ++i) {
        for (auto j = 0; j < 3; ++j) {
            for (auto k = 0; k < 4; ++k) {
                m[j][k] = (i + 1) * (j + 1) * (k + 1);
            }
        }
        testMat1[i] = cv::Mat(3, 4, CV_64F, m);
        v.push_back(testMat1[i]);
    }
    EXPECT_TRUE(checkVectorCvMat<double>(v));
}

TEST_F(StreamReaderWriterShould, VectorCvMatCV_8UC3) {
    std::vector<cv::Mat> v;
    cv::Vec<uchar, 3> cvVec;
    cv::Mat testMat2[5];
    for (auto i = 0; i < 5; ++i) {
        for (auto j = 0; j < 3; ++j) {
            for (auto k = 0; k < 4; ++k) {
                for (auto l = 0; l < 3; ++l) {
                    cvVec[l] = (i + 1) * (j + 1) * (k + 1) + l;
                }
            }
        }
        testMat2[i] =  cv::Mat(3, 4, CV_64F, cvVec);
        v.push_back(testMat2[i]);
    }
    EXPECT_TRUE(checkVectorCvMat<cv::Vec3b>(v));
}

TEST_F(StreamReaderWriterShould, VectorCvRect) {
    std::vector<cv::Rect> v;
    cv::Rect testRect[3];
    for (auto i = 0; i < 3; ++i) {
        testRect[i] = cv::Rect(1.6 * i, 7.4 * i, 284.6 * i, 2945.3 * i);
        v.push_back(testRect[i]);
    }
    EXPECT_TRUE(checkVectorCvRect(v));
}

TEST_F(StreamReaderWriterShould, ReadAndWriteDifferentMixedTypesCorrectly) {
    uint8_t expectedUint = 69;
    float expectedFloat = -69.69;
    std::string expectedString("xxx yyy 666");
    std::vector<double> expectedVectorDouble({0.1, 0.2, 0.3, 0.4, 0.5});
    double m[3][4] = { {0.1, 0.2, 0.3, 0.4}, {5, 6, 7, 8}, {0.9, 0.1, 0.11, 0.12} };
    cv::Mat expectedMat(3, 4, CV_64F, m);
    crf::utility::communicationutility::StreamWriter writer;
    writer.write(expectedUint);
    writer.write(expectedFloat);
    writer.write(expectedString);
    writer.write(expectedVectorDouble);
    writer.write(expectedMat);
    crf::utility::communicationutility::StreamReader reader(writer.toString());
    uint8_t obtainedUint;
    float obtainedFloat;
    std::string obtainedString;
    std::vector<double> obtainedVectorDouble;
    cv::Mat obtainedMat;
    reader.read(&obtainedUint);
    reader.read(&obtainedFloat);
    reader.read(&obtainedString);
    reader.read(&obtainedVectorDouble);
    reader.read(&obtainedMat);
    ASSERT_EQ(expectedUint, obtainedUint);
    ASSERT_EQ(expectedFloat, obtainedFloat);
    ASSERT_EQ(expectedVectorDouble, obtainedVectorDouble);
    ASSERT_TRUE(compareCvMat<double>(expectedMat, obtainedMat));
}
