/* © Copyright CERN 2020.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@crf.ch
 *
 * Author: Carlos Prados Sesmero CERN EN/SMM/MRO
 *
 *  ==================================================================================================
*/

#pragma once

#include <vector>
#include <memory>
#include <sys/time.h>
#include <chrono>
#include <string>
#include <sstream>
#include <typeinfo>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>

#include "EventLogger/EventLogger.hpp"

namespace crf {
namespace utility {
namespace visionutility {
namespace pointcloud {
namespace communication {

/**
 * @brief      Saves a cloudin in ply format.
 *
 * @param[in]  cloud      The cloud
 * @param[in]  fileName   The file name
 * @param[in]  binary     True if save in binary, false if save in ASCII
 *
 * @tparam     PointT     Whatever point cloud of pcl
 * 
 * @return     True if success, false if not
 */
template<typename PointT, std::enable_if_t<pcl::traits::has_xyz<PointT>::value>* = nullptr>
bool saveCloudinPlyFormat(const typename pcl::PointCloud<PointT>::Ptr &cloud,
        const std::string &fileName, bool binary = false) {
    crf::utility::logger::EventLogger logger("saveCloudinPlyFormat");
    if ((cloud->size() < 1) || (cloud == nullptr)) {
        logger->warn("Cloud provided invalid");
        return false;
    }

    logger->debug("Saving point cloud in PLY format...");
    if (binary) {
        if (pcl::io::savePLYFileBinary(fileName, *cloud) == -1) {
            logger->error("There was a problem saving the point cloud");
            return false;
        }
    } else {
        if (pcl::io::savePLYFileASCII(fileName, *cloud) == -1) {
            logger->error("There was a problem saving the point cloud");
            return false;
        }
    }

    return true;
}

/**
 * @brief      Saves a cloudin in pcd format.
 *
 * @param[in]  cloud      The cloud
 * @param[in]  fileName   The file name
 * @param[in]  binary     True if save in binary, false if save in ASCII
 *
 * @tparam     PointT     Whatever point cloud of pcl
 * 
 * @return     True if success, false if not
 */
template<typename PointT, std::enable_if_t<pcl::traits::has_xyz<PointT>::value>* = nullptr>
bool saveCloudinPcdFormat(const typename pcl::PointCloud<PointT>::Ptr &cloud,
        const std::string &fileName, bool binary = false) {
    crf::utility::logger::EventLogger logger("saveCloudinPlyFormat");
    if ((cloud->size() < 1) || (cloud == nullptr)) {
        logger->warn("Cloud provided invalid");
        return false;
    }

    logger->debug("Saving point cloud in PLY format...");
    if (!binary) {
        if (pcl::io::savePCDFileBinary(fileName, *cloud) == -1) {
            logger->error("There was a problem saving the point cloud");
            return false;
        }
    } else {
        if (pcl::io::savePCDFileASCII(fileName, *cloud) == -1) {
            logger->error("There was a problem saving the point cloud");
            return false;
        }
    }

    return true;
}

/**
 * @brief      { function_description }
 *
 * @param[in]  inCloud      In cloud
 * @param[in]  origin       The origin
 * @param[in]  orientation  The orientation
 * @param[in]  use_camera   The use camera
 *
 * @tparam     PointT       { description }
 * @tparam     <unnamed>    { description }
 *
 * @return     { description_of_the_return_value }
 */
template<typename PointT, std::enable_if_t<pcl::traits::has_xyz<PointT>::value>* = nullptr>
std::ostringstream serializePointCloud(const typename pcl::PointCloud<PointT>::Ptr &inCloud,
        const Eigen::Vector4f &origin, const Eigen::Quaternionf &orientation,
        bool use_camera = false) {
    crf::utility::logger::EventLogger logger("serializePointCloud");
    logger->debug("Serializing");
    if ((inCloud->size() < 1) || (inCloud == nullptr)) {
        logger->warn("Cloud provided invalid");
        std::ostringstream oss;
        oss << "Empty";
        return oss;
    }

    logger->debug("Serializing point cloud");

    // Start the chrono
    std::chrono::time_point<std::chrono::system_clock> start, end;
    int elapsedMilliseconds {0};
    start = std::chrono::system_clock::now();

    pcl::PCLPointCloud2 cloud;
    pcl::toPCLPointCloud2(*inCloud, cloud);

    std::ostringstream oss;

    unsigned int nr_points  = cloud.width * cloud.height;
    unsigned int point_size = static_cast<unsigned int> (cloud.data.size() / nr_points);

    // Compute the range_grid, if necessary, and then write out the PLY header
    bool doRangeGrid = !use_camera && cloud.height > 1;
    std::vector<pcl::io::ply::int32> rangegrid(nr_points);
    if (doRangeGrid) {
        unsigned int valid_points = 0;

        // Determine the field containing the x-coordinate
        int xfield = pcl::getFieldIndex(cloud, "x");
        if (xfield >= 0 && cloud.fields[xfield].datatype != pcl::PCLPointField::FLOAT32)
            xfield = -1;

        // If no x-coordinate field exists, then assume all points are valid
        if (xfield < 0) {
            for (unsigned int i=0; i < nr_points; ++i)
                rangegrid[i] = i;
            valid_points = nr_points;
        } else {  // Otherwise, look at their x-coordinates to determine if points are valid
            for (std::size_t i=0; i < nr_points; ++i) {
                float value;
                memcpy(&value, &cloud.data[i * point_size + cloud.fields[xfield].offset],
                    sizeof(float));
                if (std::isfinite(value)) {
                    rangegrid[i] = valid_points;
                    ++valid_points;
                } else {
                    rangegrid[i] = -1;
                }
            }
        }
        pcl::PLYWriter writer;
        std::string cloudInfo = writer.generateHeaderBinary(
            cloud, origin, orientation, static_cast<int>(valid_points), true);
        oss << cloudInfo;
    } else {
        pcl::PLYWriter writer;
        std::string cloudInfo = writer.generateHeaderBinary(
            cloud, origin, orientation, static_cast<int>(nr_points), true);
        oss << cloudInfo;
    }

    // Iterate through the points
    for (unsigned int i = 0; i < nr_points; ++i) {
        // Skip writing any invalid points from range_grid
        if (doRangeGrid && rangegrid[i] < 0)
            continue;

        std::size_t total = 0;
        for (std::size_t d = 0; d < cloud.fields.size(); ++d) {
            int count = cloud.fields[d].count;
            if (count == 0)
            count = 1;  // workaround
            if (count > 1) {
                static unsigned int ucount(count);
                oss.write(reinterpret_cast<const char*>(&ucount), sizeof(unsigned int));
            }
            // Ignore invalid padded dimensions that are inherited from binary data
            if (cloud.fields[d].name == "_") {
                total += cloud.fields[d].count;  // jump over this many elements in the string token
                continue;
            }

            for (int c = 0; c < count; ++c) {
                switch (cloud.fields[d].datatype) {
                    case pcl::PCLPointField::INT8:
                    {
                        char value;
                        memcpy(&value, &cloud.data[i * point_size + cloud.fields[d].offset +
                            (total + c) * sizeof(char)], sizeof(char));
                        oss.write(reinterpret_cast<const char*>(&value), sizeof(char));
                        break;
                    }
                    case pcl::PCLPointField::UINT8:
                    {
                        unsigned char value;
                        memcpy(&value, &cloud.data[i * point_size + cloud.fields[d].offset +
                            (total + c) * sizeof(unsigned char)], sizeof(unsigned char));
                        oss.write(reinterpret_cast<const char*>(&value), sizeof(unsigned char));
                        break;
                    }
                    case pcl::PCLPointField::INT16:
                    {
                        int16_t value;
                        memcpy(&value, &cloud.data[i * point_size + cloud.fields[d].offset +
                            (total + c) * sizeof(int16_t)], sizeof(int16_t));
                        oss.write(reinterpret_cast<const char*>(&value), sizeof(int16_t));
                        break;
                    }
                    case pcl::PCLPointField::UINT16:
                    {
                        uint16_t value;
                        memcpy(&value, &cloud.data[i * point_size + cloud.fields[d].offset +
                            (total + c) * sizeof(uint16_t)], sizeof(uint16_t));
                        oss.write(reinterpret_cast<const char*>(&value), sizeof(uint16_t));
                        break;
                    }
                    case pcl::PCLPointField::INT32:
                    {
                        int value;
                        memcpy(&value, &cloud.data[i * point_size + cloud.fields[d].offset +
                            (total + c) * sizeof(int)], sizeof(int));
                        oss.write(reinterpret_cast<const char*>(&value), sizeof(int));
                        break;
                    }
                    case pcl::PCLPointField::UINT32:
                    {
                        if (cloud.fields[d].name.find("rgba") == std::string::npos) {
                            unsigned int value;
                            memcpy(&value, &cloud.data[i * point_size + cloud.fields[d].offset +
                                (total + c) * sizeof(unsigned int)], sizeof(unsigned int));
                            oss.write(reinterpret_cast<const char*>(&value), sizeof(unsigned int));
                        } else {
                            pcl::RGB color;
                            memcpy(&color, &cloud.data[i * point_size + cloud.fields[d].offset +
                                (total + c) * sizeof(unsigned int)], sizeof(pcl::RGB));
                            unsigned char r = color.r;
                            unsigned char g = color.g;
                            unsigned char b = color.b;
                            unsigned char a = color.a;
                            oss.write(reinterpret_cast<const char*>(&r), sizeof(unsigned char));
                            oss.write(reinterpret_cast<const char*>(&g), sizeof(unsigned char));
                            oss.write(reinterpret_cast<const char*>(&b), sizeof(unsigned char));
                            oss.write(reinterpret_cast<const char*>(&a), sizeof(unsigned char));
                        }
                        break;
                    }
                    case pcl::PCLPointField::FLOAT32:
                    {
                        if (cloud.fields[d].name.find("rgb") == std::string::npos) {
                            float value;
                            memcpy(&value, &cloud.data[i * point_size + cloud.fields[d].offset +
                                (total + c) * sizeof(float)], sizeof(float));
                            oss.write(reinterpret_cast<const char*>(&value), sizeof(float));
                        } else {
                            pcl::RGB color;
                            memcpy(&color, &cloud.data[i * point_size + cloud.fields[d].offset +
                                (total + c) * sizeof(float)], sizeof(pcl::RGB));
                            unsigned char r = color.r;
                            unsigned char g = color.g;
                            unsigned char b = color.b;
                            oss.write(reinterpret_cast<const char*>(&r), sizeof(unsigned char));
                            oss.write(reinterpret_cast<const char*>(&g), sizeof(unsigned char));
                            oss.write(reinterpret_cast<const char*>(&b), sizeof(unsigned char));
                        }
                        break;
                    }
                    case pcl::PCLPointField::FLOAT64:
                    {
                        double value;
                        memcpy(&value, &cloud.data[i * point_size + cloud.fields[d].offset +
                            (total + c) * sizeof(double)], sizeof(double));
                        oss.write(reinterpret_cast<const char*>(&value), sizeof(double));
                        break;
                    }
                    default:
                        logger->warn("Incorrect field data type specified ({})!",
                            cloud.fields[d].datatype);
                        break;
                }
            }
        }
    }

    if (use_camera) {
        // Append sensor information
        float t;
        for (int i = 0; i < 3; ++i) {
            if (origin[3] != 0)
                t = origin[i]/origin[3];
            else
                t = origin[i];
            oss.write(reinterpret_cast<const char*>(&t), sizeof(float));
        }
        Eigen::Matrix3f R = orientation.toRotationMatrix();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                oss.write(reinterpret_cast<const char*>(&R(i, j)), sizeof(float));
            }

        /////////////////////////////////////////////////////
        // Append those properties directly.               //
        // They are for perspective cameras so just put 0  //
        //                                                 //
        // property float focal                            //
        // property float scalex                           //
        // property float scaley                           //
        // property float centerx                          //
        // property float centery                          //
        // and later on                                    //
        // property float k1                               //
        // property float k2                               //
        /////////////////////////////////////////////////////

        const float zerof = 0;
        for (int i = 0; i < 5; ++i)
            oss.write(reinterpret_cast<const char*>(&zerof), sizeof(float));

        // width and height
        int width = cloud.width;
        oss.write(reinterpret_cast<const char*>(&width), sizeof(int));

        int height = cloud.height;
        oss.write(reinterpret_cast<const char*>(&height), sizeof(int));

        for (int i = 0; i < 2; ++i)
            oss.write(reinterpret_cast<const char*>(&zerof), sizeof(float));
    } else if (doRangeGrid) {
        // Write out range_grid
        for (std::size_t i=0; i < nr_points; ++i) {
            pcl::io::ply::uint8 listlen;

            if (rangegrid[i] >= 0) {
                listlen = 1;
                oss.write(reinterpret_cast<const char*>(&listlen), sizeof(pcl::io::ply::uint8));
                oss.write(
                    reinterpret_cast<const char*>(&rangegrid[i]), sizeof(pcl::io::ply::int32));
            } else {
                listlen = 0;
                oss.write(reinterpret_cast<const char*>(&listlen), sizeof(pcl::io::ply::uint8));
            }
        }
    }

    end = std::chrono::system_clock::now();
    elapsedMilliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    logger->debug("Serialize point cloud time: {0} milliseconds", elapsedMilliseconds);

    return oss;
}

}  // namespace communication
}  // namespace pointcloud
}  // namespace visionutility
}  // namespace utility
}  // namespace crf
