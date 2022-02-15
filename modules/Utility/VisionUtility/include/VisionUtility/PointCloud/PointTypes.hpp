/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Thanapong Chuangyanyong CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#pragma once

#include <string>
#include <vector>
#include <array>

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/instantiate.hpp>

namespace crf {
namespace utility {
namespace visionutility {
namespace pointcloud {
namespace pointtypes {

struct PointXYZS {
    PCL_ADD_POINT4D;
    float s;
    bool isEdge(float sigmaThreshold) {return s >= sigmaThreshold;}
    bool isPlane(float sigmaThreshold) {return s < sigmaThreshold;}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}  // namespace pointtypes
}  // namespace pointcloud
}  // namespace visionutility
}  // namespace utility
}  // namespace crf

POINT_CLOUD_REGISTER_POINT_STRUCT(crf::utility::visionutility::pointcloud::pointtypes::PointXYZS,
    (float, x, x) (float, y, y) (float, z, z) (float, s, s))
