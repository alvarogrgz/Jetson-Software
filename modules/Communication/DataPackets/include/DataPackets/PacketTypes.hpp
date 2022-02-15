/* © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Pawel Ptasznik CERN EN/STI/ECE 2017
 *
 *  ==================================================================================================
 */

#pragma once

#include <cstdint>

/*
 * TODO: we can define some rules for numbering, maybe later use enum instead of const short
 */

namespace crf {
namespace communication {
namespace datapackets {

const uint16_t PACKET_ARM_JOINT_BY_JOINT_POSITION_TYPE = 1;
const uint16_t PACKET_ARM_WORLD_COORDINATE_TYPE = 2;
const uint16_t PACKET_MOVE_BASE_TYPE = 3;
const uint16_t PACKET_ARM_GRIPPER_TYPE = 4;
const uint16_t PACKET_MOVE_BASE_PERCENTAGE_TYPE = 5;
const uint16_t PACKET_ARM_JOINT_BY_JOINT_VELOCITY_TYPE = 6;
const uint16_t PACKET_ARM_JOINT_BY_JOINT_POSITION_VEL_TYPE = 7;
const uint16_t PACKET_ROBOT_F_T_BIAS_UPDATE_TYPE = 19;
const uint16_t ROBOT_ARM_STATUS_PACKET_TYPE = 21;
const uint16_t ROBOT_BASE_STATUS_PACKET_TYPE = 22;
const uint16_t ROBOT_BASE_LIFTING_STAGE_STATUS_PACKET_TYPE = 23;
const uint16_t SCHUNK_ARM_STATUS_PACKET_TYPE = 24;
const uint16_t ROBOT_MICROCONTROLLER_STATUS_PACKET_TYPE = 26;
const uint16_t MOTORS_STATUS_PACKET_TYPE = 28;
const uint16_t URROBOT_STATUS_PACKET_TYPE = 29;
const uint16_t URROBOT_DASHBOARD_PACKET_TYPE = 30;

const uint16_t RP_SENSOR_STATUS_PACKET_TYPE = 35;
const uint16_t RP_SENSOR_DISTANCE_STATUS_PACKET_TYPE = 36;
const uint16_t SICK300_LASER_PACKET_TYPE = 37;

const uint16_t SCREWDRIVER_CURRENT_PACKET_TYPE = 41;
const uint16_t SCREWDRIVER_PULSE_PACKET_TYPE = 42;
const uint16_t SCREWDRIVER_STATUS_PACKET_TYPE = 43;

const uint16_t OIL_PUMP_PACKET_TYPE = 44;
const uint16_t ELECTROVALVE_PACKET_TYPE = 47;

const uint16_t REAL_SENSE_POINT_PACKET_TYPE = 48;

const uint16_t ORBBEC_PRO_POINT_PACKET_TYPE = 49;

const uint16_t PACKET_WEIGHT_SCALE_WEIGHT_TYPE = 89;

const uint16_t POINT_3D_PACKET_TYPE = 90;
const uint16_t POINT_2D_PACKET_TYPE = 91;

const uint16_t KUKA_ARM_STATUS_PACKET_TYPE = 92;

const uint16_t COLLISION_STATUS_PACKET_TYPE = 93;

const uint16_t SLAM_TRACKER_PACKET_TYPE = 94;

const uint16_t THERMAL_CAMERA_PACKET_TYPE = 95;

const uint16_t XLS_ADAPTER_PACKET_TYPE = 96;

const uint16_t PACKET_ARM_POSE_WORLD_COORDINATE_TYPE = 104;
const uint16_t PACKET_ARM_MATRIX_WORLD_COORDINATE_TYPE = 105;
const uint16_t PACKET_ARM_WORLD_COORDINATE_RELATIVE_TYPE = 106;

const uint16_t Depth_Point_Packet_TYPE = 107;
const uint16_t RGB_Depth_Point_Packet_TYPE = 108;
const uint16_t JPEG_IMAGE_PACKET = 109;
const uint16_t FRAME_PACKET = 119;
const uint16_t RGBD_FRAME_PACKET = 123;

const uint16_t PACKET_ARM_WORLD_COORDINATE_COLLISION_TYPE = 110;
const uint16_t PACKET_ROBOT_F_T_DATA_TYPE = 111;

const uint16_t CAMERA_SETTING_PACKET = 112;

const uint16_t PACKET_ROBOT_ALIGNMENT_TYPE = 113;
const uint16_t PACKET_ROBOT_WORLD_ALIGNMENT_TYPE = 114;
const uint16_t REALSENSE_STREAM_PACKET_TYPE = 115;
const uint16_t RGBD_CAMERA_SETTING_PACKET = 116;

const uint16_t UWB1D_DISTANCE_PACKET_TYPE = 117;

const uint16_t RAIL_MOTOR_PACKET_TYPE = 118;

const uint16_t CONTROLLINO_STATUS_PACKET_TYPE = 120;
const uint16_t CONTROLLINO_CONFIGURE_DIGITAL_PIN_TYPE = 121;
const uint16_t CONTROLLINO_SET_PIN_VALUE_TYPE = 122;

const uint16_t LINEAR_STAGE_POSITION_PACKET_TYPE = 201;
const uint16_t LINEAR_STAGE_VELOCITY_PACKET_TYPE = 202;
const uint16_t LINEAR_STAGE_STATUS_PACKET_TYPE = 203;

const uint16_t GSM_INFO_STATU_PACKET_TYPE = 204;

const uint16_t SYSTEM_INFO_PACKET_TYPE = 301;

const uint16_t VELODYNE_POSE_TYPE = 401;


const uint16_t User_RGBD_TYPE = 503;

const uint16_t PACKET_AREA_STILL_NO_CLICKED_TYPE = 601;
const uint16_t PACKET_AREA_CLICKED_COORDINATE_TYPE = 602;
const uint16_t PACKET_DIMENSION_OF_INTEREST_AREA_TYPE = 603;

const uint16_t IMU_DATA_PACKET_TYPE = 701;

const uint16_t JSON_PACKET_TYPE = 801;
const uint16_t STREAM_PACKET_TYPE = 802;

const uint16_t LASER_PACKET_TYPE = 901;

const uint16_t UDP_CONNECTION_PACKET_TYPE = 1001;
const uint16_t FETCHWRITE_PACKET_TYPE = 1002;
const uint16_t PACKET_TIME_SYNC_TYPE = 7001;
const uint16_t FIFO_CLOSE_PACKET_TYPE = 7002;

}  // namespace datapackets
}  // namespace communication
}  // namespace crf
