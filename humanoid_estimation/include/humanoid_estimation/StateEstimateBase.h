//
// Created by qiayuan on 2021/11/15.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once


#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/nav_msgs/msg/odometry.hpp>



#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <humanoid_interface/common/ModelSettings.h>
#include <humanoid_interface/common/Types.h>
#include <humanoid_interface/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_core/reference/ModeSchedule.h>
#include "std_msgs/msg/float64_multi_array.hpp"

namespace ocs2
{
namespace humanoid
{

class StateEstimateBase
{
public:
  StateEstimateBase(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                    const PinocchioEndEffectorKinematics& eeKinematics , const rclcpp::Node::SharedPtr &node);
  virtual void updateJointStates(const vector_t& jointPos, const vector_t& jointVel);
  virtual void updateContact(contact_flag_t contactFlag)
  {
    contactFlag_ = contactFlag;
  }
  virtual void updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal,
                         const vector3_t& linearAccelLocal, const matrix3_t& orientationCovariance,
                         const matrix3_t& angularVelCovariance, const matrix3_t& linearAccelCovariance);

  virtual vector_t update(const rclcpp::Time& time, const rclcpp::Duration& period) = 0;

  size_t getMode()
  {
    return stanceLeg2ModeNumber(contactFlag_);
  }

  feet_array_t<vector3_t>& getLatestStancePos()
  {
    return latestStanceposition_;
  }

  vector_t getBodyVelWorld()
  {
    vector_t body_vel(6);
    body_vel.head(3) = rbdState_.segment<3>(info_.generalizedCoordinatesNum + 3);
    body_vel.tail(3) = rbdState_.segment<3>(info_.generalizedCoordinatesNum);
    return std::move(body_vel);
  }

  void setStartStopTime4Legs(const feet_array_t<std::array<scalar_t, 2>>& start_stop_time_4_legs)
  {
    StartStopTime4Legs_ = start_stop_time_4_legs;
  }
  void updateCmdContact(contact_flag_t cmd_contact_flag)
  {
    cmdContactflag_ = std::move(cmd_contact_flag);
  }
  void setCmdTorque(const vector_t& cmd_torque)
  {
    cmdTorque_ = cmd_torque;
  }
  void estContactForce(const rclcpp::Duration& period);
  contact_flag_t estContactState(const scalar_t& time);
  void loadSettings(const std::string& taskFile, bool verbose);

  const vector_t& getEstContactForce()
  {
    return estContactforce_;
  }
  const vector_t& getEstDisturbanceTorque()
  {
    return estDisturbancetorque_;
  }

  const std::array<contact_flag_t, 2>& getEarlyLateContact()
  {
    return earlyLatecontact_;
  }


  Eigen::Matrix<scalar_t, 12, 1> vf_;//表示的是足端接触点的线速度
  Eigen::Matrix<scalar_t, 3, 1> vs_xianyan;//表示的先验估计的imu的速度
  Eigen::Matrix<scalar_t, 12, 1> vs_base;//表示的是足端接触点反求出的躯干速度
  Eigen::Matrix<scalar_t, 3, 1> vs_filtered;//表示的是经过低通滤波后的imu（躯干）速度

protected:
  void earlyContactDetection(const ModeSchedule& modeSchedule, scalar_t current_time);
  void lateContactDetection(const ModeSchedule& modeSchedule, scalar_t current_time);
  void updateAngular(const vector3_t& zyx, const vector_t& angularVel);
  void updateLinear(const vector_t& pos, const vector_t& linearVel);
  void publishMsgs(const nav_msgs::msg::Odometry& odom);
  
  PinocchioInterface pinocchioInterface_;
  CentroidalModelInfo info_;
  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;

  vector3_t zyxOffset_ = vector3_t::Zero();
  vector_t rbdState_;
  contact_flag_t contactFlag_{};
  contact_flag_t contactFlag_reliable{};//表示的是实际用于状态估计的接触状态(可靠接触状态)，该状态的目的是为了消除接触瞬间冲击的影响
  uint64_t contact_tick[4];//计数表示接触的时间，若为腾空状态，则清零，若，contactFlag_=1,则该变量不断的+1，目前的代码为计数50(100ms)
                           //则contactFlag_use_to_compute{}置1，

  Eigen::Quaternion<scalar_t> quat_;
  vector3_t angularVelLocal_, linearAccelLocal_;
  matrix3_t orientationCovariance_, angularVelCovariance_, linearAccelCovariance_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr posePub_;

  rclcpp::Time lastPub_;

  feet_array_t<vector3_t> latestStanceposition_;

  vector_t pSCgZinvlast_;
  vector_t vMeasuredLast_;
  vector_t estContactforce_;
  vector_t estDisturbancetorque_;
  vector_t cmdTorque_;

  scalar_t cutoffFrequency_ = 150;
  scalar_t contactThreshold_ = 23;
  contact_flag_t cmdContactflag_{};
  feet_array_t<std::array<scalar_t, 2>> StartStopTime4Legs_;

  scalar_t imuBiasYaw_ = 0;
  scalar_t imuBiasPitch_ = 0;
  scalar_t imuBiasRoll_ = 0;

  std::array<contact_flag_t, 2> earlyLatecontact_;
  std_msgs::msg::Float64MultiArray earlyLateContactMsg_;
  std::deque<std::pair<scalar_t, contact_flag_t>> estConHistory_;
};

template <typename T>
T square(T a)
{
  return a * a;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T>& q)
{
  Eigen::Matrix<SCALAR_T, 3, 1> zyx;

  SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  zyx(0) =
      std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  zyx(1) = std::asin(as);
  zyx(2) =
      std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
  return zyx;
}

}  // namespace humanoid
}  // namespace ocs2