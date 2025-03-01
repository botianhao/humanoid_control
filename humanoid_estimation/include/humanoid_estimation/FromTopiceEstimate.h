//
// Created by qiayuan on 2022/7/24.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "humanoid_estimation/StateEstimateBase.h"



#pragma once
namespace ocs2
{
namespace humanoid
{

class FromTopicStateEstimate : public StateEstimateBase
{
public:
  FromTopicStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                         const PinocchioEndEffectorKinematics& eeKinematics,  const rclcpp::Node::SharedPtr &node);

  void updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal,
                 const vector3_t& linearAccelLocal, const matrix3_t& orientationCovariance,
                 const matrix3_t& angularVelCovariance, const matrix3_t& linearAccelCovariance) override{};

  vector_t update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  void callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  nav_msgs::msg::Odometry buffer_;
};

}  // namespace humanoid
}  // namespace ocs2
