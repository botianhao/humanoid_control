//
// Created by qiayuan on 2022/7/24.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include "humanoid_estimation/StateEstimateBase.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>



#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace ocs2
{
namespace humanoid
{

class KalmanFilterEstimate : public StateEstimateBase
{
public:
  KalmanFilterEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                       const PinocchioEndEffectorKinematics& eeKinematics , const rclcpp::Node::SharedPtr &node);

  vector_t update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  void loadSettings(const std::string& taskFile, bool verbose);





protected:
  void updateFromTopic();

  void callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  nav_msgs::msg::Odometry getOdomMsg();

  vector_t feetHeights_;

  // Config
  scalar_t footRadius_ = 0.02;
  scalar_t imuProcessNoisePosition_ = 0.02;
  scalar_t imuProcessNoiseVelocity_ = 0.02;
  scalar_t footProcessNoisePosition_ = 0.002;
  scalar_t footSensorNoisePosition_ = 0.005;
  scalar_t footSensorNoiseVelocity_ = 0.1;
  scalar_t footHeightSensorNoise_ = 0.01;

private:
  Eigen::Matrix<scalar_t, 18, 1> xHat_;
  Eigen::Matrix<scalar_t, 12, 1> ps_;
  Eigen::Matrix<scalar_t, 12, 1> vs_;
  Eigen::Matrix<scalar_t, 18, 18> a_;
  Eigen::Matrix<scalar_t, 18, 18> q_;
  Eigen::Matrix<scalar_t, 18, 18> p_;
  Eigen::Matrix<scalar_t, 28, 28> r_;
  Eigen::Matrix<scalar_t, 18, 3> b_;
  Eigen::Matrix<scalar_t, 28, 18> c_;

  // Topic
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  nav_msgs::msg::Odometry buffer_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2::Transform world2odom_;
  std::string frameOdom_, frameGuess_;
  bool topicUpdated_ = false;
};

}  // namespace humanoid
}  // namespace ocs2
