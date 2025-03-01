//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <mutex>


#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>

#include <humanoid_interface/gait/MotionPhaseDefinition.h>
#include "rclcpp/rclcpp.hpp"
#include <stdio.h>

namespace ocs2 {
namespace humanoid {

class TargetTrajectoriesPublisher final {
 public:
  using CmdToTargetTrajectories = std::function<TargetTrajectories(const vector_t& cmd, const SystemObservation& observation)>;

  TargetTrajectoriesPublisher(rclcpp::Node::SharedPtr& node, const std::string& topicPrefix, CmdToTargetTrajectories goalToTargetTrajectories,
                              CmdToTargetTrajectories cmdVelToTargetTrajectories)
      : goalToTargetTrajectories_(std::move(goalToTargetTrajectories)),
        cmdVelToTargetTrajectories_(std::move(cmdVelToTargetTrajectories)),
        buffer_(node->get_clock()),
        tf2_(buffer_) {
    // Trajectories publisher
    targetTrajectoriesPublisher_.reset(new TargetTrajectoriesRosPublisher(node, topicPrefix));

    // observation subscriber

    observationSub_ = node->create_subscription<ocs2_msgs::msg::MpcObservation>(topicPrefix + "_mpc_observation", 1, std::bind(&TargetTrajectoriesPublisher::observationCallback, this, std::placeholders::_1));

    // goal subscriber


    // cmd_vel subscriber


    goalSub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1, std::bind(&TargetTrajectoriesPublisher::goalCallback, this, std::placeholders::_1));
    cmdVelSub_ = node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, std::bind(&TargetTrajectoriesPublisher::cmdVelCallback, this, std::placeholders::_1));
  }


  void observationCallback(const ocs2_msgs::msg::MpcObservation::SharedPtr msg){
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (latestObservation_.time == 0.0) {
      return;
    }

    vector_t cmdVel = vector_t::Zero(4);
    cmdVel[0] = msg->linear.x;
    cmdVel[1] = msg->linear.y;
    cmdVel[2] = msg->linear.z;
    cmdVel[3] = msg->angular.z;

    const auto trajectories = cmdVelToTargetTrajectories_(cmdVel, latestObservation_);
    targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
  };

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (latestObservation_.time == 0.0) {
      return;
    }
    geometry_msgs::msg::PoseStamped pose = *msg;
    try {
      buffer_.transform(pose, pose, "odom", tf2::durationFromSec(0.2));
    } catch (tf2::TransformException& ex) {
      printf("Failure %s\n", ex.what());
      return;
    }

    vector_t cmdGoal = vector_t::Zero(6);
    cmdGoal[0] = pose.pose.position.x;
    cmdGoal[1] = pose.pose.position.y;
    cmdGoal[2] = pose.pose.position.z;
    Eigen::Quaternion<scalar_t> q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
    cmdGoal[3] = q.toRotationMatrix().eulerAngles(0, 1, 2).z();
    cmdGoal[4] = q.toRotationMatrix().eulerAngles(0, 1, 2).y();
    cmdGoal[5] = q.toRotationMatrix().eulerAngles(0, 1, 2).x();

    const auto trajectories = goalToTargetTrajectories_(cmdGoal, latestObservation_);
    targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
  }
 private:
  CmdToTargetTrajectories goalToTargetTrajectories_, cmdVelToTargetTrajectories_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisher_;

  

  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;

  mutable std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;
};

}  // namespace humanoid
}  // namespace ocs2
