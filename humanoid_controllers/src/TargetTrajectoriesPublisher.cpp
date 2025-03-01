//
// Created by qiayuan on 2022/7/24.
//

#include "humanoid_controllers/TargetTrajectoriesPublisher.h"
#include "humanoid_controllers/humanoidController.h"

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>


#include <std_msgs/msg/string.hpp>

using namespace ocs2;
using namespace humanoid;

namespace {
scalar_t TARGET_DISPLACEMENT_VELOCITY;
scalar_t TARGET_ROTATION_VELOCITY;
scalar_t COM_HEIGHT;
vector_t DEFAULT_JOINT_STATE(12);
scalar_t TIME_TO_TARGET;
}  // namespace

scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) {
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(3);
  const scalar_t rotationTime = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY;
  return std::max(rotationTime, displacementTime);
}

TargetTrajectories targetPoseToTargetTrajectories(const vector_t& targetPose, const SystemObservation& observation,
                                                  const scalar_t& targetReachingTime) {
  // desired time trajectory
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // desired state trajectory
  vector_t currentPose = observation.state.segment<6>(6);
  currentPose(2) = COM_HEIGHT;
  currentPose(4) = 0;
  currentPose(5) = 0;
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << vector_t::Zero(6), currentPose, DEFAULT_JOINT_STATE;
  stateTrajectory[1] << vector_t::Zero(6), targetPose, DEFAULT_JOINT_STATE;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

TargetTrajectories goalToTargetTrajectories(const vector_t& goal, const SystemObservation& observation) {
  const vector_t currentPose = observation.state.segment<6>(6);
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = goal(0);
    target(1) = goal(1);
    target(2) = COM_HEIGHT;
    target(3) = goal(3);
    target(4) = 0;
    target(5) = 0;
    return target;
  }();
  const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);
  return targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
}



std::string gait_mode_ = "stance";//表示当前的默认指令步态
//std::string gait_mode_last = "stance";//表示当前的默认指令步态
TargetTrajectories cmdVelToTargetTrajectories(const vector_t& cmdVel, const SystemObservation& observation) {
    const vector_t currentPose = observation.state.segment<6>(6);
  const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
  vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3);

  const scalar_t timeToTarget = TIME_TO_TARGET;
    
  
  vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = currentPose(0) + cmdVelRot(0) * timeToTarget;
    target(1) = currentPose(1) + cmdVelRot(1) * timeToTarget;
    target(2) = COM_HEIGHT;
    target(3) = currentPose(3) + cmdVel(3) * timeToTarget;
    target(4) = 0;
    target(5) = 0;
    
    return target;
  }();

  static auto targetPose_= currentPose;




  if(gait_mode_ != "stance"){//根据当前的步态来决定绝对还是相对
    targetPose_(0) += cmdVelRot(0) * 1.0 / 100.0;
    targetPose_(1) += cmdVelRot(1) * 1.0 / 100.0;//绝对的位置控制 
    targetPose_(3) += cmdVel(3) * 1.0 / 100.0;//yaw轴绝对的角度控制

    targetPose(0) = targetPose_(0);
    targetPose(1) = targetPose_(1);
    targetPose(3) = targetPose_(3);
  }else{
    targetPose_ = currentPose;
  }



  // target reaching duration
  const scalar_t targetReachingTime = observation.time + timeToTarget;
  auto trajectories = targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
      trajectories.stateTrajectory[0].head(3) = cmdVelRot;
    trajectories.stateTrajectory[1].head(3) = cmdVelRot;
    return trajectories;
}



void _Gait_modeCallback(std_msgs::msg::String::SharedPtr msg){

  gait_mode_ = msg->data;//获取当前的步态
}



int main(int argc, char** argv) {
  const std::string robotName = "humanoid";

  // Initialize ros node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
    robotName + "_target",
    rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true));

  // Get node parameters
  std::string referenceFile = node->get_parameter("referenceFile").as_string();
  std::string taskFile = node->get_parameter("taskFile").as_string();

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr 
  modeSequenceSubscriber_ = node->create_subscription<std_msgs::msg::String>(robotName + "_gait_mode_schedule", 5, &_Gait_modeCallback);



  loadData::loadCppDataType(referenceFile, "comHeight", COM_HEIGHT);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", DEFAULT_JOINT_STATE);
  loadData::loadCppDataType(referenceFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
  loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);
  loadData::loadCppDataType(taskFile, "mpc.timeHorizon", TIME_TO_TARGET);

  TargetTrajectoriesPublisher target_pose_command(node, robotName, &goalToTargetTrajectories, &cmdVelToTargetTrajectories);

  rclcpp::spin(node);
  // Successful exit
  return 0;
}
