//
// Created by qiayuan on 2022/6/24.
//

#pragma once


#include "rclcpp/rclcpp.hpp"
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <humanoid_dummy/visualization/HumanoidVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <humanoid_estimation/StateEstimateBase.h>
#include <humanoid_interface/HumanoidInterface.h>
#include <humanoid_wbc/WbcBase.h>

#include "humanoid_controllers/SafetyChecker.h"
#include "humanoid_controllers/visualization/humanoidSelfCollisionVisualization.h"

#include "std_msgs/msg/int8_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/imu.hpp"



namespace humanoid_controller{
using namespace ocs2;
using namespace humanoid;

class humanoidController{
 public:
  humanoidController() = default;
  ~humanoidController();
  bool init(rclcpp::Node::SharedPtr &controller_nh);
  void update(const rclcpp::Time& time, const rclcpp::Duration& period);
  void starting(const rclcpp::Time& time);
  void stopping(const rclcpp::Time& /*time*/)  { mpcRunning_ = false; }

 protected:
  virtual void updateStateEstimation(const rclcpp::Time& time, const rclcpp::Duration& period);

  virtual void setupHumanoidInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  void jointStateCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void HwSwitchCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // Interface
  std::shared_ptr<HumanoidInterface> HumanoidInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;

  // State Estimation
  SystemObservation currentObservation_;
  vector_t measuredRbdState_;
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

  // Whole Body Control
  std::shared_ptr<WbcBase> wbc_;
  std::shared_ptr<SafetyChecker> safetyChecker_;

  // Nonlinear MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // Visualization
  std::shared_ptr<HumanoidVisualizer> robotVisualizer_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr observationPublisher_;

  //Controller Interface
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr targetTorquePub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr targetPosPub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr targetVelPub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr targetKpPub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr targetKdPub_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr vs_xianyanPub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr vs_basePub_;//估计的足端速度
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr foot_vel_estimatePub_;//估计的足端速度
  rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr cmd_contactFlagPub_;//命令的接触状态
  
  
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr jointPosVelSub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hwSwitchSub_;

  bool hwSwitch_ = false;

  // Node Handle
  rclcpp::Node::SharedPtr controllerNh_;


 private:
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  benchmark::RepeatedTimer mpcTimer_;
  benchmark::RepeatedTimer wbcTimer_;
  size_t jointNum_ = 12;
  vector_t jointPos_, jointVel_;
  Eigen::Quaternion<scalar_t> quat_;
  contact_flag_t contactFlag_;
  vector3_t angularVel_, linearAccel_;
  matrix3_t orientationCovariance_, angularVelCovariance_, linearAccelCovariance_;
  size_t plannedMode_ = 3;
  vector_t defalutJointPos_;
  vector_t joint_pos_bias_;
};

class humanoidCheaterController : public humanoidController {
 protected:
  void setupStateEstimate(const std::string& taskFile, bool verbose) override;
};

}  // namespace humanoid_controller

