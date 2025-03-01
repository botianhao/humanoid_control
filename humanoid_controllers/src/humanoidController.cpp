//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "humanoid_controllers/humanoidController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <humanoid_dummy/gait/GaitReceiver.h>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>
#include <humanoid_estimation/FromTopiceEstimate.h>
#include <humanoid_estimation/LinearKalmanFilter.h>
#include <humanoid_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>
#include <iostream>

namespace humanoid_controller{
using namespace ocs2;
using namespace humanoid;
bool humanoidController::init(rclcpp::Node::SharedPtr &controller_nh) {

  controllerNh_ = controller_nh;
  // Initialize OCS2
  std::string urdfFile = controllerNh_->get_parameter("urdfFile").as_string();
  std::string taskFile = controllerNh_->get_parameter("taskFile").as_string();
  std::string referenceFile = controllerNh_->get_parameter("referenceFile").as_string();


  bool verbose = false;
  loadData::loadCppDataType(taskFile, "humanoid_interface.verbose", verbose);

  joint_pos_bias_ = vector_t(12);
  loadData::loadEigenMatrix(taskFile, "joint_pos_bias", joint_pos_bias_);

  setupHumanoidInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();
  // Visualization
  
  CentroidalModelPinocchioMapping pinocchioMapping(HumanoidInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(HumanoidInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                      HumanoidInterface_->modelSettings().contactNames3DoF);
  robotVisualizer_ = std::make_shared<HumanoidVisualizer>(HumanoidInterface_->getPinocchioInterface(),
                                                             HumanoidInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, controllerNh_);

  defalutJointPos_.resize(jointNum_);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", defalutJointPos_);

  // Hardware interface
  //TODO: setup hardware controller interface
  //create a ROS subscriber to receive the joint pos and vel
  jointPos_ = vector_t::Zero(jointNum_);
  jointPos_ << 0.0, 0.0, 0.37, 0.90, 0.53, 0, 0.0, 0.0, 0.37, 0.90, 0.53, 0;
  jointVel_ = vector_t::Zero(jointNum_);
  quat_ = Eigen::Quaternion<scalar_t>(1, 0, 0, 0);

  jointPosVelSub_ =  controllerNh_->create_subscription<std_msgs::msg::Float32MultiArray>("/jointsPosVel", 2, std::bind(&humanoidController::jointStateCallback, this, std::placeholders::_1));
  imuSub_ = controllerNh_->create_subscription<sensor_msgs::msg::Imu>("/imu", 2, std::bind(&humanoidController::ImuCallback, this, std::placeholders::_1));
  hwSwitchSub_ = controllerNh_->create_subscription<std_msgs::msg::Bool>("/hwswitch", 2, std::bind(&humanoidController::HwSwitchCallback, this, std::placeholders::_1));
  
  targetTorquePub_ = controllerNh_->create_publisher<std_msgs::msg::Float32MultiArray>("/targetTorque", 2);
  targetPosPub_ = controllerNh_->create_publisher<std_msgs::msg::Float32MultiArray>("/targetPos", 2);
  targetVelPub_ = controllerNh_->create_publisher<std_msgs::msg::Float32MultiArray>("/targetVel", 2);
  targetKpPub_ = controllerNh_->create_publisher<std_msgs::msg::Float32MultiArray>("/targetKp", 2);
  targetKdPub_ = controllerNh_->create_publisher<std_msgs::msg::Float32MultiArray>("/targetKd", 2);

  foot_vel_estimatePub_ = controllerNh_->create_publisher<std_msgs::msg::Float32MultiArray>("/foot_vel_estimate", 2);
  cmd_contactFlagPub_ = controllerNh_->create_publisher<std_msgs::msg::Int8MultiArray>("/cmd_contactFlag", 2);
  vs_basePub_ = controllerNh_->create_publisher<std_msgs::msg::Float32MultiArray>("/vs_base", 2);
  vs_xianyanPub_ = controllerNh_->create_publisher<std_msgs::msg::Float32MultiArray>("/vs_xianyan", 2);
  // State estimation
  setupStateEstimate(taskFile, verbose);

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>(HumanoidInterface_->getPinocchioInterface(), HumanoidInterface_->getCentroidalModelInfo(),
                                       *eeKinematicsPtr_);
  wbc_->loadTasksSetting(taskFile, verbose);

  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(HumanoidInterface_->getCentroidalModelInfo());

  return true;
}

void humanoidController::jointStateCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  static float vel_l1 = 0,vel_l2 = 0,vel_l3 = 0, vel_l4 = 0, vel_l5 = 0, vel_l6 = 0,
               vel_r1 = 0,vel_r2 = 0,vel_r3 = 0, vel_r4 = 0, vel_r5 = 0, vel_r6 = 0;
  const float kf_ankle_pitch = 0.3, kf_ankle_roll = 0.3, kf_waist_1 = 0.3,kf_waist_2 = 0.3,kf_waist_3 = 0.3, kf_knee = 0.3;

  if (msg->data.size() != 2 * jointNum_) {
    RCLCPP_ERROR( controllerNh_->get_logger() ,"Received joint state message with wrong size: %ld" , msg->data.size());
    return;
  }
  for (size_t i = 0; i < jointNum_; ++i) {
      jointPos_(i) = msg->data[i];
      jointVel_(i) = msg->data[i + jointNum_];
  }

  //单独给踝关节速度滤波
  vel_l1 = kf_waist_1 * vel_l1 + (1 - kf_waist_1) * msg->data[0 + jointNum_];
  vel_l2 = kf_waist_2 * vel_l2 + (1 - kf_waist_2) * msg->data[1 + jointNum_];
  vel_l3 = kf_waist_3 * vel_l3 + (1 - kf_waist_3) * msg->data[2 + jointNum_];
  
  vel_l4 = kf_knee * vel_l4 + (1 - kf_knee) * msg->data[3 + jointNum_];
  vel_l5 = kf_ankle_pitch * vel_l5 + (1 - kf_ankle_pitch) * msg->data[4 + jointNum_];
  vel_l6 = kf_ankle_roll * vel_l6 + (1 - kf_ankle_roll) * msg->data[5 + jointNum_];

  vel_r1 = kf_waist_1 * vel_r1 + (1 - kf_waist_1) * msg->data[6 + jointNum_];
  vel_r2 = kf_waist_2 * vel_r2 + (1 - kf_waist_2) * msg->data[7 + jointNum_];
  vel_r3 = kf_waist_3 * vel_r3 + (1 - kf_waist_3) * msg->data[8 + jointNum_];

  vel_r4 = kf_knee * vel_r4 + (1 - kf_knee) * msg->data[9 + jointNum_];
  vel_r5 = kf_ankle_pitch * vel_r5 + (1 - kf_ankle_pitch) * msg->data[10 + jointNum_];
  vel_r6 = kf_ankle_roll * vel_r6 + (1 - kf_ankle_roll) * msg->data[11 + jointNum_];

  jointVel_(0) = vel_l1;
  jointVel_(1) = vel_l2;
  jointVel_(2) = vel_l3;
  jointVel_(3) = vel_l4;
  jointVel_(4) = vel_l5;
  jointVel_(5) = vel_l6;

  jointVel_(6) = vel_r1;
  jointVel_(7) = vel_r2;
  jointVel_(8) = vel_r3;
  jointVel_(9) = vel_r4;
  jointVel_(10) = vel_r5;
  jointVel_(11) = vel_r6;

  jointPos_ += joint_pos_bias_;
}

void humanoidController::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    //std::cout << "时间延迟为："  << ( (controllerNh_->now() - msg->header.stamp).nanoseconds() )/ 1000000.0 << std::endl;

    quat_.coeffs().w() = msg->orientation.w;
    quat_.coeffs().x() = msg->orientation.x;
    quat_.coeffs().y() = msg->orientation.y;
    quat_.coeffs().z() = msg->orientation.z;
    angularVel_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    linearAccel_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    orientationCovariance_ << msg->orientation_covariance[0], msg->orientation_covariance[1], msg->orientation_covariance[2],
            msg->orientation_covariance[3], msg->orientation_covariance[4], msg->orientation_covariance[5],
            msg->orientation_covariance[6], msg->orientation_covariance[7], msg->orientation_covariance[8];
    angularVelCovariance_ << msg->angular_velocity_covariance[0], msg->angular_velocity_covariance[1], msg->angular_velocity_covariance[2],
            msg->angular_velocity_covariance[3], msg->angular_velocity_covariance[4], msg->angular_velocity_covariance[5],
            msg->angular_velocity_covariance[6], msg->angular_velocity_covariance[7], msg->angular_velocity_covariance[8];
    linearAccelCovariance_ << msg->linear_acceleration_covariance[0], msg->linear_acceleration_covariance[1], msg->linear_acceleration_covariance[2],
            msg->linear_acceleration_covariance[3], msg->linear_acceleration_covariance[4], msg->linear_acceleration_covariance[5],
            msg->linear_acceleration_covariance[6], msg->linear_acceleration_covariance[7], msg->linear_acceleration_covariance[8];
}

void humanoidController::HwSwitchCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    hwSwitch_ = msg->data;
}

void humanoidController::starting(const rclcpp::Time& time) {
  // Initial state
  currentObservation_.state = vector_t::Zero(HumanoidInterface_->getCentroidalModelInfo().stateDim);
  currentObservation_.state(8) = 0.976;
  currentObservation_.state.segment(6 + 6, jointNum_) = defalutJointPos_;

  updateStateEstimation(time, rclcpp::Duration::from_seconds(0.002));
  currentObservation_.input.setZero(HumanoidInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  RCLCPP_INFO(controllerNh_->get_logger(),"Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && rclcpp::ok()) {
    mpcMrtInterface_->advanceMpc();
    rclcpp::WallRate(HumanoidInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  RCLCPP_INFO(controllerNh_->get_logger(),"Initial policy has been received.");

  mpcRunning_ = true;
}

void humanoidController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {  

  // State Estimate
  updateStateEstimation(time, period);

  // Update the current state of the system
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();

  // Evaluate the current policy
  vector_t optimizedState, optimizedInput;
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode_);

  // Whole body control
  currentObservation_.input = optimizedInput;

  wbcTimer_.startTimer();
  vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode_, period.seconds());
  wbcTimer_.endTimer();

  const vector_t& torque = x.tail(jointNum_);
  const vector_t& wbc_planned_joint_acc = x.segment(6, jointNum_);
  const vector_t& wbc_planned_body_acc = x.head(6);
  const vector_t& wbc_planned_contact_force = x.segment(6 + jointNum_, wbc_->getContactForceSize());


  vector_t posDes = centroidal_model::getJointAngles(optimizedState, HumanoidInterface_->getCentroidalModelInfo());
  vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, HumanoidInterface_->getCentroidalModelInfo());

  scalar_t dt = period.seconds();
  posDes = posDes + 0.5 * wbc_planned_joint_acc * dt * dt;
  velDes = velDes + wbc_planned_joint_acc * dt;
  
  posDes += joint_pos_bias_;

  // Safety check, if failed, stop the controller
  if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
    RCLCPP_ERROR(controllerNh_->get_logger() , "[humanoid Controller] Safety check failed, stopping the controller.");
    //TODO: send the stop command to hardware interface
    return;
  }

    std_msgs::msg::Float32MultiArray targetTorqueMsg;
    for (int i1 = 0; i1 < 12; ++i1) {
        targetTorqueMsg.data.push_back(torque(i1));
    }
    //output targetTorqueMsg
//    std::cerr << "targetTorqueMsg: " << targetTorqueMsg << std::endl;
    std_msgs::msg::Float32MultiArray targetPosMsg;
    for (int i1 = 0; i1 < 12; ++i1) {
        targetPosMsg.data.push_back(posDes(i1));
    }
    std_msgs::msg::Float32MultiArray targetVelMsg;
    for (int i1 = 0; i1 < 12; ++i1) {
        targetVelMsg.data.push_back(velDes(i1));
    }
    if (hwSwitch_){
      targetTorquePub_->publish(targetTorqueMsg);
      targetPosPub_->publish(targetPosMsg);
      targetVelPub_->publish(targetVelMsg);
    }
    
    std_msgs::msg::Float32MultiArray targetKp;
    std_msgs::msg::Float32MultiArray targetKd;

    targetKp.data = {100.0, 100.0, 120.0, 120.0, 5.0, 2.8, 100.0, 100.0, 120.0, 120.0, 5.0, 2.8};
    targetKd.data = {0.7, 0.7, 0.6, 0.6, 0.15, 0.05, 0.7, 0.7, 0.6, 0.6, 0.15, 0.05};
    

    if (hwSwitch_){
      targetKpPub_->publish(targetKp);
      targetKdPub_->publish(targetKd);
    }

    // Visualization
  robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());

  // Publish the observation. Only needed for the command interface
  observationPublisher_->publish(ros_msg_conversions::createObservationMsg(currentObservation_));

//自己加的，可以都删掉
  {
    std_msgs::msg::Float32MultiArray foot_vel,Vs_base,Vs_xianyan;
    std_msgs::msg::Int8MultiArray cmd_contactFlag;
    cmd_contactFlag.data.resize(2);
    foot_vel.data.resize(16);
    Vs_base.data.resize(12);
    Vs_xianyan.data.resize(3);
    for(int i=0;i<12;i++){
      foot_vel.data[i] = stateEstimate_->vf_(i);
      Vs_base.data[i] = stateEstimate_->vs_base(i);
    }
    foot_vel.data[12] = sqrt(foot_vel.data[0]*foot_vel.data[0]+foot_vel.data[1]*foot_vel.data[1]+foot_vel.data[2]*foot_vel.data[2]);
    foot_vel.data[13] = sqrt(foot_vel.data[3]*foot_vel.data[3]+foot_vel.data[4]*foot_vel.data[4]+foot_vel.data[5]*foot_vel.data[5]);
    foot_vel.data[14] = sqrt(foot_vel.data[6]*foot_vel.data[6]+foot_vel.data[7]*foot_vel.data[7]+foot_vel.data[8]*foot_vel.data[8]);
    foot_vel.data[15] = sqrt(foot_vel.data[9]*foot_vel.data[9]+foot_vel.data[10]*foot_vel.data[10]+foot_vel.data[11]*foot_vel.data[11]);

    Vs_xianyan.data[0] = stateEstimate_->vs_xianyan(0);
    Vs_xianyan.data[1] = stateEstimate_->vs_xianyan(1);
    Vs_xianyan.data[2] = stateEstimate_->vs_xianyan(2);

    contact_flag_t  contactFlag = modeNumber2StanceLeg(plannedMode_);
    cmd_contactFlag.data[0] = contactFlag[0];
    cmd_contactFlag.data[1] = contactFlag[1];
    foot_vel_estimatePub_->publish(foot_vel);
    cmd_contactFlagPub_->publish(cmd_contactFlag);
    vs_basePub_->publish(Vs_base);
    vs_xianyanPub_->publish(Vs_xianyan);
  }







}

void humanoidController::updateStateEstimation(const rclcpp::Time& time, const rclcpp::Duration& period) {
  vector_t jointPos(jointNum_), jointVel(jointNum_);
  contact_flag_t contacts;
  Eigen::Quaternion<scalar_t> quat;
  contact_flag_t contactFlag;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;


  jointPos = jointPos_;
  jointVel = jointVel_;
  //TODO: get contactFlag from hardware interface
  //暂时用plannedMode_代替，需要在接触传感器可靠之后修改为stateEstimate_->getMode()
  contactFlag = modeNumber2StanceLeg(plannedMode_);

  quat = quat_;
  angularVel = angularVel_;
  linearAccel = linearAccel_;
  orientationCovariance = orientationCovariance_;
  angularVelCovariance = angularVelCovariance_;
  linearAccelCovariance = linearAccelCovariance_;

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(contactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period);
  currentObservation_.time += period.seconds();
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  //  currentObservation_.mode = stateEstimate_->getMode();
  //TODO: 暂时用plannedMode_代替，需要在接触传感器可靠之后修改为stateEstimate_->getMode()
  currentObservation_.mode =  plannedMode_;
}

humanoidController::~humanoidController() {
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void humanoidController::setupHumanoidInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  HumanoidInterface_ = std::make_shared<HumanoidInterface>(taskFile, urdfFile, referenceFile);
  HumanoidInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void humanoidController::setupMpc() {
  mpc_ = std::make_shared<SqpMpc>(HumanoidInterface_->mpcSettings(), HumanoidInterface_->sqpSettings(),
                                  HumanoidInterface_->getOptimalControlProblem(), HumanoidInterface_->getInitializer());
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(HumanoidInterface_->getPinocchioInterface(),
                                                                    HumanoidInterface_->getCentroidalModelInfo());

  const std::string robotName = "humanoid";
  

  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(controllerNh_, HumanoidInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, HumanoidInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(controllerNh_);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = controllerNh_->create_publisher<ocs2_msgs::msg::MpcObservation>(robotName + "_mpc_observation", 1);
}

void humanoidController::setupMrt() {
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&HumanoidInterface_->getRollout());
  mpcTimer_.reset();

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            HumanoidInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        RCLCPP_ERROR(controllerNh_->get_logger() , "[Ocs2 MPC thread] Error : %s" , e.what());
        //TODO: send the stop command to hardware interface
      }
    }
  });
  setThreadPriority(HumanoidInterface_->sqpSettings().threadPriority, mpcThread_);
}

void humanoidController::setupStateEstimate(const std::string& taskFile, bool verbose) {
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(HumanoidInterface_->getPinocchioInterface(),
                                                          HumanoidInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_ , controllerNh_);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

void humanoidCheaterController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/) {
  stateEstimate_ = std::make_shared<FromTopicStateEstimate>(HumanoidInterface_->getPinocchioInterface(),
                                                            HumanoidInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, controllerNh_);
}

}  // namespace humanoid_controller
