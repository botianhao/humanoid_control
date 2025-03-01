/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "humanoid_dummy/gait/GaitKeyboardPublisher.h"

#include <algorithm>

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/LoadData.h>


#include "humanoid_dummy/gait/ModeSequenceTemplateRos.h"




namespace ocs2 {
namespace humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaitKeyboardPublisher::GaitKeyboardPublisher(const rclcpp::Node::SharedPtr &node, const std::string& gaitFile, const std::string& robotName,
                                             bool verbose) {
  RCLCPP_INFO_STREAM(node->get_logger(),robotName + "_mpc_mode_schedule node is setting up ...");
  loadData::loadStdVector(gaitFile, "list", gaitList_, verbose);

  modeSequenceTemplatePublisher_ = node->create_publisher<ocs2_msgs::msg::ModeSchedule>(robotName + "_mpc_mode_schedule", 1);

  modeSequencePublisher_ = node->create_publisher<std_msgs::msg::String>(robotName + "_gait_mode_schedule", 1);

  gaitMap_.clear();
  for (const auto& gaitName : gaitList_) {
    gaitMap_.insert({gaitName, loadModeSequenceTemplate(gaitFile, gaitName, verbose)});
  }
  RCLCPP_INFO_STREAM(node->get_logger(),robotName + "_mpc_mode_schedule command node is ready.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitKeyboardPublisher::getKeyboardCommand() {
  const std::string commadMsg = "Enter the desired gait, for the list of available gait enter \"list\"";
  std::cout << commadMsg << ": ";

  auto shouldTerminate = []() { return !rclcpp::ok(); };
  const auto commandLine = stringToWords(getCommandLineString(shouldTerminate));

  if (commandLine.empty()) {
    return;
  }

  if (commandLine.size() > 1) {
    std::cout << "WARNING: The command should be a single word." << std::endl;
    return;
  }

  // lower case transform
  auto gaitCommand = commandLine.front();
  std::transform(gaitCommand.begin(), gaitCommand.end(), gaitCommand.begin(), ::tolower);

  if (gaitCommand == "list") {
    printGaitList(gaitList_);
    return;
  }

  try {
    std_msgs::msg::String gaitCommand_;//获取当前的步伐状态
    gaitCommand_.data = gaitCommand;


    ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(gaitCommand);
    modeSequenceTemplatePublisher_->publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
    modeSequencePublisher_->publish(gaitCommand_);//向外发送当前的踏步状态    
  } catch (const std::out_of_range& e) {
    std::cout << "Gait \"" << gaitCommand << "\" not found.\n";
    printGaitList(gaitList_);
  }
}

void GaitKeyboardPublisher::publishGaitCommandFromString(const std::string& gaitCommand) {
  try {
    ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(gaitCommand);
    modeSequenceTemplatePublisher_->publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
  } catch (const std::out_of_range& e) {
    std::cout << "Gait \"" << gaitCommand << "\" not found.\n";
    printGaitList(gaitList_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitKeyboardPublisher::printGaitList(const std::vector<std::string>& gaitList) const {
  std::cout << "List of available gaits:\n";
  size_t itr = 0;
  for (const auto& s : gaitList) {
    std::cout << "[" << itr++ << "]: " << s << "\n";
  }
  std::cout << std::endl;
}

}  // namespace humanoid
}  // end of namespace ocs2
