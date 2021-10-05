// Copyright (c) 2018 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_ACTION_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/wait.hpp"

namespace nav2_behavior_tree {

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps
 * nav2_msgs::action::Wait
 */
class WaitAction : public BtActionNode<nav2_msgs::action::Wait> {
public:
  /**
   * @brief A constructor for nav2_behavior_tree::WaitAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  WaitAction(const std::string &xml_tag_name, const std::string &action_name,
             const BT::NodeConfiguration &conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() {
    return providedBasicPorts(
        {BT::InputPort<int>("wait_duration", 1, "Wait time")});
  }
};

/**
 * @brief A BT::SyncActionNode that returns SUCCESS when goal is
 * updated on the blackboard and RUNNING otherwise
 */
class WaitForNewGoal : public BT::ActionNodeBase {
public:
  /**
   * @brief A constructor for nav2_behavior_tree::WaitForNewGoal
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  WaitForNewGoal(const std::string &xml_tag_name,
                 const BT::NodeConfiguration &conf);

  WaitForNewGoal() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  void halt() override { setStatus(BT::NodeStatus::IDLE); }

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts() { return {}; }

private:
  geometry_msgs::msg::PoseStamped goal_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
};

} // namespace nav2_behavior_tree

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_ACTION_HPP_
