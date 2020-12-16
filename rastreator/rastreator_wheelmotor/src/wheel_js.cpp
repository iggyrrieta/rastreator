// Copyright 2020
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
//
// Authors: Iñaki Lorente

#include <array>

#include "rastreator_wheelmotor/wheel_js.hpp"

using namespace rastreator;

Wheeljs::Wheeljs(
  std::shared_ptr<rclcpp::Node> &nh,
  const std::string topic_name,
  const std::string frame_id)
: nh_(nh),
  topic_name_(topic_name),
  frame_id_(frame_id)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  js_pub_ = nh_->create_publisher<sensor_msgs::msg::JointState>(topic_name_, qos);

  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create joint state publisher");
}

void Wheeljs::publish(
  const rclcpp::Time & now,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
{
  auto msg = std::make_unique<sensor_msgs::msg::JointState>();

  static std::array<uint32_t, JOINT_NUM> last_diff_position, last_position;

  std::array<uint32_t, JOINT_NUM> position =
    {dxl_sdk_wrapper->get_data(
      1,
      extern_control_table.present_position.addr,
      extern_control_table.present_position.length),
    dxl_sdk_wrapper->get_data(
      2,
      extern_control_table.present_position.addr,
      extern_control_table.present_position.length)};

  std::array<uint32_t, JOINT_NUM> velocity =
    {dxl_sdk_wrapper->get_data(
      1,
      extern_control_table.present_velocity.addr,
      extern_control_table.present_velocity.length),
    dxl_sdk_wrapper->get_data(
      2,
      extern_control_table.present_velocity.addr,
      extern_control_table.present_velocity.length)};

  msg->header.frame_id = this->frame_id_;
  msg->header.stamp = now;

  msg->name.push_back("wheel_left_joint");
  msg->name.push_back("wheel_right_joint");

  msg->position.push_back(TICK_TO_RAD * last_diff_position[0]);
  msg->position.push_back(TICK_TO_RAD * last_diff_position[1]);

  msg->velocity.push_back(RPM_TO_MS * velocity[0]);
  msg->velocity.push_back(RPM_TO_MS * velocity[1]);


  last_diff_position[0] += (position[0] - last_position[0]);
  last_diff_position[1] += (position[1] - last_position[1]);

  last_position = position;

  js_pub_->publish(std::move(msg));
}