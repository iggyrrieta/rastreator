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

#include "rastreator_wheelmotor/wheelmotor.hpp"

using namespace rastreator;
using std::placeholders::_1;

Wheelmotor::Wheelmotor() :
     Node("Wheelmotor")
{
  RCLCPP_INFO(this->get_logger(), "Wheel motors ON");

  nh_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  // Timer every 50ms to update joint states (wheel_js)
  // Odometry (wheel_odom) is updated as being subscribed to wheel_js
  publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                     std::bind(&Wheelmotor::update_wheel_js, this));

  // Subscriber to cmd_vel, every new cmd_vel the cmd_vel_callback
  // will send the command to the motors
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
      qos, 
      std::bind(&Wheelmotor::cmd_vel_callback, this, _1));

  add_motors();
  add_js();
  add_odom();
}

void Wheelmotor::add_motors()
{
  RCLCPP_INFO(this->get_logger(), "Add Motors");

  DynamixelSDKWrapper::Device config_ = {"/dev/ttyUSB0", 
                                        1, 
                                        2,
                                        57600, 
                                        2.0f
                                        };  

  this->declare_parameter("motor.usb_port");
  this->declare_parameter("motor.id_right");
  this->declare_parameter("motor.id_left");
  this->declare_parameter("motor.baud_rate");
  this->declare_parameter("motor.protocol_version");

  this->get_parameter_or<std::string>("motor.dev", config_.usb_port, "/dev/ttyUSB0");
  this->get_parameter_or<uint8_t>("motor.id_right", config_.id_right, 1);
  this->get_parameter_or<uint8_t>("motor.id_left", config_.id_left, 2);
  this->get_parameter_or<uint32_t>("motor.baud_rate", config_.baud_rate, 57600);
  this->get_parameter_or<float>("motor.protocol_version", config_.protocol_version, 2.0);

  dxl_sdk_wrapper_ = std::make_shared<DynamixelSDKWrapper>(config_);

  //Enable Torque
  dxl_sdk_wrapper_->write_single_register_sync(
    extern_control_table.motor_torque_enable.addr,
    extern_control_table.motor_torque_enable.length
    );

  dxl_sdk_wrapper_->create_storage(
    extern_control_table.drive_mode.addr,
    (extern_control_table.max_velocity_value.addr-extern_control_table.drive_mode.addr) +
     extern_control_table.max_velocity_value.length
    );

}

void Wheelmotor::add_js()
{
  RCLCPP_INFO(this->get_logger(), "Add Wheels joint state");

  this->declare_parameter("joint_state.topic_name");
  this->declare_parameter("joint_state.frame_id");

  this->get_parameter_or<std::string>("joint_state.topic_name", js_.topic_name, "joint_states");
  this->get_parameter_or<std::string>("joint_state.frame_id", js_.frame_id, "base_link");

  wheel_js_ = std::make_unique<Wheeljs>(
    nh_,
    js_.topic_name,
    js_.frame_id);
}

void Wheelmotor::add_odom()
{
  RCLCPP_INFO(this->get_logger(), "Add Wheels joint state");

  this->declare_parameter("odometry.wheel_separation");
  this->declare_parameter("odometry.wheel_radius");
  this->declare_parameter("odometry.frame_id");
  this->declare_parameter("odometry.child_frame_id");
  this->declare_parameter("odometry.publish_tf");

  this->get_parameter_or<double>("odometry.wheel_separation", odom_.wheel_separation, 0.160);
  this->get_parameter_or<double>("odometry.wheel_radius", odom_.wheel_radius, 0.033);
  this->get_parameter_or<std::string>("odometry.frame_id", odom_.frame_id, "odom");
  this->get_parameter_or<std::string>("odometry.child_frame_id", odom_.child_frame_id, "base_footprint");
  this->get_parameter_or<bool>("odometry.publish_tf", odom_.publish_tf, true);

  wheel_odom_ = std::make_unique<Odometry>(
    nh_,
    odom_.wheel_separation,
    odom_.wheel_radius,
    odom_.frame_id,
    odom_.child_frame_id,
    odom_.publish_tf);
}

void Wheelmotor::update_wheel_js()
{
  rclcpp::Time now = this->now();
  wheel_js_->publish(now, dxl_sdk_wrapper_);
}

void Wheelmotor::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  int32_t sp_vel_linear = static_cast<int32_t>(msg->linear.x);
  int32_t sp_vel_angular = static_cast<int32_t>(msg->angular.z);
          
  dxl_sdk_wrapper_->update_storage_velocity(
    extern_control_table.goal_velocity.addr,
    extern_control_table.goal_velocity.length,
    sp_vel_linear,
    sp_vel_angular);
}

/*****************************************************************************
** Main
*****************************************************************************/
int main()
{
  rclcpp::init(0, nullptr);
  rclcpp::spin(std::make_shared<rastreator::Wheelmotor>());
  rclcpp::shutdown();

  return 0;
}


