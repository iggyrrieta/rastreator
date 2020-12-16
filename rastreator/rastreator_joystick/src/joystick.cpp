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

#include "rastreator_joystick/joystick.hpp"

using namespace rastreator;
using std::placeholders::_1;

Joystick::Joystick() :
     Node("Joystick_playstation")
{

  RCLCPP_INFO(this->get_logger(), "Joystick publisher ON");

  this->declare_parameter("commands.axes_x");
  this->declare_parameter("commands.axes_y");
  this->declare_parameter("commands.button_a");
  this->declare_parameter("commands.button_b");
  this->declare_parameter("commands.button_x");
  this->declare_parameter("commands.button_y");

  this->get_parameter_or<float>("commands.axes_x", commands_.axes_x, 1);
  this->get_parameter_or<float>("commands.axes_y", commands_.axes_y, 0);
  this->get_parameter_or<float>("commands.button_a", commands_.button_a, 2);
  this->get_parameter_or<float>("commands.button_b", commands_.button_b, 1);
  this->get_parameter_or<float>("commands.button_x", commands_.button_x, 3);
  this->get_parameter_or<float>("commands.button_y", commands_.button_y, 0);
  this->get_parameter_or<float>("speed.linear_gain", speed_.linear_gain, 10.0);
  this->get_parameter_or<float>("speed.angular_gain", speed_.angular_gain, 5.0);
  this->get_parameter_or<float>("speed.velocity", speed_.velocity, 1.0);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy",
            qos,
            std::bind(&Joystick::joy_callback, this, std::placeholders::_1));
}

void Joystick::joy_callback(const sensor_msgs::msg::Joy::SharedPtr  msg)
{
  geometry_msgs::msg::Twist twist;

  
  // Increment velocity
  if (msg->buttons[commands_.button_y])
  {
    velocity += 1.0;
  }
  // Reset velocity
  if (msg->buttons[commands_.button_a])
  {
    velocity = speed_.velocity;
  }

  twist.linear.x = speed_.linear_gain*velocity*msg->axes[commands_.axes_x];
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;

  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = speed_.angular_gain*velocity*msg->axes[commands_.axes_y];

  cmd_vel_pub_->publish(twist);
 
}

/*****************************************************************************
** Main
*****************************************************************************/
int main()
{
  rclcpp::init(0, nullptr);
  rclcpp::spin(std::make_shared<rastreator::Joystick>());
  rclcpp::shutdown();

  return 0;
}


