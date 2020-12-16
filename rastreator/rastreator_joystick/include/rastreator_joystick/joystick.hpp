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

#ifndef RASTREATOR_JOYSTICK_JOYSTICK_HPP_
#define RASTREATOR_JOYSTICK_JOYSTICK_HPP_

#include <chrono>
#include <memory>
#include <string>

//ROS2
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include "geometry_msgs/msg/twist.hpp"

namespace rastreator
{ 
class Joystick : public rclcpp::Node
{
 public:
  explicit Joystick();

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr  msg);

  typedef struct
  {
    float axes_x;
    float axes_y;
    float button_a;
    float button_b;
    float button_x;
    float button_y;
    float linear_gain;
    float angular_gain;
    float velocity;
  } commands;

  typedef struct
  {
    float linear_gain;
    float angular_gain;
    float velocity;
  } speed;

  commands commands_;
  speed speed_;

 private:
  /*==================== 
    Publishers,
    Subscribers and
    Timers
  ====================*/
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  /*==================== 
    Variables
  ====================*/
  float velocity = 1.0;


};
} // rastreator
#endif // RASTREATOR_NODE_SENSORS_JOYSTICK_HPP_