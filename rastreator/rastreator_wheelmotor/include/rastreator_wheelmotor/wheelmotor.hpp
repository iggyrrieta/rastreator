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

#ifndef RASTREATOR_WHEELMOTOR_WHEELMOTOR_HPP_
#define RASTREATOR_WHEELMOTOR_WHEELMOTOR_HPP_

#include <chrono>
#include <memory>
#include <string>

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rastreator_wheelmotor/control_table.hpp"
#include "rastreator_wheelmotor/wheel_js.hpp"
#include "rastreator_wheelmotor/wheel_odometry.hpp"
#include "rastreator_wheelmotor/dynamixel_sdk_wrapper.hpp"

namespace rastreator
{ 
extern const ControlTable extern_control_table;
class Wheelmotor : public rclcpp::Node
{
 public:
  explicit Wheelmotor();

  typedef struct
  {
    std::string usb_port;
    uint8_t id_right;
    uint8_t id_left;
    uint32_t baud_rate;
    float protocol_version;
  } Motors;

  typedef struct
  {
    std::string topic_name;
    std::string frame_id;
  } JS;

  typedef struct
  {
    double wheel_separation;
    double wheel_radius;
    std::string frame_id;
    std::string child_frame_id;
    bool publish_tf;
  } Odom;


  Motors config_;
  JS js_;
  Odom odom_;

 private:
  /*==================== 
    Publishers,
    Subscribers and
    Timers
  ====================*/
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

 /*==================== 
    Functions
  ====================*/
  void add_motors();
  void add_js();
  void add_odom();
  void update_wheel_js();
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /*==================== 
    Others
  ====================*/
  std::shared_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper_;
  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<Wheeljs> wheel_js_;
  std::unique_ptr<Odometry> wheel_odom_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

};
} // rastreator
#endif // RASTREATOR_WHEELMOTOR_WHEELMOTOR_HPP_