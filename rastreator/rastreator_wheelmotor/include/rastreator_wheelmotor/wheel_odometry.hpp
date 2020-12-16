/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Darby Lim */
/* Adapted by: Iñaki Lorente */

#ifndef RASTREATOR_WHEELMOTOR_WHEEL_ODOMETRY_HPP_
#define RASTREATOR_WHEELMOTOR_WHEEL_ODOMETRY_HPP_

#include <array>
#include <chrono>
#include <memory>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>


namespace rastreator
{
class Odometry
{
 public:
  explicit Odometry(
    std::shared_ptr<rclcpp::Node> &nh,
    const double wheels_separation,
    const double wheels_radius,
    const std::string frame_id,
    const std::string child_frame_id,
    const bool publish_tf);
  virtual ~Odometry(){};

 private:
  bool calculate_odometry(const rclcpp::Duration & duration);

  void update_joint_state(const std::shared_ptr<sensor_msgs::msg::JointState const> & joint_state);

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg);

  void publish(const rclcpp::Time & now);

  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::JointState>> msg_ftr_joint_state_sub_;


  double wheels_separation_;
  double wheels_radius_;
  std::string frame_id_of_odometry_;
  std::string child_frame_id_of_odometry_;
  bool publish_tf_;

  std::array<double, 2> diff_joint_positions_;

  std::array<double,3> robot_pose_;
  std::array<double,3> robot_vel_;
};
} // rastreator
#endif //RASTREATOR_WHEELMOTOR_WHEEL_ODOMETRY_HPP_
