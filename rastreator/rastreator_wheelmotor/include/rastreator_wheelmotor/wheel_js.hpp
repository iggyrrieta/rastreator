#ifndef RASTREATOR_WHEELMOTOR_WHEEL_JS_HPP_
#define RASTREATOR_WHEELMOTOR_WHEEL_JS_HPP_


#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rastreator_wheelmotor/control_table.hpp"
#include "rastreator_wheelmotor/dynamixel_sdk_wrapper.hpp"

namespace rastreator
{
constexpr uint8_t JOINT_NUM = 2;

// ref) http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#goal-velocity104
constexpr double RPM_TO_MS = 0.229 * 0.0034557519189487725;

// 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
constexpr double TICK_TO_RAD = 0.001533981;

extern const ControlTable extern_control_table;
class Wheeljs
{
 public:
  explicit Wheeljs(
    std::shared_ptr<rclcpp::Node> &nh,
    const std::string topic_name,
    const std::string frame_id);
  virtual ~Wheeljs(){};

  void publish(
    const rclcpp::Time & now,
    std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper);

 private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;

  std::shared_ptr<rclcpp::Node> nh_;
  std::string topic_name_;
  std::string frame_id_;
  
};
} // rastreator
#endif // RASTREATOR_WHEELMOTOR_WHEEL_JS_HPP_