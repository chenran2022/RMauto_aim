// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Eigen>


// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/debug_send.hpp"
#include "auto_aim_interfaces/msg/debug_send_armors.hpp"
#include "auto_aim_interfaces/msg/detect_flag.hpp"
#include "auto_aim_interfaces/msg/serial_port.hpp"
// #include "decision_msgs/msg/serial_port.hpp"

namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  ~RMSerialDriver() override;

private:
  ReceivePacket receive_data;

  void getParams();

  void receiveData();

  void sendData(auto_aim_interfaces::msg::Target::SharedPtr msg);

  void reopenPort();

  void setParam(const rclcpp::Parameter & param, const std::string & param_name);

  void resetTracker();

  double dynamicCalcPitchOffset(Eigen::Vector3d & xyz);

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  // Param client to set detect_colr
  using ResultFuturePtr = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  bool initial_set_color_param_ = false;
  bool initial_set_mode_param_ = false;
  bool initial_set_num_param_ = false;     //不击打的数字
  uint8_t previous_receive_color_ = 0;
  uint8_t previous_aim_mode_ = 0;
  int previous_unrecognized_num_ = 0;

  rclcpp::AsyncParametersClient::SharedPtr detector_param_client_;
  ResultFuturePtr set_param_future_;

  double receive_yaw_ = 0;
  double shoot_speed = 27;

  // Service client to reset tracker
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;

  // Aimimg point receiving from serial port for visualization
  visualization_msgs::msg::Marker aiming_point_;

  // Broadcast tf from odom to gimbal_link
  double timestamp_offset_ = 0;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;

  // For debug usage
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  // auto_aim_interfaces::msg::DetectFlag send_detect_flag_msg;
  // rclcpp::Publisher<auto_aim_interfaces::msg::DetectFlag>::SharedPtr detect_flag_pub_;

  auto_aim_interfaces::msg::SerialPort send_serial_port_msg;
  rclcpp::Publisher<auto_aim_interfaces::msg::SerialPort>::SharedPtr serial_port_pub_;

  // For debug send data
  auto_aim_interfaces::msg::DebugSendArmors debug_send_armors_msg_;
  rclcpp::Publisher<auto_aim_interfaces::msg::DebugSendArmors>::SharedPtr debug_send_armors_pub_;

  std::thread receive_thread_;
  std::thread send_thread_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
