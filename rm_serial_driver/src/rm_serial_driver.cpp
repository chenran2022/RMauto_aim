// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{
int data_length = 10;
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  // TF broadcaster
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create Publisher
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);

  // Detect parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

  // Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
      
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  // Create Subscription
  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));
  
}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}


void RMSerialDriver::receiveData()
{

  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));
  // data.reserve(sizeof(uint8_t) * 50);
  bzero(data.data(), sizeof(uint8_t) * 10);

  while (rclcpp::ok()) {
    try {
      
      bzero(data.data(), sizeof(uint8_t) * 10);
      serial_driver_->port()->receive(header);
      if (header[0] == 0xAA) {
     
        data.resize(data_length-1);
        int len = serial_driver_->port()->receive(data);
        if(!serial_driver_->port()->is_open())
        {
            serial_driver_->port()->open();

        }
      
        if(len < data_length-1 ){
          
          std::vector<uint8_t> data_(data_length - 1 - len);
          // auto first_time = this->now();

           serial_driver_->port()->receive(data_);

          //  auto final_time = this->now();
          // auto latency = (final_time - first_time).seconds() * 1000;
          // std::cout<<"   "<<latency<<std::endl;
          data.resize(len);
          data.insert(data.end(),data_.begin(),data_.end());    
        }
      
        data.insert(data.begin(), header[0]);

      //first_time = final_time;
        if(data[9]!=0xAF)
        {
          std::cout<<"qqqqqqqqqqqqqq"<<std::endl;
          continue;
        }
        // printf("1   %d\n",data[0]);
        // printf("2   %d\n",data[1]);
        // printf("3   %d\n",data[2]);
        // printf("4   %d\n",data[3]);
        // printf("5   %d\n",data[4]);
        // printf("6   %d\n",data[5]);
        // printf("7   %d\n",data[6]);
        // printf("8   %d\n",data[7]);
        // printf("9   %d\n",data[8]);
        // printf("0   %d\n",data[9]);

        ReceivePacket packet = fromVector(data);
        std::cout << "receive yaw: " << packet.yaw_angle << "  receive pitch: " << packet.pitch_angle << std::endl;

        // bool crc_ok =
        //   crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        // if (crc_ok) {
        if (!initial_set_color_param_ || packet.detect_color != previous_receive_color_) {
          setParam(rclcpp::Parameter("detect_color", packet.detect_color), "detect_color");
          previous_receive_color_ = packet.detect_color;
        }

        // RCLCPP_INFO(get_logger(), "receive aim mode: %d", packet.aim_mode);
        if (!initial_set_mode_param_ || packet.aim_mode != previous_aim_mode_) {
          setParam(rclcpp::Parameter("aim_mode", packet.aim_mode), "aim_mode");
          // setParam(rclcpp::Parameter("aim_mode", 3), "aim_mode");
          previous_aim_mode_ = packet.aim_mode;
        }

        //    不击打的数字
        // RCLCPP_INFO(get_logger(), "receive unrecognized_num: %d", packet.unrecognized_num);
        if (!initial_set_num_param_ || packet.unrecognized_num != previous_unrecognized_num_) {
          // setParam(rclcpp::Parameter("unrecognized_num", packet.unrecognized_num), "unrecognized_num");
          setParam(rclcpp::Parameter("unrecognized_num", packet.unrecognized_num), "unrecognized_num");
          previous_unrecognized_num_  = packet.unrecognized_num;
        }


        // if (packet.reset_tracker) {
        //   resetTracker();
        // }
        shoot_speed = packet.bullet_speed;
        geometry_msgs::msg::TransformStamped t;
        timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
        t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
        t.header.frame_id = "odom";
        t.child_frame_id = "gimbal_link";
        tf2::Quaternion q;
        q.setRPY(0, packet.pitch_angle / 180 * M_PI, packet.yaw_angle / 180 * M_PI);
        t.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(t);
        receive_data = packet;
        // } else {
        //   RCLCPP_ERROR(get_logger(), "CRC error!");
        // }
      } 
      else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %04X", header[0]);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

/// @brief
/// @param target_msg
void RMSerialDriver::sendData(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  const static std::map<std::string, uint8_t> id_uint8_map{
    {"", 0},  {"negative", 0}, {"1", 1}, {"1", 1},     {"2", 2},
    {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};
  std::string armor_id = msg->id;
  uint8_t id = id_uint8_map.at(msg->id);
  aiming_point_.header.frame_id = "odom";
  aiming_point_.ns = "aiming_point";
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  aiming_point_.color.r = 1.0;
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0;
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

  try {
    SendPacket packet;

    struct tar_pos
    {
      double x;
      double y;
      double z;
      double yaw;
    };

    double yaw = msg->yaw;
    double r1 = msg->radius_1, r2 = msg->radius_2;
    double xc = msg->position.x, yc = msg->position.y, za = msg->position.z;
    double dz = msg->dz;
    double vx = msg->velocity.x, vy = msg->velocity.y;
    bool is_current_pair = true;
    size_t a_n = msg->armors_num;
    struct tar_pos tar_position[a_n];
    geometry_msgs::msg::Point p_w;
    auto_aim_interfaces::msg::DebugSend target_data;

    double armor_yaw[4];
    double r = 0;
    for (size_t i = 0; i < a_n; i++) {
      double tmp_yaw = yaw + i * (2 * M_PI / a_n);
      if (tmp_yaw < 0) {
        tmp_yaw = 2 * M_PI + tmp_yaw;
      }
      if (a_n == 4) {
        r = is_current_pair ? r1 : r2;
        tar_position[i].z = za + (is_current_pair ? 0 : dz);
        is_current_pair = !is_current_pair;
      } else {
        r = r1;
        tar_position[i].z = za;
      }
      tar_position[i].x = xc - r * cos(tmp_yaw);
      tar_position[i].y = yc - r * sin(tmp_yaw);
      armor_yaw[i] = tmp_yaw;
      target_data.point.x = tar_position[i].x;
      target_data.point.y = tar_position[i].y;
      target_data.point.z = tar_position[i].z;
      target_data.yaw = tmp_yaw;
      target_data.id = i;
      target_data.angle = tmp_yaw * 180 / M_PI;
      this->debug_send_armors_msg_.data.emplace_back(target_data);
    }

    // Calculate center angle
    double center_yaw = 0;
    center_yaw = atan2(yc, xc);
    if (center_yaw > 2 * M_PI)
      center_yaw -= 2 * M_PI;
    else if (center_yaw < 0)
      center_yaw += 2 * M_PI;
    this->debug_send_armors_msg_.center_angle = center_yaw * 180 / M_PI;
    // RCLCPP_INFO_STREAM(get_logger(), "center_yaw: " + std::to_string(center_yaw* 180 / M_PI));
    //
    double bullet_speed = shoot_speed;
    double dis = sqrt(
      msg->position.x * msg->position.x + msg->position.y * msg->position.y +
      msg->position.z * msg->position.z);
    double time_delay = dis / bullet_speed + 0.005;
    int idx = 0;
    double pre_armor_yaw[4] = {0};
    for (size_t i = 0; i < a_n; i++) {
      pre_armor_yaw[i] = time_delay * msg->v_yaw + armor_yaw[i];
      if (pre_armor_yaw[i] > 4 * M_PI) {
        pre_armor_yaw[i] -= 4 * M_PI;
      }
      if (pre_armor_yaw[i] > 2 * M_PI) {
        pre_armor_yaw[i] -= 2 * M_PI;
      }
    }
    // std::cout<<"0:::"<<pre_armor_yaw[0]<<std::endl;
    // std::cout<<"1:::"<<pre_armor_yaw[1]<<std::endl;
    // std::cout<<"2:::"<<pre_armor_yaw[2]<<std::endl;
    // std::cout<<"3:::"<<pre_armor_yaw[3]<<std::endl;
    // std::cout<<"center_yaw"<<center_yaw<<std::endl;
    double yaw_diff_min = fabs(center_yaw - pre_armor_yaw[0]);
    if (yaw_diff_min > M_PI) {
      if (center_yaw > 0 && center_yaw < M_PI)
        yaw_diff_min = 2 * M_PI - pre_armor_yaw[0] + center_yaw;
      else
        yaw_diff_min = 2 * M_PI - center_yaw + pre_armor_yaw[0];
    }
    // std::cout<<"yaw_diff1         :"<<yaw_diff_min<<std::endl;
    // std::cout<<"                          "<<std::endl;
    for (size_t i = 1; i < a_n; i++) {
      double temp_yaw_diff = fabs(center_yaw - pre_armor_yaw[i]);
      if (temp_yaw_diff > M_PI) {
        if (center_yaw > 0 && center_yaw < M_PI)
          temp_yaw_diff = 2 * M_PI - pre_armor_yaw[i] + center_yaw;
        else
          temp_yaw_diff = 2 * M_PI - center_yaw + pre_armor_yaw[i];
      }
      // std::cout<<"yaw_diff        :"<<temp_yaw_diff<<std::endl;

      if (temp_yaw_diff < yaw_diff_min) {
        yaw_diff_min = temp_yaw_diff;
        idx = i;
      }
    }
    // tar_position[i].x = xc - r * cos(tmp_yaw);
    //   tar_position[i].y = yc - r * sin(tmp_yaw);

    p_w.x = xc - r * cos(pre_armor_yaw[idx]) + time_delay * vx;
    p_w.y = yc - r * sin(pre_armor_yaw[idx]) + time_delay * vy;
    p_w.z = tar_position[idx].z;
    Eigen::Vector3d xyz = {p_w.x, p_w.z, p_w.y};

    double pitch_offset = dynamicCalcPitchOffset(xyz);
    
    ///
    //调参 误差
    //下+ 左+？
    packet.yaw_angle = atan2(p_w.y, p_w.x) / M_PI * 180 + 1;
    if (packet.yaw_angle > 360)
      packet.yaw_angle -= 360;
    else if (packet.yaw_angle < 0)
      packet.yaw_angle += 360;
    packet.pitch_angle =
      -atan2(p_w.z, sqrt(p_w.x * p_w.x + p_w.y * p_w.y)) / M_PI * 180 + 90 - pitch_offset;
    ///
    
    
    packet.distance = sqrt(p_w.x * p_w.x + p_w.y * p_w.y + p_w.z * p_w.z);
    double a_distance = sqrt(p_w.x * p_w.x + p_w.y * p_w.y + p_w.z * p_w.z);
    
    if (msg->tracking > 0)
      packet.detect_flag = id;
    else
      packet.detect_flag = 0;


///
    // detect_flag_pub_=
    //   this->create_publisher<auto_aim_interfaces::msg::DetectFlag>("/detect_flag", 10);
    //   if(packet.detect_flag>0)
    //   {
    //     send_detect_flag_msg.detect_flag=1;
    //     detect_flag_pub_->publish(send_detect_flag_msg);
    //   }
    //   else
    //   {
    //     send_detect_flag_msg.detect_flag=0;
    //     detect_flag_pub_->publish(send_detect_flag_msg);
    //   }

      serial_port_pub_=
      this->create_publisher<auto_aim_interfaces::msg::SerialPort>("/detect_flag", 10);
      if(packet.detect_flag>0)
      {
        send_serial_port_msg.detect_flag=1;
        serial_port_pub_->publish(send_serial_port_msg);
      }
      else
      {
        send_serial_port_msg.detect_flag=0;
        serial_port_pub_->publish(send_serial_port_msg);
      }
      
      
///


    //packet.shoot_flag = 1;
    // std::cout<<"      aaa       "<<packet.detect_flag<<std::endl;
    double yaw_error = abs(receive_yaw_-packet.yaw_angle);
    if (yaw_error > 180) {
      if (receive_yaw_ > 0 && packet.yaw_angle > M_PI)
        yaw_error = 360 - packet.yaw_angle + receive_yaw_;
      else
        yaw_error = 360 - receive_yaw_ + packet.yaw_angle;
    }
    if(packet.detect_flag !=0){
      if(msg->armors_num == 2){
        if(yaw_error<(atan(0.17/a_distance))/M_PI*180 ){
          packet.shoot_flag = 1;
        }
          
      }else{
        if(id == 1){
          if(yaw_error<(atan(0.20/a_distance))/M_PI*180){
            packet.shoot_flag = 1;  
          }
            
        }else{
          if(yaw_error<(atan(0.135/a_distance))/M_PI*180 ){
            packet.shoot_flag = 1;
            // std::cout<<"angle"<<atan(0.15/a_distance)/M_PI*180<<std::endl;
          }

        }
      }

      // std::cout<<"receive_yaw_"<<receive_yaw_<<std::endl;
      // std::cout<<"packet.yaw_angle"<<packet.yaw_angle<<std::endl;
            std::cout<<"angle"<<yaw_error<<std::endl;
    }
    // std::cout<<packet.detect_flag<<std::endl;
    if (!packet.detect_flag) {
      packet.yaw_angle = packet.pitch_angle = 0;
      packet.shoot_flag = 0;
    }
    std::cout<<"angle"<<packet.pitch_angle<<std::endl;
    std::vector<uint8_t> data = toVector(packet);
    serial_driver_->port()->send(data);

    // Publish aiming point
    if (marker_pub_->get_subscription_count() > 0) {
      aiming_point_.header.stamp = msg->header.stamp;
      aiming_point_.pose.position.x = p_w.x;
      aiming_point_.pose.position.y = p_w.y;
      aiming_point_.pose.position.z = p_w.z;
      marker_pub_->publish(aiming_point_);
    }
    //rm -r build
    //
    //source /opt/ros/humble/setup.bash 
    //colcon build --symlink-install
    // Publish debug info
    debug_send_armors_pub_ =
      this->create_publisher<auto_aim_interfaces::msg::DebugSendArmors>("/debug_send_armors", 10);
    debug_send_armors_pub_->publish(this->debug_send_armors_msg_);

 
    std_msgs::msg::Float64 latency;
    // std_msgs::msg::Float64 dec;
   // latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    latency.data = (float)packet.shoot_flag;
    // latency.data = (float)packet.detect_flag;
    latency_pub_->publish(latency);

    this->debug_send_armors_msg_.data.clear();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}


void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param, const std::string & name)
{
  if (!detector_param_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    if (name == "detect_color") {
      RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
      set_param_future_ = detector_param_client_->set_parameters(
        {param}, [this, param](const ResultFuturePtr & results) {
          for (const auto & result : results.get()) {
            if (!result.successful) {
              RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
              return;
            }
          }
          RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
          initial_set_color_param_ = true;
        });
    } else if (name == "aim_mode") {
      RCLCPP_INFO(get_logger(), "Setting aim_mode to %ld...", param.as_int());
      set_param_future_ = detector_param_client_->set_parameters(
        {param}, [this, param](const ResultFuturePtr & results) {
          for (const auto & result : results.get()) {
            if (!result.successful) {
              RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
              return;
            }
          }
          RCLCPP_INFO(get_logger(), "Successfully set aim_mode to %ld!", param.as_int());
          initial_set_mode_param_ = true;
        });
    } else if (name == "unrecognized_num") {
      RCLCPP_INFO(get_logger(), "Setting unrecognized_num to %ld...", param.as_int());
      set_param_future_ = detector_param_client_->set_parameters(
        {param}, [this, param](const ResultFuturePtr & results) {
          for (const auto & result : results.get()) {
            if (!result.successful) {
              RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
              return;
            }
          }
          RCLCPP_INFO(get_logger(), "Successfully set unrecognized_num  to %ld!", param.as_int());
          initial_set_num_param_ = true;
        });
    }else {
      RCLCPP_ERROR(get_logger(), "Unknown parameter name: %s", name.c_str());
    }
  }
}

void RMSerialDriver::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

double RMSerialDriver::dynamicCalcPitchOffset(Eigen::Vector3d & xyz)
{
  int max_iter = 10;
  float stop_error = 0.001;
  int R_K_iter = 50;
  // double bullet_speed_ = 28;
  // double bullet_speed = 16;            //TODO:弹速可变
  // const double k = 0.01903;
  const double k = 0.02903;  //25°C,1atm,小弹丸
  // const double k = 0.000556;                //25°C,1atm,大弹丸
  // const double k = 0.000530;                //25°C,1atm,发光大弹丸
  const double g = 9.788;

  auto bullet_speed = shoot_speed;
  //TODO:根据陀螺仪安装位置调整距离求解方式
  //降维，坐标系Y轴以垂直向上为正方向
  auto dist_vertical = xyz[1] ;
  auto vertical_tmp = dist_vertical;
  // auto dist_horizonal = sqrt(xyz.squaredNorm() - dist_vertical * dist_vertical);
  // dist_horizonal = xyz[2];
  // dist_horizonal=6;
  // auto dist_vertical = xyz[2];
  auto dist_horizonal = sqrt(xyz.squaredNorm() - dist_vertical * dist_vertical);
  // cout<<"dist.horizonal          "<<dist_horizonal<<endl;
  // cout<<"speed:::::::::::"<<bullet_speed<<endl;
  auto pitch = atan(dist_vertical / dist_horizonal) * 180 / M_PI;
  auto pitch_new = pitch;
  // auto pitch_offset = 0.0;

  //开始使用龙格库塔法求解弹道补偿
  for (int i = 0; i < max_iter; i++) {
    //TODO:可以考虑将迭代起点改为世界坐标系下的枪口位置
    //初始化
    auto x = 0.0;
    auto y = 0.0;
    auto p = tan(pitch_new / 180 * M_PI);
    auto v = bullet_speed;
    auto u = v / sqrt(1 + pow(p, 2));
    auto delta_x = dist_horizonal / R_K_iter;
    for (int j = 0; j < R_K_iter; j++) {
      auto k1_u = -k * u * sqrt(1 + pow(p, 2));
      auto k1_p = -g / pow(u, 2);
      auto k1_u_sum = u + k1_u * (delta_x / 2);
      auto k1_p_sum = p + k1_p * (delta_x / 2);

      auto k2_u = -k * k1_u_sum * sqrt(1 + pow(k1_p_sum, 2));
      auto k2_p = -g / pow(k1_u_sum, 2);
      auto k2_u_sum = u + k2_u * (delta_x / 2);
      auto k2_p_sum = p + k2_p * (delta_x / 2);

      auto k3_u = -k * k2_u_sum * sqrt(1 + pow(k2_p_sum, 2));
      auto k3_p = -g / pow(k2_u_sum, 2);
      auto k3_u_sum = u + k3_u * (delta_x / 2);
      auto k3_p_sum = p + k3_p * (delta_x / 2);

      auto k4_u = -k * k3_u_sum * sqrt(1 + pow(k3_p_sum, 2));
      auto k4_p = -g / pow(k3_u_sum, 2);

      u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
      p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);

      x += delta_x;
      y += p * delta_x;
    }
    //评估迭代结果,若小于迭代精度需求则停止迭代
    auto error = dist_vertical - y;
    if (abs(error) <= stop_error) {
      break;
    } else {
      vertical_tmp += error;
      // xyz_tmp[1] -= error;
      pitch_new = atan(vertical_tmp / dist_horizonal) * 180 / M_PI;
    }
  }
  return pitch_new - pitch;
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
