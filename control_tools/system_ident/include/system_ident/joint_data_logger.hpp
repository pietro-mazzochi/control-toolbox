// Copyright (c) 2024, ISI SIM
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef SYSTEM_IDENT__JOINT_DATA_LOGGER_HPP_
#define SYSTEM_IDENT__JOINT_DATA_LOGGER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <fstream>
#include <string>

class JointDataLogger : public rclcpp::Node
{
public:
    JointDataLogger();
    ~JointDataLogger();

private:
    void left_joint_effort_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void right_joint_effort_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void timer_callback();

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr left_wheel_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr right_wheel_cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time initial_time_;
    double timebase_ms_;
    int timer_frequency;

    std::ofstream output_file_;
    std::string output_file_name_;

    sensor_msgs::msg::JointState joint_data_;
};

#endif // SYSTEM_IDENT__JOINT_DATA_LOGGER_HPP_
