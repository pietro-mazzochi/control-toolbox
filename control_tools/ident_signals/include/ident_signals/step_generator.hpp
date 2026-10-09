// Copyright (c) 2024, ISI SIM
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef IDENT_SIGNALS__STEP_GENERATOR_NODE_HPP_
#define IDENT_SIGNALS__STEP_GENERATOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <vector>
#include <string>
#include <memory>
#include <chrono>

class StepGeneratorNode : public rclcpp::Node
{
public:
    // Constructor
    StepGeneratorNode();

private:
    // Control loop that is triggered periodically by the timer
    void control_loop();

    // Member variables for parameters
    double sample_frequency_;
    int take_;
    std::string date_;

    // Motor control variables
    int Amplitude;
    std::vector<double> vecl, vecr;
    int step_index_value;
    int step_index_size;
    int cycles_of_pulses_count;
    int NUM_OF_CYCLES_FOR_PULSES;

    // ROS2 components
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_left_motor_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_right_motor_;
};

#endif // IDENT_SIGNALS__STEP_GENERATOR_NODE_HPP_
