// Copyright (c) 2024, ISI SIM
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <iostream>
#include <vector>
#include <cassert>

class StepGeneratorNode : public rclcpp::Node
{
public:
    StepGeneratorNode() : Node("step_generator_node")
    {
        this->declare_parameter<bool>("is_float32", false);
        this->declare_parameter<double>("signal_frequency", 1.0);
        this->declare_parameter<double>("signal_amplitude", 1.0);
        this->declare_parameter<int>("signal_sequences", 0);
        this->declare_parameter<std::vector<double>>("left_step_vector", {1, -1});
        this->declare_parameter<std::vector<double>>("right_step_vector", {1, -1});
        this->declare_parameter("left_joint_cmd_topic", "/left_step_seq");
        this->declare_parameter("right_joint_cmd_topic", "/right_step_seq");

        this->get_parameter("is_float32", is_float32_);
        this->get_parameter("signal_amplitude", signal_amplitude_);
        this->get_parameter("signal_frequency", signal_frequency_);
        this->get_parameter("signal_sequences", signal_sequences_);
        this->get_parameter("left_step_vector", left_step_vector_);
        this->get_parameter("right_step_vector", right_step_vector_);
        this->get_parameter("left_joint_cmd_topic", left_joint_cmd_topic_);
        this->get_parameter("right_joint_cmd_topic", right_joint_cmd_topic_);

        RCLCPP_INFO_STREAM(this->get_logger(), "Signal amplitude: " << signal_amplitude_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Signal frequency: " << signal_frequency_);
        if (signal_sequences_ == 0)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Signal sequences: Inf");
        }
        else
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Signal sequences: " << signal_sequences_);
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Left joint step cmd topic: " << left_joint_cmd_topic_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Right joint step cmd topic: " << right_joint_cmd_topic_);

        if (!is_float32_)
        {
            left_joint_cmd_pub_64_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(left_joint_cmd_topic_, 1);
            right_joint_cmd_pub_64_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(right_joint_cmd_topic_, 1);
        }
        else
        {
            left_joint_cmd_pub_32_ = this->create_publisher<std_msgs::msg::Float32>(left_joint_cmd_topic_, 1);
            right_joint_cmd_pub_32_ = this->create_publisher<std_msgs::msg::Float32>(right_joint_cmd_topic_, 1);
        }

        rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback([this]()
                                                                                  {
            this->timer_->cancel();
            this->pre_shutdown_callback(); });

        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / signal_frequency_)),
                                         std::bind(&StepGeneratorNode::timer_callback, this));

        // Initialization of data
        assert(left_step_vector_.size() == right_step_vector_.size() && "Step vectors must have same length");

        seq_counter_ = 0;
        index_counter_ = 0;
        signal_len_ = left_step_vector_.size();
    }

private:
    void timer_callback()
    {
        double left_effort_ = 0;
        double right_effort_ = 0;

        bool signal_update_ = signal_sequences_ == 0 || seq_counter_ < signal_sequences_;

        if (signal_update_)
        {
            left_effort_ = left_step_vector_[index_counter_] * signal_amplitude_;
            right_effort_ = right_step_vector_[index_counter_] * signal_amplitude_;
        }

        if (!is_float32_)
        {
            std_msgs::msg::Float64MultiArray left_cmd_msg;
            std_msgs::msg::Float64MultiArray right_cmd_msg;

            left_cmd_msg.data.push_back(left_effort_);
            right_cmd_msg.data.push_back(right_effort_);

            left_joint_cmd_pub_64_->publish(left_cmd_msg);
            right_joint_cmd_pub_64_->publish(right_cmd_msg);
        }
        else
        {
            std_msgs::msg::Float32 left_cmd_msg;
            std_msgs::msg::Float32 right_cmd_msg;

            left_cmd_msg.data = left_effort_;
            right_cmd_msg.data = right_effort_;

            left_joint_cmd_pub_32_->publish(left_cmd_msg);
            right_joint_cmd_pub_32_->publish(right_cmd_msg);
        }

        RCLCPP_INFO_STREAM(this->get_logger(), "Effort signals: " << left_effort_ << " Nm -- " << right_effort_ << " Nm");

        if (signal_update_)
        {
            if (0 == ((1 + index_counter_) % signal_len_))
            {
                RCLCPP_INFO_STREAM(this->get_logger(), ++seq_counter_ << " seqs. finished!");
            }

            index_counter_ = (1 + index_counter_) % signal_len_;
        }
    }

    void pre_shutdown_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Pre-shutdown. Setting throttle commands to zero.");

        if (!is_float32_)
        {
            auto left_joint_message = std_msgs::msg::Float64MultiArray();
            auto right_joint_message = std_msgs::msg::Float64MultiArray();

            left_joint_message.data.resize(1);
            right_joint_message.data.resize(1);

            left_joint_message.data[0] = 0;
            right_joint_message.data[0] = 0;

            left_joint_cmd_pub_64_->publish(left_joint_message);
            right_joint_cmd_pub_64_->publish(right_joint_message);
        }
        else
        {
            auto left_joint_message = std_msgs::msg::Float32();
            auto right_joint_message = std_msgs::msg::Float32();

            left_joint_message.data = 0;
            right_joint_message.data = 0;

            left_joint_cmd_pub_32_->publish(left_joint_message);
            right_joint_cmd_pub_32_->publish(right_joint_message);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    bool is_float32_;
    int signal_len_;
    int seq_counter_;
    int index_counter_;
    int signal_sequences_;
    double signal_frequency_;
    double signal_amplitude_;
    std::vector<double> left_step_vector_, right_step_vector_;

    std::string left_joint_cmd_topic_;
    std::string right_joint_cmd_topic_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_joint_cmd_pub_32_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_joint_cmd_pub_32_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_joint_cmd_pub_64_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_joint_cmd_pub_64_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StepGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}
