// Copyright (c) 2024, ISI SIM
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <random>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "ident_signals/prbs_generator.hpp"

class PRBSGeneratorNode : public rclcpp::Node
{
public:
    PRBSGeneratorNode() : Node("prbs_generator_node")
    {
        this->declare_parameter<bool>("is_float32", false);
        this->declare_parameter<int>("num_of_bits", 2);
        this->declare_parameter<double>("signal_frequency", 10.0);
        this->declare_parameter<double>("signal_amplitude", 1.0);
        this->declare_parameter<double>("offset", 0.0); // Declare the offset parameter
        this->declare_parameter<std::string>("left_joint_cmd_topic", "/left_prbs_seq");
        this->declare_parameter<std::string>("right_joint_cmd_topic", "/right_prbs_seq");

        this->get_parameter("is_float32", is_float32_);
        this->get_parameter("num_of_bits", num_of_bits_);
        this->get_parameter("signal_frequency", signal_frequency_);
        this->get_parameter("signal_amplitude", signal_amplitude_);
        this->get_parameter("offset", offset_); // Retrieve the offset parameter
        this->get_parameter("left_joint_cmd_topic", left_joint_cmd_topic_);
        this->get_parameter("right_joint_cmd_topic", right_joint_cmd_topic_);

        std::random_device rd;
        gen = std::mt19937(rd());
        std::uniform_int_distribution<> dis(1, 0xFFFFFF);

        left_joint_prbs_seed_ = dis(gen);
        right_joint_prbs_seed_ = dis(gen);

        // Garantir que os seeds sejam diferentes
        while (left_joint_prbs_seed_ == right_joint_prbs_seed_)
        {
            right_joint_prbs_seed_ = dis(gen);
        }

        RCLCPP_INFO_STREAM(this->get_logger(), "Num of bits: " << num_of_bits_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Signal frequency: " << signal_frequency_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Signal amplitude: " << signal_amplitude_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Offset: " << offset_); // Log the offset
        RCLCPP_INFO_STREAM(this->get_logger(), "Left joint PRBS seed: " << left_joint_prbs_seed_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Right joint PRBS seed: " << right_joint_prbs_seed_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Left joint PRBS cmd topic: " << left_joint_cmd_topic_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Right joint PRBS cmd topic: " << right_joint_cmd_topic_);

        left_joint_prbs_generator_ = std::make_shared<PRBSGenerator>(num_of_bits_, left_joint_prbs_seed_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Initializing left joint PRBS generator with seed: " << left_joint_prbs_seed_);

        right_joint_prbs_generator_ = std::make_shared<PRBSGenerator>(num_of_bits_, right_joint_prbs_seed_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Initializing right joint PRBS generator with seed: " << right_joint_prbs_seed_);

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
                                         std::bind(&PRBSGeneratorNode::publishCurrentSequenceValue, this));
    }

private:
    void publishCurrentSequenceValue()
    {
        // Add the offset to the PRBS signal
        double left_effort = 2 * (left_joint_prbs_generator_->getNextValue() - 0.5) * signal_amplitude_ + offset_;
        double right_effort = 2 * (right_joint_prbs_generator_->getNextValue() - 0.5) * signal_amplitude_ + offset_;

        if (!is_float32_)
        {
            auto left_joint_message = std_msgs::msg::Float64MultiArray();
            auto right_joint_message = std_msgs::msg::Float64MultiArray();

            left_joint_message.data.resize(1);
            right_joint_message.data.resize(1);

            left_joint_message.data[0] = left_effort;
            right_joint_message.data[0] = right_effort;

            left_joint_cmd_pub_64_->publish(left_joint_message);
            right_joint_cmd_pub_64_->publish(right_joint_message);
        }
        else
        {
            auto left_joint_message = std_msgs::msg::Float32();
            auto right_joint_message = std_msgs::msg::Float32();

            left_joint_message.data = left_effort;
            right_joint_message.data = right_effort;

            left_joint_cmd_pub_32_->publish(left_joint_message);
            right_joint_cmd_pub_32_->publish(right_joint_message);
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
    int num_of_bits_;
    double signal_frequency_;
    double signal_amplitude_;
    double offset_; // Offset parameter
    uint32_t left_joint_prbs_seed_;
    uint32_t right_joint_prbs_seed_;
    std::string left_joint_cmd_topic_;
    std::string right_joint_cmd_topic_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_joint_cmd_pub_32_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_joint_cmd_pub_32_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_joint_cmd_pub_64_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_joint_cmd_pub_64_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<PRBSGenerator> left_joint_prbs_generator_;
    std::shared_ptr<PRBSGenerator> right_joint_prbs_generator_;

    std::mt19937 gen;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PRBSGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}
