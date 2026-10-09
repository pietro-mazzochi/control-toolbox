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

using namespace std::chrono_literals;

class TempGeneratorNode : public rclcpp::Node
{
public:
    TempGeneratorNode() : Node("step_generator_node")
    {
        this->declare_parameter<bool>("is_float32", false);
        this->declare_parameter<std::string>("left_joint_cmd_topic", "/left_step_seq");
        this->declare_parameter<std::string>("right_joint_cmd_topic", "/right_step_seq");
        this->declare_parameter<double>("left_cmd", 1.0);
        this->declare_parameter<double>("right_cmd", 1.0);

        this->get_parameter("is_float32", is_float32_);
        this->get_parameter("left_joint_cmd_topic", left_joint_cmd_topic_);
        this->get_parameter("right_joint_cmd_topic", right_joint_cmd_topic_);
        this->get_parameter("left_cmd", left_cmd_);
        this->get_parameter("right_cmd", right_cmd_);

        RCLCPP_INFO_STREAM(this->get_logger(), "Left joint step cmd topic: " << left_joint_cmd_topic_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Right joint step cmd topic: " << right_joint_cmd_topic_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Left joint step cmd value: " << left_cmd_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Right joint step cmd value: " << right_cmd_);

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

        timer_ = this->create_wall_timer(1000ms, std::bind(&TempGeneratorNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        double left_effort_ = left_cmd_;
        double right_effort_ = right_cmd_;

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
    double left_cmd_;
    double right_cmd_;
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
    rclcpp::spin(std::make_shared<TempGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}
