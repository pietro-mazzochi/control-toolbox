// Copyright (c) 2024, ISI SIM
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "pid_controllers/digital_pid_controller.hpp"
#include "pid_controllers_msgs/msg/pid_output.hpp"

class PIDControllerNode : public rclcpp::Node
{
public:
    PIDControllerNode() : Node("pid_controller_node")
    {
        this->declare_parameter<double>("kp");
        this->declare_parameter<double>("ki");
        this->declare_parameter<double>("kd");
        this->declare_parameter<double>("dt");
        this->declare_parameter<double>("umax");
        this->declare_parameter<double>("umin");
        this->declare_parameter<double>("Nfilter");

        this->declare_parameter<std::string>("cmd_topic");
        this->declare_parameter<std::string>("ref_topic");
        this->declare_parameter<std::string>("output_topic");
        this->declare_parameter<std::string>("measurement_topic");

        std::string cmd_topic_ = this->get_parameter("cmd_topic").as_string();
        std::string ref_topic_ = this->get_parameter("ref_topic").as_string();
        std::string output_topic_ = this->get_parameter("output_topic").as_string();
        std::string measurement_topic_ = this->get_parameter("measurement_topic").as_string();

        double kp = this->get_parameter("kp").as_double();
        double ki = this->get_parameter("ki").as_double();
        double kd = this->get_parameter("kd").as_double();
        double dt = this->get_parameter("dt").as_double();
        double umax = this->get_parameter("umax").as_double();
        double umin = this->get_parameter("umin").as_double();
        double N = this->get_parameter("Nfilter").as_double();

        pid_ = std::make_shared<PIDControllers::DigitalPID>(kp, ki, kd, dt, umin, umax, N);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DigitalPID");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - Node: %s", this->get_name());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - Reference: %s (std_msgs::msg::Float64MultiArray[0])", ref_topic_.c_str());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - Control action: %s  (std_msgs::msg::Float64MultiArray[0])", cmd_topic_.c_str());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - Controlled variable: %s (std_msgs::msg::Float64MultiArray[0])", measurement_topic_.c_str());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - Info: %s  (std_msgs::msg::Float64MultiArray[0])", output_topic_.c_str());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - Kp: %f", kp);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - Ki: %f", ki);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - Kd: %f", kd);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - N: %f", N);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - dt: %f", dt);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - umax: %f", umax);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - umin: %f", umin);

        cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(cmd_topic_, 1);
        ref_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(ref_topic_, 1, std::bind(&PIDControllerNode::setpoint_callback, this, std::placeholders::_1));
        measurement_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(measurement_topic_, 1, std::bind(&PIDControllerNode::measurement_callback, this, std::placeholders::_1));
        output_pub_ = this->create_publisher<pid_controllers_msgs::msg::PIDOutput>(output_topic_, 1);

        timer_ = this->create_wall_timer(std::chrono::duration<double>(dt), std::bind(&PIDControllerNode::control_loop, this));
    }

private:
    void control_loop(void)
    {
        auto cmd_msg_ = std_msgs::msg::Float64MultiArray();
        cmd_msg_.data.resize(1);
        cmd_msg_.data[0] = pid_->compute(setpoint_, measured_value_);
        cmd_pub_->publish(cmd_msg_);

        auto output_msg_ = pid_controllers_msgs::msg::PIDOutput();
        output_msg_.ref = setpoint_;
        output_msg_.measurement = measured_value_;
        output_msg_.control = cmd_msg_.data[0];
        output_msg_.error = pid_->get_error();
        output_msg_.derror = pid_->get_derivative_error();
        output_msg_.ierror = pid_->get_integral_error();

        output_pub_->publish(output_msg_);
    }

    void setpoint_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        setpoint_ = msg->data[0];
    }

    void measurement_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        measured_value_ = msg->data[0];
    }

    double setpoint_ = 0.0;
    double measured_value_ = 0.0;

    PIDControllers::DigitalPidControllerPtr pid_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr measurement_sub_;
    rclcpp::Publisher<pid_controllers_msgs::msg::PIDOutput>::SharedPtr output_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDControllerNode>());
    rclcpp::shutdown();
    return 0;
}
